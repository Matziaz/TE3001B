#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include <math.h>

// ===================== Pines =====================
#define EN_A 18
#define EN_B 19

#define IN3 12  // PWM adelante
#define IN4 13  // PWM reversa

// ===================== Encoder =====================
volatile long pulsos = 0;
long pulsos_ant = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR int_callback() {
  if (digitalRead(EN_B) == 0) {
    portENTER_CRITICAL_ISR(&mux);
    pulsos++;
    portEXIT_CRITICAL_ISR(&mux);
  } else {
    portENTER_CRITICAL_ISR(&mux);
    pulsos--;
    portEXIT_CRITICAL_ISR(&mux);
  }
}

// Ajusta a tu encoder real (tu código usaba 408)
static const double PULSOS_POR_VUELTA = 408.0;

// ===================== Muestreo =====================
static const double Ts = 0.10;              // [s]
static const uint32_t Ts_us = 100000;       // 0.10 s en microsegundos
static uint32_t t_prev_us = 0;

static float w_rad_s = 0.0f;

// ===================== Motor driver =====================
static inline int clamp_int(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void setMotorPwmSigned(int pwmSigned) {
  pwmSigned = clamp_int(pwmSigned, -255, 255);

  int pwmVal = abs(pwmSigned);

  // Stop duro
  if (pwmVal == 0) {
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    return;
  }

  if (pwmSigned > 0) {
    // Adelante: PWM en IN3, IN4 apagado
    analogWrite(IN4, 0);
    digitalWrite(IN4, LOW);
    analogWrite(IN3, pwmVal);
  } else {
    // Reversa: PWM en IN4, IN3 apagado
    analogWrite(IN3, 0);
    digitalWrite(IN3, LOW);
    analogWrite(IN4, pwmVal);
  }
}

// ===================== micro-ROS: objetos =====================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_subscription_t sub_cmd_pwm;
std_msgs__msg__Int32 msg_cmd_pwm;

rcl_publisher_t pub_vel;
std_msgs__msg__Float32 msg_vel;

rcl_publisher_t pub_pwm_applied;
std_msgs__msg__Int32 msg_pwm_applied;

rclc_executor_t executor;

// ===================== Callbacks =====================
void cb_cmd_pwm(const void *msgin) {
  const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msgin;
  int pwm = clamp_int((int)m->data, -255, 255);

  setMotorPwmSigned(pwm);

  // Debug: publica el PWM aplicado realmente
  msg_pwm_applied.data = pwm;
  (void)rcl_publish(&pub_pwm_applied, &msg_pwm_applied, NULL);
}

// ===================== Máquina de estados agente =====================
enum agent_state_t {
  WAITING_AGENT = 0,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

agent_state_t state = WAITING_AGENT;

// Limpia entidades ROS cuando se cae el agente
bool destroy_entities() {
  rcl_ret_t rc = RCL_RET_OK;

  rc |= rcl_subscription_fini(&sub_cmd_pwm, &node);
  rc |= rcl_publisher_fini(&pub_vel, &node);
  rc |= rcl_publisher_fini(&pub_pwm_applied, &node);

  rc |= rcl_node_fini(&node);
  rc |= rclc_executor_fini(&executor);
  rc |= rclc_support_fini(&support);

  return (rc == RCL_RET_OK);
}

// Crea entidades ROS cuando el agente aparece
bool create_entities() {
  allocator = rcl_get_default_allocator();

  // soporte
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;

  // nodo
  if (rclc_node_init_default(&node, "esp32_motor_node", "", &support) != RCL_RET_OK) return false;

  // sub /cmd_pwm
  if (rclc_subscription_init_default(
        &sub_cmd_pwm,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/cmd_pwm") != RCL_RET_OK) return false;

  // pub /motor_vel
  if (rclc_publisher_init_default(
        &pub_vel,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/motor_vel") != RCL_RET_OK) return false;

  // pub /motor_pwm_applied
  if (rclc_publisher_init_default(
        &pub_pwm_applied,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/motor_pwm_applied") != RCL_RET_OK) return false;

  // executor
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &sub_cmd_pwm, &msg_cmd_pwm, &cb_cmd_pwm, ON_NEW_DATA) != RCL_RET_OK) return false;

  return true;
}

// ===================== Setup / Loop =====================
void setup() {
  // Pines
  pinMode(EN_A, INPUT);
  pinMode(EN_B, INPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  attachInterrupt(EN_A, int_callback, RISING);

  // Serial micro-ROS
  Serial.begin(115200);
  delay(200);

  // micro-ROS transport serial
  set_microros_transports();

  // Estado inicial motor apagado
  setMotorPwmSigned(0);

  t_prev_us = micros();
}

void loop() {
  // 1) Máquina de estados: conexión con el agente
  switch (state) {
    case WAITING_AGENT:
      // Ping frecuente hasta que aparezca
      if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
        state = AGENT_AVAILABLE;
      }
      break;

    case AGENT_AVAILABLE:
      if (create_entities()) {
        state = AGENT_CONNECTED;
      } else {
        // si falla, espera y vuelve a intentar
        destroy_entities();
        state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      // Si el agente se cae, pasa a disconnected
      if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        state = AGENT_DISCONNECTED;
        break;
      }

      // 2) Ejecuta callbacks (recibir /cmd_pwm)
      (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

      // 3) Muestreo de velocidad y publish
      if ((uint32_t)(micros() - t_prev_us) >= Ts_us) {
        t_prev_us = micros();

        long p;
        portENTER_CRITICAL(&mux);
        p = pulsos;
        portEXIT_CRITICAL(&mux);

        long deltaP = p - pulsos_ant;
        pulsos_ant = p;

        // rad/s = (deltaP / PPR) * 2*pi / Ts
        double w = ((double)deltaP / PULSOS_POR_VUELTA) * (2.0 * M_PI) / Ts;
        w_rad_s = (float)w;

        msg_vel.data = w_rad_s;
        (void)rcl_publish(&pub_vel, &msg_vel, NULL);
      }

      break;

    case AGENT_DISCONNECTED:
      // Apaga motor por seguridad
      setMotorPwmSigned(0);
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}