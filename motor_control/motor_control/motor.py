# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import numpy as np


class MotorNode(Node):
    """
    Nodo /motor
    -------------------------------------------------------
    Suscribe:
      /motor_input_u   (Float32, u en rad/s aprox ±u_max)

    Publica:
      /cmd_pwm         (Int32, PWM firmado ±255 para ESP32)
      /cmd_pwm_norm    (Float32, magnitud normalizada 0..1)

    Objetivo de esta versión:
    - quitar saltos bruscos por pwm_min alto y kick de arranque
    - mantener un mapeo limpio y proporcional u -> PWM
    - limitar la pendiente del PWM para no castigar motor/driver
    """

    def __init__(self):
        super().__init__('motor')

        # --- Parámetros ---
        self.declare_parameter('u_max', 15.0)
        self.declare_parameter('sample_time', 0.1)

        # Pequeña zona muerta para evitar vibración cerca de cero
        self.declare_parameter('u_deadband', 0.15)

        # PWM mínimo. Déjalo en 0 para que el mapeo sea realmente proporcional.
        self.declare_parameter('pwm_min', 0)

        # Límite de cambio del PWM en "cuentas por segundo"
        # Ejemplo: 400 con Ts=0.1 => máximo 40 cuentas por muestra
        self.declare_parameter('pwm_slew_rate', 400.0)

        self.u_max = float(self.get_parameter('u_max').value)
        self.sample_time = float(self.get_parameter('sample_time').value)
        self.u_deadband = float(self.get_parameter('u_deadband').value)
        self.pwm_min = int(self.get_parameter('pwm_min').value)
        self.pwm_slew_rate = float(self.get_parameter('pwm_slew_rate').value)

        # --- Estado interno ---
        self.input_u = 0.0
        self.last_pwm_f = 0.0

        # --- ROS I/O ---
        self.sub = self.create_subscription(
            Float32,
            'motor_input_u',
            self.input_callback,
            10
        )

        self.pub_pwm = self.create_publisher(Int32, 'cmd_pwm', 10)
        self.pub_pwm_norm = self.create_publisher(Float32, 'cmd_pwm_norm', 10)

        self.timer = self.create_timer(self.sample_time, self.timer_cb)

        self.get_logger().info(
            'Motor Node Started | '
            f'u_max={self.u_max}, '
            f'u_deadband={self.u_deadband}, '
            f'pwm_min={self.pwm_min}, '
            f'pwm_slew_rate={self.pwm_slew_rate}'
        )

    def input_callback(self, msg: Float32):
        self.input_u = float(msg.data)

    def timer_cb(self):
        u = self.input_u

        # 1) Zona muerta en u
        if abs(u) < self.u_deadband:
            target_pwm_f = 0.0
        else:
            # 2) Normalización y saturación
            u_norm = float(np.clip(u / self.u_max, -1.0, 1.0))

            # 3) Mapeo lineal limpio a PWM
            target_pwm_f = 255.0 * u_norm

            # 4) PWM mínimo opcional
            if self.pwm_min > 0 and abs(target_pwm_f) < self.pwm_min:
                target_pwm_f = float(np.sign(target_pwm_f) * self.pwm_min)

        # 5) Limitador de pendiente (slew-rate limiter)
        max_delta = max(1.0, self.pwm_slew_rate * self.sample_time)
        delta = float(np.clip(target_pwm_f - self.last_pwm_f, -max_delta, max_delta))
        applied_pwm_f = self.last_pwm_f + delta

        # Fuerza cero exacto al acercarse al reposo
        if target_pwm_f == 0.0 and abs(applied_pwm_f) < 1.0:
            applied_pwm_f = 0.0

        # 6) Saturación final y publicación
        applied_pwm = int(np.clip(round(applied_pwm_f), -255, 255))
        self.last_pwm_f = float(applied_pwm)

        msg_pwm = Int32()
        msg_pwm.data = applied_pwm
        self.pub_pwm.publish(msg_pwm)

        msg_pwm_norm = Float32()
        msg_pwm_norm.data = abs(applied_pwm) / 255.0
        self.pub_pwm_norm.publish(msg_pwm_norm)


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()