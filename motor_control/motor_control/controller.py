import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from rcl_interfaces.msg import SetParametersResult
import numpy as np


class ControllerNode(Node):
    """
    Nodo /Control — Final Challenge MCR2
    -------------------------------------------------------
    Suscribe:  /set_point     (Float32, rad/s) desde /Input
               /motor_output  (Float32, rad/s) desde ESP32
    Publica:   /motor_input_u (Float32, u normalizada [-u_max, u_max])
    -------------------------------------------------------
    PID discreto implementado desde cero con NumPy:
      u[k] = Kp*e[k] + Ki*Ts*sum(e) + Kd*(e[k]-e[k-1])/Ts

    Anti-windup: el integrador solo acumula cuando u no está saturada.
    Tuning en vivo via rqt_reconfigure o ros2 param set.
    """

    def __init__(self):
        super().__init__('ctrl')

        # --- Parámetros PID ---
        self.declare_parameter('sample_time', 0.1)
        self.declare_parameter('kp',   1.0)
        self.declare_parameter('ki',   0.0)
        self.declare_parameter('kd',   0.0)
        self.declare_parameter('u_max',  10.0)
        self.declare_parameter('u_min', -10.0)

        self.sample_time = self.get_parameter('sample_time').value
        self.kp          = self.get_parameter('kp').value
        self.ki          = self.get_parameter('ki').value
        self.kd          = self.get_parameter('kd').value
        self.u_max       = self.get_parameter('u_max').value
        self.u_min       = self.get_parameter('u_min').value

        # --- Variables internas PID ---
        self.set_point  = np.float64(0.0)
        self.motor_y    = np.float64(0.0)
        self.error      = np.float64(0.0)
        self.prev_error = np.float64(0.0)
        self.integral   = np.float64(0.0)
        self.control_u  = np.float64(0.0)
        self.ctrl_active = True

        # --- Publishers ---
        # Salida del PID hacia motor.py (que convierte a PWM)
        self.control_pub = self.create_publisher(Float32, '/motor_input_u', 10)

        # --- Subscribers ---
        # /set_point viene del nodo /Input (global)
        self.sp_sub = self.create_subscription(
            Float32, '/set_point', self.sp_cb, 10)

        # /motor_output viene del ESP32 (/motor_vel remapeado en launch)
        self.motor_sub = self.create_subscription(
            Float32, '/motor_output', self.motor_cb, 10)

        # --- Servicio activación ---
        self.ctrl_activate_srv = self.create_service(
            SetBool, 'ctrl_activate', self.ctrl_activate_cb)

        # --- Timer de control ---
        self.timer = self.create_timer(self.sample_time, self.timer_cb)

        # --- Callback parámetros en vivo ---
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info('=/Control Node Started (PID en PC)=')
        self.get_logger().info(
            f'  Kp={self.kp}  Ki={self.ki}  Kd={self.kd}')
        self.get_logger().info(
            f'  Ts={self.sample_time}s  u=[{self.u_min}, {self.u_max}]')

    # ------------------------------------------------------------------
    # Subscriber Callbacks
    # ------------------------------------------------------------------
    def sp_cb(self, msg):
        """Set-point deseado en rad/s desde /Input."""
        self.set_point = np.float64(msg.data)

    def motor_cb(self, msg):
        """Velocidad medida del motor (rad/s) desde ESP32."""
        self.motor_y = np.float64(msg.data)

    # ------------------------------------------------------------------
    # Timer Callback — PID discreto
    # u[k] = Kp*e[k] + Ki*Ts*sum(e) + Kd*(e[k]-e[k-1])/Ts
    # ------------------------------------------------------------------
    def timer_cb(self):
        if not self.ctrl_active:
            self.control_u = np.float64(0.0)
            msg_out = Float32()
            msg_out.data = 0.0
            self.control_pub.publish(msg_out)
            return

        # 1. Error
        self.error = self.set_point - self.motor_y

        # 2. Integral con anti-windup
        #    Solo acumula cuando la salida NO está saturada
        if self.u_min < self.control_u < self.u_max:
            self.integral += self.error * np.float64(self.sample_time)

        # 3. Derivada (diferencia hacia atrás)
        derivative = (self.error - self.prev_error) / np.float64(self.sample_time)

        # 4. Salida PID
        self.control_u = (self.kp * self.error
                          + self.ki * self.integral
                          + self.kd * derivative)

        # 5. Saturación
        self.control_u = np.clip(self.control_u, self.u_min, self.u_max)

        # 6. Guarda error para próxima iteración
        self.prev_error = self.error

        # 7. Publica señal de control
        msg_out = Float32()
        msg_out.data = float(self.control_u)
        self.control_pub.publish(msg_out)

    # ------------------------------------------------------------------
    # Service Callback — activar / desactivar controlador
    # ------------------------------------------------------------------
    def ctrl_activate_cb(self, request, response):
        self.ctrl_active = request.data
        if self.ctrl_active:
            response.success = True
            response.message = 'Controlador activado.'
            self.get_logger().info('Controlador activado.')
            return response
        # Al desactivar, reinicia estado PID
        self.integral   = np.float64(0.0)
        self.prev_error = np.float64(0.0)
        self.control_u  = np.float64(0.0)
        response.success = True
        response.message = 'Controlador desactivado. Estado PID reiniciado.'
        self.get_logger().info('Controlador desactivado.')
        return response

    # ------------------------------------------------------------------
    # Parameter Callback — tuning en vivo
    # ------------------------------------------------------------------
    def parameters_callback(self, params):
        for param in params:

            if param.name == 'kp':
                if param.value < 0.0:
                    return SetParametersResult(
                        successful=False, reason='kp debe ser >= 0')
                self.kp = param.value
                self.get_logger().info(f'kp -> {self.kp}')

            elif param.name == 'ki':
                if param.value < 0.0:
                    return SetParametersResult(
                        successful=False, reason='ki debe ser >= 0')
                self.ki = param.value
                self.integral = np.float64(0.0)   # reset integrador
                self.get_logger().info(f'ki -> {self.ki}  (integrador reiniciado)')

            elif param.name == 'kd':
                if param.value < 0.0:
                    return SetParametersResult(
                        successful=False, reason='kd debe ser >= 0')
                self.kd = param.value
                self.get_logger().info(f'kd -> {self.kd}')

            elif param.name == 'u_max':
                if param.value <= self.u_min:
                    return SetParametersResult(
                        successful=False, reason='u_max debe ser > u_min')
                self.u_max = param.value
                self.get_logger().info(f'u_max -> {self.u_max}')

            elif param.name == 'u_min':
                if param.value >= self.u_max:
                    return SetParametersResult(
                        successful=False, reason='u_min debe ser < u_max')
                self.u_min = param.value
                self.get_logger().info(f'u_min -> {self.u_min}')

            elif param.name == 'sample_time':
                if param.value <= 0.0:
                    return SetParametersResult(
                        successful=False, reason='sample_time debe ser > 0')
                self.sample_time = param.value
                self.get_logger().info(f'sample_time -> {self.sample_time} s')

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()