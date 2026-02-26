# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import numpy as np


class MotorNode(Node):
    """
    Nodo /motor — Mini Challenge 3
    -------------------------------------------------------
    Suscribe:  /motor_input_u  (Float32, u en rad/s aprox ±u_max)
    Publica:   /cmd_pwm        (Int32, PWM firmado ±255)
    -------------------------------------------------------
    Extras para motor real:
      - u_deadband: zona muerta en u (rad/s) -> manda PWM=0
      - pwm_min: mínimo PWM cuando u != 0 (vence fricción)
      - kick_pwm + kick_time: "patada" breve al arrancar
    """

    def __init__(self):
        super().__init__('motor')

        # --- Parámetros ---
        self.declare_parameter('u_max', 15.0)
        self.declare_parameter('sample_time', 0.01)

        # Para motor real (ajustables por ROS args)
        self.declare_parameter('u_deadband', 0.3)   # rad/s: si |u|<esto -> 0
        self.declare_parameter('pwm_min', 200)      # mínimo PWM para moverse (tú ya viste 200)
        self.declare_parameter('kick_pwm', 230)     # "patada" al arrancar
        self.declare_parameter('kick_time', 0.15)   # segundos de patada al cambio 0->mov

        self.u_max = float(self.get_parameter('u_max').value)
        self.sample_time = float(self.get_parameter('sample_time').value)

        self.u_deadband = float(self.get_parameter('u_deadband').value)
        self.pwm_min = int(self.get_parameter('pwm_min').value)
        self.kick_pwm = int(self.get_parameter('kick_pwm').value)
        self.kick_time = float(self.get_parameter('kick_time').value)

        # --- Estado interno ---
        self.input_u = 0.0
        self.last_pwm = 0
        self.kick_ticks_left = 0

        # --- ROS I/O ---
        self.sub = self.create_subscription(
            Float32,
            'motor_input_u',
            self.input_callback,
            10
        )
        self.pub = self.create_publisher(Int32, 'cmd_pwm', 10)

        self.timer = self.create_timer(self.sample_time, self.timer_cb)

        self.get_logger().info('Motor Node Started 🚀')
        self.get_logger().info(
            f'u_max={self.u_max} u_deadband={self.u_deadband} pwm_min={self.pwm_min} '
            f'kick_pwm={self.kick_pwm} kick_time={self.kick_time}s'
        )

    def input_callback(self, msg: Float32):
        self.input_u = float(msg.data)

    def timer_cb(self):
        u = self.input_u

        # 0) Zona muerta en u: evita vibración/cacería cerca de 0
        if abs(u) < self.u_deadband:
            pwm = 0
        else:
            # 1) Normaliza y satura
            u_norm = u / self.u_max
            u_norm = float(np.clip(u_norm, -1.0, 1.0))

            # 2) PWM base
            pwm = int(u_norm * 255)

            # 3) Mínimo PWM para vencer fricción (si no es cero)
            if pwm != 0 and abs(pwm) < self.pwm_min:
                pwm = int(np.sign(pwm) * self.pwm_min)

        # 4) Kick: cuando pasas de parado (0) a movimiento, mete patada breve
        # Detecta transición 0 -> no-cero
        if self.last_pwm == 0 and pwm != 0:
            ticks = int(max(1, round(self.kick_time / self.sample_time)))
            self.kick_ticks_left = ticks

        if self.kick_ticks_left > 0:
            pwm = int(np.sign(pwm) * self.kick_pwm)
            self.kick_ticks_left -= 1

        # 5) Publica
        msg = Int32()
        msg.data = int(np.clip(pwm, -255, 255))
        self.pub.publish(msg)

        self.last_pwm = msg.data


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