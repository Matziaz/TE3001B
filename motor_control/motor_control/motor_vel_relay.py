"""
motor_vel_relay.py — Final Challenge MCR2
==========================================
Relay mínimo: copia /motor_vel (publicado por el ESP32)
hacia /motor_output (que escucha el nodo /ctrl).

Por qué existe:
  El ESP32 publica en /motor_vel (nombre fijo en el firmware Arduino).
  El controlador escucha /motor_output (nombre del reto).
  Este relay desacopla ambos nombres sin tocar el firmware del ESP32.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class MotorVelRelay(Node):
    def __init__(self):
        super().__init__('motor_vel_relay')

        self.pub = self.create_publisher(Float32, '/motor_output', 10)

        self.sub = self.create_subscription(
            Float32,
            '/motor_vel',
            self.cb,
            10
        )
        self.get_logger().info('motor_vel_relay: /motor_vel → /motor_output')

    def cb(self, msg: Float32):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()