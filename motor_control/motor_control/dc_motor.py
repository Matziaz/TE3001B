# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from rcl_interfaces.msg import SetParametersResult
import numpy as np


# Class Definition
class DCMotor(Node):
    def __init__(self):
        # NOTE: No hardcoded name here — the launch file sets it to 'motor_sys'
        super().__init__('motor_sys')

        # Declare parameters
        self.declare_parameter('sample_time', 0.01)   # System sample time in seconds
        self.declare_parameter('sys_gain_K', 2.16)    # System gain K
        self.declare_parameter('sys_tau_T', 0.05)     # System time constant Tau
        self.declare_parameter('initial_conditions', 0.0)

        # Read parameters
        self.sample_time        = self.get_parameter('sample_time').value
        self.param_K            = self.get_parameter('sys_gain_K').value
        self.param_T            = self.get_parameter('sys_tau_T').value
        self.initial_conditions = self.get_parameter('initial_conditions').value

        # State variable
        self.input_u  = np.float64(0.0)
        self.output_y = np.float64(self.initial_conditions)
        self.motor_active = True

        # Message
        self.motor_output_msg = Float32()

        # Publishers, Subscribers, Timer
        self.motor_input_sub = self.create_subscription(
            Float32, 'motor_input_u', self.input_callback, 10)
        self.motor_speed_pub = self.create_publisher(Float32, 'motor_output_y', 10)
        self.motor_activate_srv = self.create_service(SetBool, 'motor_activate', self.motor_activate_cb)
        self.timer = self.create_timer(self.sample_time, self.timer_cb)

        # Parameter callback for runtime tuning
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info('DC Motor Node Started 🚀')
        self.get_logger().info(f'  K={self.param_K}  T={self.param_T}  Ts={self.sample_time}')

    # ------------------------------------------------------------------
    # Timer Callback — Euler integration of first-order system
    # y[k+1] = y[k] + ((-1/T)*y[k] + (K/T)*u[k]) * Ts
    # ------------------------------------------------------------------
    def timer_cb(self):
        if not self.motor_active:
            return

        self.output_y += ((-1.0 / self.param_T) * self.output_y
                          + (self.param_K / self.param_T) * self.input_u) * self.sample_time
        self.motor_output_msg.data = float(self.output_y)
        self.motor_speed_pub.publish(self.motor_output_msg)

    # Subscriber Callback
    def input_callback(self, msg):
        self.input_u = np.float64(msg.data)

    # Service Callback
    def motor_activate_cb(self, request, response):
        self.motor_active = request.data

        if self.motor_active:
            response.success = True
            response.message = 'Motor simulation activated.'
            self.get_logger().info('Motor simulation activated.')
            return response

        self.output_y = np.float64(0.0)
        response.success = True
        response.message = 'Motor simulation deactivated. Output reset to 0.0.'
        self.get_logger().info('Motor simulation deactivated. output_y reset to 0.0.')
        return response

    # Parameter Callback
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'sys_gain_K':
                if param.value < 0.0:
                    self.get_logger().warn('Invalid sys_gain_K! Cannot be negative.')
                    return SetParametersResult(successful=False, reason='sys_gain_K cannot be negative')
                self.param_K = param.value
                self.get_logger().info(f'sys_gain_K updated to {self.param_K}')

            elif param.name == 'sys_tau_T':
                if param.value <= 0.0:
                    self.get_logger().warn('Invalid sys_tau_T! Must be > 0.')
                    return SetParametersResult(successful=False, reason='sys_tau_T must be > 0')
                self.param_T = param.value
                self.get_logger().info(f'sys_tau_T updated to {self.param_T}')

            elif param.name == 'sample_time':
                if param.value <= 0.0:
                    self.get_logger().warn('Invalid sample_time! Must be > 0.')
                    return SetParametersResult(successful=False, reason='sample_time must be > 0')
                self.sample_time = param.value
                self.get_logger().info(f'sample_time updated to {self.sample_time}')

        return SetParametersResult(successful=True)


# Main
def main(args=None):
    rclpy.init(args=args)
    node = DCMotor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()