import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from rcl_interfaces.msg import SetParametersResult


# Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        # Name must match launch file: 'sp_gen'
        super().__init__('sp_gen')

        # Declare parameters (readable from YAML / rqt_reconfigure)
        self.declare_parameter('amplitude', 2.0)
        self.declare_parameter('omega', 1.0)
        self.declare_parameter('sample_time', 0.1)
        # signal_type: 0 = sine, 1 = square, 2 = step
        self.declare_parameter('signal_type', 0)

        # Read parameters
        self.amplitude   = self.get_parameter('amplitude').value
        self.omega       = self.get_parameter('omega').value
        self.sample_time = self.get_parameter('sample_time').value
        self.signal_type = self.get_parameter('signal_type').value

        # Publisher and Timer
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)
        self.timer = self.create_timer(self.sample_time, self.timer_cb)

        # Service clients to activate motor and controller
        self.motor_activate_client = self.create_client(SetBool, 'motor_activate')
        self.ctrl_activate_client = self.create_client(SetBool, 'ctrl_activate')

        # One-shot startup timer (1s) to activate dependent nodes
        self.startup_timer = self.create_timer(1.0, self.startup_activate_cb)

        # Message and start time
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        # Parameter callback for runtime tuning
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info('SetPoint Node Started 🚀')
        self.get_logger().info(
            f'  signal_type={self.signal_type} (0=sine, 1=square, 2=step)  '
            f'A={self.amplitude}  w={self.omega}')

    # ------------------------------------------------------------------
    # Timer Callback
    # ------------------------------------------------------------------
    def timer_cb(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if self.signal_type == 0:
            # Sinusoidal
            value = self.amplitude * np.sin(self.omega * elapsed)

        elif self.signal_type == 1:
            # Square wave using sign of sine
            value = self.amplitude * float(np.sign(np.sin(self.omega * elapsed)))

        else:
            # Step (constant)
            value = self.amplitude

        self.signal_msg.data = float(value)
        self.signal_publisher.publish(self.signal_msg)

    # ------------------------------------------------------------------
    # Startup activation callback
    # ------------------------------------------------------------------
    def startup_activate_cb(self):
        if not self.motor_activate_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn('Service motor_activate not available yet.')
            return

        if not self.ctrl_activate_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn('Service ctrl_activate not available yet.')
            return

        req_motor = SetBool.Request()
        req_motor.data = True
        self.motor_activate_client.call_async(req_motor)

        req_ctrl = SetBool.Request()
        req_ctrl.data = True
        self.ctrl_activate_client.call_async(req_ctrl)

        self.get_logger().info('Requested activation: motor_activate=True, ctrl_activate=True')
        self.startup_timer.cancel()

    # ------------------------------------------------------------------
    # Parameter Callback
    # ------------------------------------------------------------------
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'amplitude':
                self.amplitude = param.value
                self.get_logger().info(f'amplitude updated to {self.amplitude}')

            elif param.name == 'omega':
                if param.value <= 0.0:
                    self.get_logger().warn('Invalid omega! Must be > 0.')
                    return SetParametersResult(successful=False, reason='omega must be > 0')
                self.omega = param.value
                self.get_logger().info(f'omega updated to {self.omega}')

            elif param.name == 'signal_type':
                if param.value not in [0, 1, 2]:
                    self.get_logger().warn('Invalid signal_type! Must be 0 (sine), 1 (square) or 2 (step).')
                    return SetParametersResult(
                        successful=False, reason='signal_type must be 0, 1, or 2')
                self.signal_type = param.value
                self.get_logger().info(
                    f'signal_type updated to {self.signal_type} '
                    f'(0=sine, 1=square, 2=step)')

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
    set_point = SetPointPublisher()
    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()             