# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from rcl_interfaces.msg import SetParametersResult
import numpy as np

# Class Definition
class ControllerNode(Node):
    def __init__(self):
        # Name must match launch file: 'ctrl'
        super().__init__('ctrl')

        # --- Declare Parameters ---
        self.declare_parameter('sample_time', 0.01)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('u_max',  10.0)
        self.declare_parameter('u_min', -10.0)

        # --- Read Parameters ---
        self.sample_time = self.get_parameter('sample_time').value
        self.kp          = self.get_parameter('kp').value
        self.ki          = self.get_parameter('ki').value
        self.kd          = self.get_parameter('kd').value
        self.u_max       = self.get_parameter('u_max').value
        self.u_min       = self.get_parameter('u_min').value

        # --- PID Internal Variables ---
        self.set_point  = np.float64(0.0)
        self.motor_y    = np.float64(0.0)
        self.error      = np.float64(0.0)
        self.prev_error = np.float64(0.0)
        self.integral   = np.float64(0.0)
        self.control_u  = np.float64(0.0)
        self.ctrl_active = True

        # --- Publishers & Subscribers ---
        self.control_pub = self.create_publisher(Float32, 'motor_input_u', 10)

        # Subscribe to set_point from sp_gen node
        self.sp_sub = self.create_subscription(
            Float32, 'set_point', self.sp_cb, 10)

        # Subscribe to motor output — topic published by dc_motor node
        self.motor_sub = self.create_subscription(
            Float32, 'motor_output_y', self.motor_cb, 10)

        # Service to activate/deactivate controller
        self.ctrl_activate_srv = self.create_service(SetBool, 'ctrl_activate', self.ctrl_activate_cb)

        # --- Control Timer ---
        self.timer = self.create_timer(self.sample_time, self.timer_cb)

        # --- Parameter Callback ---
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info('Controller Node Started 🚀')
        self.get_logger().info(f'  Kp={self.kp}  Ki={self.ki}  Kd={self.kd}')
        self.get_logger().info(f'  sample_time={self.sample_time}s  u=[{self.u_min}, {self.u_max}]')

    # ------------------------------------------------------------------
    # Subscriber Callbacks
    # ------------------------------------------------------------------
    def sp_cb(self, msg):
        """Receives desired set-point from sp_gen."""
        self.set_point = np.float64(msg.data)

    def motor_cb(self, msg):
        """Receives measured motor speed from motor_sys."""
        self.motor_y = np.float64(msg.data)

    # ------------------------------------------------------------------
    # Timer Callback — Discrete PID
    # u[k] = Kp*e[k] + Ki*Ts*sum(e) + Kd*(e[k]-e[k-1])/Ts
    # ------------------------------------------------------------------
    def timer_cb(self):
        if not self.ctrl_active:
            self.control_u = np.float64(0.0)
            msg_out = Float32()
            msg_out.data = 0.0
            self.control_pub.publish(msg_out)
            return

        # 1. Compute error
        self.error = self.set_point - self.motor_y

        # 2. Integral with anti-windup (only accumulate when not saturated)
        if self.u_min < self.control_u < self.u_max:
            self.integral += self.error * np.float64(self.sample_time)

        # 3. Derivative (backward difference)
        derivative = (self.error - self.prev_error) / np.float64(self.sample_time)

        # 4. PID output
        self.control_u = (self.kp * self.error
                          + self.ki * self.integral
                          + self.kd * derivative)

        # 5. Saturation
        self.control_u = np.clip(self.control_u, self.u_min, self.u_max)

        # 6. Save error for next iteration
        self.prev_error = self.error

        # 7. Publish control signal
        msg_out = Float32()
        msg_out.data = float(self.control_u)
        self.control_pub.publish(msg_out)

    # ------------------------------------------------------------------
    # Service Callback — controller activation
    # ------------------------------------------------------------------
    def ctrl_activate_cb(self, request, response):
        self.ctrl_active = request.data

        if self.ctrl_active:
            response.success = True
            response.message = 'Controller activated.'
            self.get_logger().info('Controller activated.')
            return response

        self.integral = np.float64(0.0)
        self.prev_error = np.float64(0.0)
        self.control_u = np.float64(0.0)

        response.success = True
        response.message = 'Controller deactivated. PID state reset.'
        self.get_logger().info('Controller deactivated. PID state reset.')
        return response

    # ------------------------------------------------------------------
    # Parameter Callback — runtime tuning via rqt_reconfigure
    # ------------------------------------------------------------------
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'kp':
                if param.value < 0.0:
                    self.get_logger().warn('Invalid kp! Must be >= 0.')
                    return SetParametersResult(successful=False, reason='kp must be >= 0')
                self.kp = param.value
                self.get_logger().info(f'kp updated to {self.kp}')

            elif param.name == 'ki':
                if param.value < 0.0:
                    self.get_logger().warn('Invalid ki! Must be >= 0.')
                    return SetParametersResult(successful=False, reason='ki must be >= 0')
                self.ki = param.value
                self.integral = np.float64(0.0)  # Reset integral on ki change
                self.get_logger().info(f'ki updated to {self.ki}  (integral reset)')

            elif param.name == 'kd':
                if param.value < 0.0:
                    self.get_logger().warn('Invalid kd! Must be >= 0.')
                    return SetParametersResult(successful=False, reason='kd must be >= 0')
                self.kd = param.value
                self.get_logger().info(f'kd updated to {self.kd}')

            elif param.name == 'u_max':
                if param.value <= self.u_min:
                    self.get_logger().warn('Invalid u_max! Must be > u_min.')
                    return SetParametersResult(successful=False, reason='u_max must be > u_min')
                self.u_max = param.value
                self.get_logger().info(f'u_max updated to {self.u_max}')

            elif param.name == 'u_min':
                if param.value >= self.u_max:
                    self.get_logger().warn('Invalid u_min! Must be < u_max.')
                    return SetParametersResult(successful=False, reason='u_min must be < u_max')
                self.u_min = param.value
                self.get_logger().info(f'u_min updated to {self.u_min}')

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
    controller = ControllerNode()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()