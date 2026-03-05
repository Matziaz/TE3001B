import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult


class SetPointPublisher(Node):
    """
    Nodo /Input — Final Challenge MCR2
    -------------------------------------------------------
    Publica:  /set_point  (Float32, rad/s)

    Tipos de señal (signal_type):
      0 = Seno        — respuesta continua suave
      1 = Cuadrada    — seguimiento ante cambios bruscos
      2 = Escalón     — caracterización clásica del sistema
      3 = Triangular  — rampa continua, prueba tracking lineal
      4 = Diente de sierra — rampa + caída brusca
      5 = Pulso       — escalón corto repetido, prueba recuperación
    -------------------------------------------------------
    Todos los parámetros son ajustables en vivo via
    rqt_reconfigure o:  ros2 param set /sp_gen <param> <valor>
    """

    # Nombres para logs legibles
    SIGNAL_NAMES = {
        0: 'Seno',
        1: 'Cuadrada',
        2: 'Escalon',
        3: 'Triangular',
        4: 'Diente de sierra',
        5: 'Pulso',
    }

    def __init__(self):
        super().__init__('sp_gen')

        # --- Parámetros ---
        self.declare_parameter('amplitude',    2.0)  # [rad/s] amplitud pico
        self.declare_parameter('omega',        1.0)  # [rad/s] frecuencia angular
        self.declare_parameter('sample_time',  0.1)  # [s]
        self.declare_parameter('signal_type',  2)    # 0-5, ver tabla arriba
        # Parámetro exclusivo del pulso: fracción del periodo que está "alto"
        self.declare_parameter('pulse_duty',   0.3)  # [0.0 – 1.0]

        self.amplitude   = self.get_parameter('amplitude').value
        self.omega       = self.get_parameter('omega').value
        self.sample_time = self.get_parameter('sample_time').value
        self.signal_type = self.get_parameter('signal_type').value
        self.pulse_duty  = self.get_parameter('pulse_duty').value

        # --- Publisher (topic GLOBAL /set_point) ---
        self.signal_publisher = self.create_publisher(Float32, '/set_point', 10)
        self.timer = self.create_timer(self.sample_time, self.timer_cb)

        # --- Tiempo de inicio ---
        self.start_time  = self.get_clock().now()
        self.signal_msg  = Float32()

        # --- Callback de parámetros en vivo ---
        self.add_on_set_parameters_callback(self.parameters_callback)

        self._log_config()

    # ------------------------------------------------------------------
    # Timer Callback — genera la señal según signal_type
    # ------------------------------------------------------------------
    def timer_cb(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        value   = self._compute_signal(elapsed)

        self.signal_msg.data = float(value)
        self.signal_publisher.publish(self.signal_msg)

    # ------------------------------------------------------------------
    # Generador de señales
    # ------------------------------------------------------------------
    def _compute_signal(self, t: float) -> float:
        A = self.amplitude
        w = self.omega

        if self.signal_type == 0:
            # Seno: A * sin(w*t)
            return A * np.sin(w * t)

        elif self.signal_type == 1:
            # Cuadrada: signo del seno
            return A * float(np.sign(np.sin(w * t)))

        elif self.signal_type == 2:
            # Escalón: amplitud constante
            return A

        elif self.signal_type == 3:
            # Triangular: sube linealmente la primera mitad del periodo,
            # baja la segunda. Rango: [-A, +A]
            period = (2.0 * np.pi) / w
            t_mod  = t % period
            if t_mod < period / 2.0:
                return A * (4.0 * t_mod / period - 1.0)
            else:
                return A * (3.0 - 4.0 * t_mod / period)

        elif self.signal_type == 4:
            # Diente de sierra: rampa de -A a +A, luego caída instantánea
            period = (2.0 * np.pi) / w
            t_mod  = t % period
            return A * (2.0 * t_mod / period - 1.0)

        else:
            # Pulso (type == 5): alto durante pulse_duty fracción del periodo
            period = (2.0 * np.pi) / w
            t_mod  = t % period
            return A if t_mod < self.pulse_duty * period else 0.0

    # ------------------------------------------------------------------
    # Parameter Callback — tuning en vivo (rqt_reconfigure compatible)
    # ------------------------------------------------------------------
    def parameters_callback(self, params):
        for param in params:

            if param.name == 'amplitude':
                self.amplitude = param.value
                self.get_logger().info(f'amplitude -> {self.amplitude} rad/s')

            elif param.name == 'omega':
                if param.value <= 0.0:
                    return SetParametersResult(
                        successful=False, reason='omega debe ser > 0')
                self.omega = param.value
                self.get_logger().info(f'omega -> {self.omega} rad/s')

            elif param.name == 'signal_type':
                if param.value not in self.SIGNAL_NAMES:
                    return SetParametersResult(
                        successful=False,
                        reason='signal_type debe ser 0(seno) 1(cuadrada) 2(escalon) '
                               '3(triangular) 4(diente sierra) 5(pulso)')
                self.signal_type = param.value
                # Reinicia tiempo para que la nueva señal empiece desde cero
                self.start_time = self.get_clock().now()
                self.get_logger().info(
                    f'signal_type -> {self.signal_type} '
                    f'({self.SIGNAL_NAMES[self.signal_type]})')

            elif param.name == 'pulse_duty':
                if not (0.0 < param.value < 1.0):
                    return SetParametersResult(
                        successful=False, reason='pulse_duty debe estar en (0.0, 1.0)')
                self.pulse_duty = param.value
                self.get_logger().info(f'pulse_duty -> {self.pulse_duty}')

            elif param.name == 'sample_time':
                if param.value <= 0.0:
                    return SetParametersResult(
                        successful=False, reason='sample_time debe ser > 0')
                self.sample_time = param.value
                self.get_logger().info(f'sample_time -> {self.sample_time} s')

        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Helper log
    # ------------------------------------------------------------------
    def _log_config(self):
        name = self.SIGNAL_NAMES.get(self.signal_type, '?')
        self.get_logger().info('=/Input Node Started=')
        self.get_logger().info(
            f'  signal_type={self.signal_type} ({name})  '
            f'A={self.amplitude} rad/s  omega={self.omega} rad/s  '
            f'Ts={self.sample_time} s  pulse_duty={self.pulse_duty}')


def main(args=None):
    rclpy.init(args=args)
    node = SetPointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()