from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction


def generate_launch_description():

    # ------------------------------------------------------------------ #
    #  GROUP 1 — Sine wave, connected to PHYSICAL motor via remapping    #
    # ------------------------------------------------------------------ #
    group1 = GroupAction([
        PushRosNamespace('group1'),

        # Simulador (se mantiene para comparación, pero NO se usa como feedback)
        Node(
           name='motor_sys',
           package='motor_control',
           executable='dc_motor',
           emulate_tty=True,
           output='screen',
           parameters=[{
                'sample_time': 0.01,
                'sys_gain_K': 2.16,
                'sys_tau_T': 0.05,
                'initial_conditions': 0.0,
           }]
        ),

        # Set point generator
        Node(
            name='sp_gen',
            package='motor_control',
            executable='set_point',
            emulate_tty=True,
            output='screen',
            parameters=[{
                'sample_time': 0.1,
                'amplitude': 2.0,
                'omega': 1.0,
                'signal_type': 0,   # 0 = sine
            }]
        ),

        # Controlador PID — feedback viene del encoder real (/motor_vel)
        Node(
            name='ctrl',
            package='motor_control',
            executable='controller',
            emulate_tty=True,
            output='screen',
            parameters=[{
                'sample_time': 0.01,
                'kp': 1.0,
                'ki': 5.0,
                'kd': 0.01,
                'u_max':  10.0,
                'u_min': -10.0,
            }],
            remappings=[
                # El controlador normalmente lee motor_output_y (simulador)
                # Lo redirigimos al encoder real del ESP32
                ('motor_output_y', '/motor_vel'),
            ]
        ),

        # Nodo motor.py — convierte u (rad/s) a PWM firmado para el ESP32
        Node(
            name='motor',
            package='motor_control',
            executable='motor',
            emulate_tty=True,
            output='screen',
            parameters=[{
                'u_max': 10.0,
                'sample_time': 0.01,
                'u_deadband': 0.3,
                'pwm_min': 200,
                'kick_pwm': 230,
                'kick_time': 0.15,
            }],
            remappings=[
                # motor.py escucha /motor_input_u → lo conectamos a la salida del ctrl
                ('motor_input_u', 'motor_input_u'),
                # motor.py publica /cmd_pwm → va directo al ESP32 (sin namespace)
                ('cmd_pwm', '/cmd_pwm'),
            ]
        ),
    ])

    # ------------------------------------------------------------------ #
    #  GROUP 2 — Square wave, simulador únicamente                       #
    # ------------------------------------------------------------------ #
    group2 = GroupAction([
        PushRosNamespace('group2'),
        Node(
            name='motor_sys',
            package='motor_control',
            executable='dc_motor',
            emulate_tty=True,
            output='screen',
            parameters=[{
                'sample_time': 0.01,
                'sys_gain_K': 2.16,
                'sys_tau_T': 0.05,
                'initial_conditions': 0.0,
            }]
        ),
        Node(
            name='sp_gen',
            package='motor_control',
            executable='set_point',
            emulate_tty=True,
            output='screen',
            parameters=[{
                'sample_time': 0.1,
                'amplitude': 3.0,
                'omega': 0.5,
                'signal_type': 1,   # 1 = square
            }]
        ),
        Node(
            name='ctrl',
            package='motor_control',
            executable='controller',
            emulate_tty=True,
            output='screen',
            parameters=[{
                'sample_time': 0.01,
                'kp': 2.0,
                'ki': 8.0,
                'kd': 0.05,
                'u_max':  10.0,
                'u_min': -10.0,
            }]
        ),
    ])

    # ------------------------------------------------------------------ #
    #  GROUP 3 — Step, PI only, simulador únicamente                     #
    # ------------------------------------------------------------------ #
    group3 = GroupAction([
        PushRosNamespace('group3'),
        Node(
            name='motor_sys',
            package='motor_control',
            executable='dc_motor',
            emulate_tty=True,
            output='screen',
            parameters=[{
                'sample_time': 0.01,
                'sys_gain_K': 2.16,
                'sys_tau_T': 0.05,
                'initial_conditions': 0.0,
            }]
        ),
        Node(
            name='sp_gen',
            package='motor_control',
            executable='set_point',
            emulate_tty=True,
            output='screen',
            parameters=[{
                'sample_time': 0.1,
                'amplitude': 1.5,
                'omega': 1.0,
                'signal_type': 2,   # 2 = step
            }]
        ),
        Node(
            name='ctrl',
            package='motor_control',
            executable='controller',
            emulate_tty=True,
            output='screen',
            parameters=[{
                'sample_time': 0.01,
                'kp': 1.5,
                'ki': 10.0,
                'kd': 0.0,
                'u_max':  10.0,
                'u_min': -10.0,
            }]
        ),
    ])

    return LaunchDescription([group1, group2, group3])