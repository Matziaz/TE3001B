"""
challenge_launch.py — Final Challenge MCR2
===========================================
Arquitectura (todo en la PC, ESP32 corre su propio firmware):

  /sp_gen  →  /set_point  →  /ctrl  →  /motor_input_u  →  /motor  →  /cmd_pwm  →  [ESP32]
                                  ↑                                          ↓
                             /motor_output  ←←←←←←←←←  /motor_vel  ←←←  [ESP32]

Topics globales (sin namespace):
  /set_point      Float32   set-point en rad/s
  /motor_output   Float32   velocidad medida en rad/s (remapeada desde /motor_vel del ESP32)
  /motor_input_u  Float32   señal de control u del PID
  /cmd_pwm        Int32     PWM firmado ±255 para el ESP32

Para monitorear en rqt_plot agrega los topics:
  /set_point  /motor_output  /motor_input_u

Para ajuste en vivo de parámetros:
  rqt  →  Plugins → Configuration → Dynamic Reconfigure
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'params.yaml'
    )

    # ------------------------------------------------------------------
    # Nodo /Input — generador de set-point
    # Publica:  /set_point (global)
    # ------------------------------------------------------------------
    input_node = Node(
        name='sp_gen',
        package='motor_control',
        executable='set_point',
        emulate_tty=True,
        output='screen',
        parameters=[params_file],
    )

    # ------------------------------------------------------------------
    # Nodo /Control — PID en la PC
    # Suscribe: /set_point, /motor_output
    # Publica:  /motor_input_u
    # ------------------------------------------------------------------
    control_node = Node(
        name='ctrl',
        package='motor_control',
        executable='controller',
        emulate_tty=True,
        output='screen',
        parameters=[params_file],
    )

    # ------------------------------------------------------------------
    # Nodo motor — convierte u (rad/s) → PWM firmado ±255
    # Suscribe: /motor_input_u
    # Publica:  /cmd_pwm  (recibido por el ESP32 vía micro-ROS)
    # ------------------------------------------------------------------
    motor_node = Node(
        name='motor',
        package='motor_control',
        executable='motor',
        emulate_tty=True,
        output='screen',
        parameters=[params_file],
        remappings=[
            ('motor_input_u', '/motor_input_u'),
            ('cmd_pwm',       '/cmd_pwm'),
        ]
    )

    # ------------------------------------------------------------------
    # Bridge: el ESP32 publica /motor_vel (Float32, rad/s)
    # El controlador escucha /motor_output
    # El remapeo se hace aquí con un relay node usando topic_tools,
    # O más simple: el controller.py ya suscribe /motor_output,
    # y lanzamos un relay que copie /motor_vel → /motor_output.
    #
    # ALTERNATIVA más limpia: nodo relay mínimo inline
    # (no requiere topic_tools, solo rclpy)
    # ------------------------------------------------------------------
    relay_node = Node(
        name='motor_vel_relay',
        package='motor_control',
        executable='motor_vel_relay',   # ver nota abajo (*)
        emulate_tty=True,
        output='screen',
    )

    return LaunchDescription([
        input_node,
        control_node,
        motor_node,
        relay_node,
    ])
