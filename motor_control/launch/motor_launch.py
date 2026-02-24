from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'params.yaml'
    )

    motor_node = Node(
        name='motor_sys',
        package='motor_control',
        executable='dc_motor',
        emulate_tty=True,
        output='screen',
        parameters=[params_file]
    )

    sp_node = Node(
        name='sp_gen',
        package='motor_control',
        executable='set_point',
        emulate_tty=True,
        output='screen',
        parameters=[params_file]
    )

    ctrl_node = Node(
        name='ctrl',
        package='motor_control',
        executable='controller',
        emulate_tty=True,
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([motor_node, sp_node, ctrl_node])