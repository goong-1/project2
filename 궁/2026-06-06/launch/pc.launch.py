from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='p2_pkg',
            executable='cam_line_node',
            name='vision_node',
            output='screen',
        ),

        Node(
            package='p2_pkg',
            executable='cam_yolo_node',
            name='cam_yolo_node',
            output='screen',
        ),

        Node(
            package='p2_pkg',
            executable='brain_node',
            name='control_handler',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='p2_pkg',
            executable='dashboard_node',
            name='dashboard_node',
            output='screen',
        ),
    ])