from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='geolocation_kf_25',
            executable='geolocation',
            name='geolocation_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='rohang25_test',
            executable='offboard_control_test', 
            name='offboard_test_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='gimbal_control_25',
            executable='gimbal_control_node', 
            name='gimbal_control_node',
            output='screen',
            emulate_tty=True,
        )
        # px4_msgs is a message-only package, no node needed
    ])
