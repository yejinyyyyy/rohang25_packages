from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1) Declare the launch‑time argument
    declare_precision_arg = DeclareLaunchArgument(
        'precision_control',
        default_value='0',
        description='0: pixel-based, 1: NED-based'
    )
    
    declare_video_arg = DeclareLaunchArgument(
        'video_source',
        default_value='real_camera',
        description='real_camera or sim_camera'
    )
    
    return LaunchDescription([
    	declare_precision_arg,
    	declare_video_arg,
    	
        Node(
            package='geolocation_kf_25',
            executable='geolocation',
            name='geolocation_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='rohang25_test',
            executable='offboard_control_test', 
            name='offboard_test_node',
            output='screen',
            emulate_tty=True,
	    parameters=[{'precision_control': LaunchConfiguration('precision_control')}], # 0: Pixel-based control, 1: NED-based controls 
        ),
         Node(
             package='gimbal_control_25',
             executable='gimbal_control_node', 
             name='gimbal_control_node',
             output='screen',
             emulate_tty=True,
         ),
        #Node(
            #package='yolov11',
            #executable='yolov11_node', 
            #name='yolov11_node',
            #output='screen',
            #emulate_tty=True,
            #parameters=[{'video_source': LaunchConfiguration('video_source')}],  # 'real_camera' or 'sim_camera'   
        #)
        # px4_msgs is a message-only package, no node needed
    ])
