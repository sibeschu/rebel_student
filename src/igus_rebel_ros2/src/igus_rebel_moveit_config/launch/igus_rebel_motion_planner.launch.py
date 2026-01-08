from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='false', description='')
    load_robot_description_arg = DeclareLaunchArgument(
        'load_robot_description', default_value='false', description='')
    use_gui_arg = DeclareLaunchArgument(
        'use_gui', default_value='true', description='')
    
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('igus_rebel_moveit_config'), 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_gui': LaunchConfiguration('use_gui'),
        }.items(),
    )

    camera_dir = FindPackageShare('realsense2_camera')

    # camera_to_link6_rel = TimerAction(
    # period=10.0, # delay in s
    # actions=[
    # Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['-0.066', '0', '0', '0', '-1.57', '0', 'link6', 'camera_link']
    #     )]
    # )


    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([camera_dir, 'launch', 'rs_launch.py'])
        ),
        launch_arguments={
        'depth_module.depth_profile': '424x240x30',
        'depth_module.infra_profile': '424x240x30',
        'rgb_camera.color_profile': '424x240x30',
        'enable_rgbd': 'true',
        'enable_sync': 'true',
        'align_depth.enable': 'true',
        'pointcloud.enable': 'true',
        'pointcloud.stream_filter': '2',
        'publish_tf': 'true',
        'camera_namespace': 'rebel',
        'camera_name': 'camera',

        # # FORCE frame naming so your static mount connects
        # 'base_frame_id': 'camera_link',
        # 'color_frame_id': 'camera_color_frame',
        # 'color_optical_frame_id': 'camera_color_optical_frame',
        # 'depth_frame_id': 'camera_depth_frame',
        # 'depth_optical_frame_id': 'camera_depth_optical_frame',
        }.items(),
    )



    return LaunchDescription([
        debug_arg,
        load_robot_description_arg,
        use_gui_arg,
        moveit_launch,
        realsense_launch,
        # camera_to_link6_rel,
    ])
