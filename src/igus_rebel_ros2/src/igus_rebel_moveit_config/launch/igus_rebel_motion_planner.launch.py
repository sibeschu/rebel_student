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

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([camera_dir, 'launch', 'rs_launch.py'])
        ),
        launch_arguments={
        'depth_module.depth_profile': '640x480x30',
        'depth_module.infra_profile': '640x480x30',
        'rgb_camera.color_profile': '640x480x30',
        'enable_rgbd': 'true',
        'enable_sync': 'true',
        'align_depth.enable': 'true',
        'pointcloud.enable': 'true',
        'pointcloud.stream_filter': '2',
        'publish_tf': 'true',
        'camera_namespace': 'rebel',
        'camera_name': 'camera',
        }.items(),
    )

    puck_opencv_node = Node(
        package='igus_student',
        executable='puck_opencv',
        name='puck_opencv',
        output='screen'
    )

    puck_2d_to_3d_node = Node(
        package='igus_student',
        executable='puck_2d_to_3d',
        name='puck_2d_to_3d',
        output='screen'
    )

    puck_3d_to_world_node = Node(
        package='igus_student',
        executable='puck_3d_to_world',
        name='puck_3d_to_world',
        output='screen'
    )


    return LaunchDescription([
        debug_arg,
        load_robot_description_arg,
        use_gui_arg,
        moveit_launch,
        realsense_launch,
        puck_opencv_node,
        puck_2d_to_3d_node,
        puck_3d_to_world_node
    ])
