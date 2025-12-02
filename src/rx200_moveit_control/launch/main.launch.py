from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    xsarm_moveit_path = get_package_share_directory('interbotix_xsarm_moveit')

    return LaunchDescription([
        # 1. MoveIt + Hardware
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(xsarm_moveit_path, 'launch', 'xsarm_moveit.launch.py')
            ),
            launch_arguments={
                'robot_model': 'rx200',
                'hardware_type': 'actual',
                'use_world_frame': 'true',
            }.items()
        ),

        # 2. Static TF: Connects the two trees
        #    Note: arguments are x y z yaw pitch roll parent child
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            arguments=[
                '--x', '0.05', 
                '--y', '0.00', 
                '--z', '0.08',
                '--roll', '0.0', 
                '--pitch', '0.9', 
                '--yaw', '0.0',
                '--frame-id', 'rx200/ee_gripper_link',  # Parent frame (from Tree 1)
                '--child-frame-id', 'camera_link'       # Child frame (Root of Tree 2)
            ],
            output='screen'
        ),

        # 3. RealSense Camera
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'align_depth.enable': True,
                # IMPORTANT: This must match the child frame ID above
                'base_frame_id': 'camera_link',
            }],
            output='screen'
        ),

        # 4. Your Nodes
        Node(
            package='rx200_moveit_control',
            executable='manipulation_node',
            name='manipulation_node',
            output='screen'
        ),
        
    ])
