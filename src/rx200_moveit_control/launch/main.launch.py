from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    xsarm_moveit_path = get_package_share_directory('interbotix_xsarm_moveit')

    return LaunchDescription([
        # 1) MoveIt + robot (same as your CLI command)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(xsarm_moveit_path, 'launch', 'xsarm_moveit.launch.py')
            ),
            launch_arguments={
                'robot_model': 'rx200',
                'hardware_type': 'actual',
            }.items()
        ),

        # 2) RealSense
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_pointcloud': True,
                'align_depth.enable': True,
            }],
            output='screen'
        ),

        # 3) Static TF: ee -> camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_publisher',
            arguments=[
                '0.198', '-0.004', '0.102',
                '-0.010', '1.127', '-0.029',
                'rx200/ee_gripper_link',
                'camera_link'
            ]
        ),

        # 4) Manipulation node (scan pose)
        Node(
            package='rx200_moveit_control',
            executable='manipulation_node',
            name='manipulation_node',
            output='screen'
        ),
    ])
