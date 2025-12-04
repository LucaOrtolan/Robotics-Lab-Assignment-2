from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    xsarm_moveit_path = get_package_share_directory('interbotix_xsarm_moveit')

    # ==========================================
    # 0. USER ARGUMENTS
    # ==========================================
    cube_order_arg = DeclareLaunchArgument(
        'cube_order',
        default_value='r,y,b',
        description='Comma-separated order of cubes, e.g. r,y,b'
    )

    place_x_arg = DeclareLaunchArgument(
        'place_x',
        default_value='0.2',
        description='Place point X coordinate (meters)'
    )

    place_y_arg = DeclareLaunchArgument(
        'place_y',
        default_value='0.0',
        description='Place point Y coordinate (meters)'
    )

    cube_order = LaunchConfiguration('cube_order')
    place_x = LaunchConfiguration('place_x')
    place_y = LaunchConfiguration('place_y')

    # ==========================================
    # 1. LAUNCH MOVEIT (uses default URDF)
    # ==========================================
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xsarm_moveit_path, 'launch', 'xsarm_moveit.launch.py')
        ),
        launch_arguments={
            'robot_name': 'rx200',
            'robot_model': 'rx200',
            'hardware_type': 'actual',
            'use_rviz': 'true',
            'rviz_frame': 'rx200/base_link',
            'use_world_frame': 'true',
        }.items()
    )

    # ==========================================
    # 2. STATIC TF: WRIST → CAMERA_MOUNT
    # ==========================================
    camera_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_mount_broadcaster',
        arguments=[
            # Translation (meters): forward 2.5cm, up 3cm
            '0.0', '0.025', '0.03',
            # Rotation (radians): no tilt
            '0.0', '0.0', '0.0',
            # Frame names (parent → child)
            'rx200/wrist_link',        # Must match URDF namespace
            'rx200/camera_mount',      # Our custom frame
        ],
        output='screen'
    )

    # ==========================================
    # 3. STATIC TF: CAMERA_MOUNT → CAMERA_LINK
    # ==========================================
    camera_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_broadcaster',
        arguments=[
            # Translation (meters): 4.5cm up from mount
            '0.0', '0.0', '0.045',
            # Rotation (radians): no tilt
            '0.0', '0.0', '0.0',
            # Frame names
            'rx200/camera_mount',      # Parent (namespaced)
            'camera_link',             # Child (RealSense standard, NO namespace)
        ],
        output='screen'
    )

    camera_camera_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_camera_link_broadcaster',
        arguments=[
            # No offset, no rotation (identity transform)
            '0.0', '0.0', '0.0',   # x, y, z translation
            '0.0', '0.0', '0.0',   # roll, pitch, yaw rotation
            # Parent → Child
            'camera_link',          # Your robot's camera frame
            'camera_camera_link',   # RealSense driver's base frame
        ],
        output='screen'
    )

    # ==========================================
    # 4. REALSENSE CAMERA NODE (starts immediately)
    # ==========================================
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'enable_pointcloud': True,
            'align_depth.enable': True,
            'base_frame_id': 'camera_link',  # Must match TF we published
            'depth_module.profile': '1280x720x30',
            'rgb_camera.profile': '1280x720x30',
        }],
        output='screen'
    )

    # ==========================================
    # 6. PLANNING SCENE UPDATER (delayed 5 seconds)
    # ==========================================

    planning_scene_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rx200_moveit_control',
                executable='add_camera_to_scene',
                name='camera_scene_updater',
                output='screen'
            )
        ]
    )

    # ==========================================
    # 5. MANIPULATION NODE (delayed 5 seconds)
    # ==========================================
    manipulation_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rx200_moveit_control',
                executable='manipulation_node',
                name='manipulation_node',
                output='screen',
                arguments=[cube_order, place_x, place_y]
            )
        ]
    )

    return LaunchDescription([
        cube_order_arg,
        place_x_arg,
        place_y_arg,
        moveit_launch,
        camera_mount_tf,
        camera_link_tf,
        realsense_node,
        camera_camera_link_tf,
        #vision_node,
        planning_scene_node,
        manipulation_node,
    ])
