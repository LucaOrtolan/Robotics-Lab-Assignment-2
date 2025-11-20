from launch import LaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    dc = DeclareLaunchArgument(
        'default_gr_state', default_value='True',
        description='what should the start_state of the gripper be'
    )

    moveit_action_client_node = Node(
        package='rx200_moveit_control',
        exec_name='rx200_moveit_action_client',
        parameters=[{'start_state_gripper': LaunchConfiguration('default_gr_state')}],
    )

    ld.add_action(moveit_action_client_node)

    return ld