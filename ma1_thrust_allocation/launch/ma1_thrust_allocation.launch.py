from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    arg_mode = DeclareLaunchArgument(
        'mode',
        default_value='normal',
        description='Mode of operation: normal, hybrid',
        choices=['normal', 'hybrid']
    )
    ma1_thrust_allocation_node = Node(
        package='ma1_thrust_allocation',
        executable='ma1_thrust_allocation_node.py',
        name='ma1_thrust_allocation_node',
        parameters=[
            {'mode': LaunchConfiguration('mode')}
        ],
        output='screen'
    )

    return LaunchDescription([
        arg_mode,
        ma1_thrust_allocation_node
    ])