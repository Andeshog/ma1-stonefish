from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ma1_thrust_allocation_node = Node(
        package='ma1_thrust_allocation',
        executable='ma1_thrust_allocation_node.py',
        name='ma1_thrust_allocation_node',
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    return LaunchDescription([
        ma1_thrust_allocation_node,
        joy_node
    ])