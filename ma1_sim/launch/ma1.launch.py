from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ma1_interface_node = Node(
        package='ma1_sim',
        executable='ma1_sim_interface',
        name='ma1_sim_interface',
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    return LaunchDescription([
        ma1_interface_node,
        joy_node
    ])