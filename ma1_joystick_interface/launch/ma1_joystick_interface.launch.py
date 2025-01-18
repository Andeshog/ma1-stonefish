from os import path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

config_file = path.join(
    get_package_share_directory('ma1_joystick_interface'),
    'config',
    'config.yaml'
)

def generate_launch_description():
    ma1_joystick_interface_node = Node(
        package='ma1_joystick_interface',
        executable='ma1_joystick_interface_node.py',
        name='ma1_joystick_interface_node',
        parameters=[config_file],
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    return LaunchDescription([
        ma1_joystick_interface_node,
        joy_node
    ])