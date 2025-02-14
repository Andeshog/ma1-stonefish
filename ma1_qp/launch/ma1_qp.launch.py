from os import path
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    qp_node = Node(
        package='ma1_qp',
        executable='ma1_qp',
        name='ma1_qp',
        output='screen')
    
    return LaunchDescription([qp_node])
