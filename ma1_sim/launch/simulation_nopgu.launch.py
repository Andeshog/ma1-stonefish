from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.substitution import Substitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the directories of the involved packages
    ma1_stonefish_sim_dir = get_package_share_directory('ma1_sim')
    stonefish_ros2_dir = get_package_share_directory('stonefish_ros2')

    simulation_data_default = PathJoinSubstitution([ma1_stonefish_sim_dir, 'data'])

    simulation_data_arg = DeclareLaunchArgument(
        'simulation_data',
        default_value=simulation_data_default,
        description='Path to the simulation data folder'
    )
    
    scenario_desc = PathJoinSubstitution([
        ma1_stonefish_sim_dir, 
        'scenarios', 
        'demo.scn'
    ])

    include_stonefish_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stonefish_ros2_dir, '/launch/stonefish_simulator_nogpu.launch.py']),
        launch_arguments={
            'simulation_data': LaunchConfiguration('simulation_data'),
            'scenario_desc': scenario_desc,
        }.items()
    )

    return LaunchDescription([
        simulation_data_arg,
        include_stonefish_launch
    ])