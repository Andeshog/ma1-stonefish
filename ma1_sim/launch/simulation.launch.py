from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the directories of the involved packages
    ma1_stonefish_sim_dir = get_package_share_directory('ma1_sim')
    stonefish_ros2_dir = get_package_share_directory('stonefish_ros2')

    # Set default paths for simulation_data and scenario_desc
    simulation_data_default = PathJoinSubstitution([ma1_stonefish_sim_dir, 'data'])
    scenario_desc_default = PathJoinSubstitution([ma1_stonefish_sim_dir, 'scenarios'])

    # Declare arguments
    simulation_data_arg = DeclareLaunchArgument(
        'simulation_data',
        default_value=simulation_data_default,
        description='Path to the simulation data folder'
    )

    scenario_desc_arg = DeclareLaunchArgument(
        'scenario_desc',
        default_value=PathJoinSubstitution([scenario_desc_default, 'demo.scn']),
        description='Path to the scenario file'
    )

    window_res_x_arg = DeclareLaunchArgument(
        'window_res_x',
        default_value='1920',
        description='Window resolution width'
    )

    window_res_y_arg = DeclareLaunchArgument(
        'window_res_y',
        default_value='1080',
        description='Window resolution height'
    )

    quality_arg = DeclareLaunchArgument(
        'rendering_quality',
        default_value='high',
    )

    # Logic to prepend the scenarios directory if only a filename is provided
    scenario_desc_resolved = PathJoinSubstitution([
        ma1_stonefish_sim_dir, 'scenarios', LaunchConfiguration('scenario_desc')
    ])

    # Include the original launch file from stonefish_ros2 package
    include_stonefish_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stonefish_ros2_dir, '/launch/stonefish_simulator.launch.py']),
        launch_arguments={
            'simulation_data': LaunchConfiguration('simulation_data'),
            'scenario_desc': scenario_desc_resolved,
            'window_res_x': LaunchConfiguration('window_res_x'),
            'window_res_y': LaunchConfiguration('window_res_y'),
            'rendering_quality': LaunchConfiguration('rendering_quality')
        }.items()
    )

    return LaunchDescription([
        simulation_data_arg,
        scenario_desc_arg,
        window_res_x_arg,
        window_res_y_arg,
        quality_arg,
        include_stonefish_launch
    ])
