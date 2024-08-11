from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    param_file = LaunchConfiguration('param_file', default='slam_gmapping/param/slam_gmapping_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value='slam_gmapping/param/slam_gmapping_params.yaml',
            description='Path to the YAML file with parameters relative to the package'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        
        launch_ros.actions.Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            output='screen',
            parameters=[param_file, {'use_sim_time': use_sim_time}],
        ),
    ])
