from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # --- Declare arguments users can override on the CLI ---
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.10',
        description='IP address of the Kinova Gen3',
    )
    use_fake_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use mock hardware for testing without robot',
    )

    # --- Include the kortex_bringup launch file, launch file inception ---
    kortex_dir = get_package_share_directory('kortex_bringup')
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kortex_dir, 'launch', 'gen3.launch.py')
        ),
        launch_arguments={
            'robot_ip':           LaunchConfiguration('robot_ip'),
            'use_fake_hardware':  LaunchConfiguration('use_fake_hardware'),
            'dof':                '7',
            'arm':                'gen3',
        }.items(),
    )

    # --- Your recorder node ---
    recorder_node = Node(
        package='rec_rep2',
        executable='recorder',
        name='motion_recorder',
        output='screen',
        # Parameters can be passed here or from a YAML file
        parameters=[{
            'save_directory': '/tmp',
        }],
    )

    return LaunchDescription([
        robot_ip_arg,
        use_fake_arg,
        driver_launch,
        recorder_node,
    ])