"""
fake_hardware.launch.py — full system test without a physical robot.

Starts:
  - robot_state_publisher     (Gen3 7-DoF URDF, fake hardware variant)
  - rviz2                     (kortex_description view_robot.rviz config)
  - joint_state_publisher_gui (drag sliders to pose the robot interactively)
  - recorder                  (FAKE_HARDWARE=1, no gRPC connection)

Workflow:
  1. After launch, drag the joint sliders to pose the robot.
  2. Start recording:
       ros2 service call /motion_recorder/start_recording std_srvs/srv/Trigger {}
  3. Move the sliders to trace a trajectory.
  4. Stop recording:
       ros2 service call /motion_recorder/stop_recording std_srvs/srv/Trigger {}
  5. Replay the saved JSON (animates in RViz — stop the GUI first to avoid
     fighting the slider publisher):
       ros2 run rec_rep2 fake_replayer <path_to_json> [speed]

Or use the GUI (ros2 run rec_rep2 gui) with the "Fake hardware" checkbox for a
fully integrated experience, including automatic replay routing.
"""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('kortex_description'),
            'robots',
            'gen3.xacro',
        ]),
        ' robot_ip:=0.0.0.0 name:=arm arm:=gen3 dof:=7 use_fake_hardware:=true',
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('kortex_description'),
        'rviz',
        'view_robot.rviz',
    ])

    return LaunchDescription([
        SetEnvironmentVariable('FAKE_HARDWARE', '1'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', rviz_config],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rec_rep2',
            executable='recorder',
            name='motion_recorder',
            output='screen',
        ),
    ])
