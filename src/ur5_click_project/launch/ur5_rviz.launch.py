from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ),
        launch_arguments={
            'ur_type': 'ur5',
            'robot_ip': '192.168.56.101',
            'launch_rviz': 'true'
        }.items()
    )

    control = Node(
        package='ur5_click_project',
        executable='ur5_click_node',
        output='screen'
    )

    return LaunchDescription([driver, control])
