from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')

    ur_share = FindPackageShare('ur_description')

    # Główny xacro
    xacro_file = PathJoinSubstitution([ur_share, 'urdf', 'ur.urdf.xacro'])

    # Pliki konfiguracyjne wymagane przez ur.urdf.xacro (dla danego modelu)
    kinematics = PathJoinSubstitution([ur_share, 'config', ur_type, 'default_kinematics.yaml'])
    joint_limits = PathJoinSubstitution([ur_share, 'config', ur_type, 'joint_limits.yaml'])
    physical = PathJoinSubstitution([ur_share, 'config', ur_type, 'physical_parameters.yaml'])
    visual = PathJoinSubstitution([ur_share, 'config', ur_type, 'visual_parameters.yaml'])

    # xacro + wymagane argumenty
    robot_description = Command([
        'xacro ', xacro_file,
        ' name:=', TextSubstitution(text='ur5'),
        ' kinematics_params:=', kinematics,
        ' joint_limit_params:=', joint_limits,
        ' physical_params:=', physical,
        ' visual_params:=', visual,
    ])

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='ur5_click_project',
            executable='ur5_click_node',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),
    ])
