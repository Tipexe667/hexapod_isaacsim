import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod_isaacsim')

    default_model_path = os.path.join(pkg_share, 'urdf', 'hexapod_gazebo.urdf')
    default_rviz_path = os.path.join(pkg_share, 'rviz', 'hexapod.rviz')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file'
    )
    rviz_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_path,
        description='Absolute path to rviz config file'
    )

    robot_description = LaunchConfiguration('model')

    # Load URDF content
    with open(default_model_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        model_arg,
        rviz_arg,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')]
        )
    ])
