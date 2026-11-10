import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod_isaacsim')
    pkg_parent = os.path.dirname(pkg_share)

    # Ensure Gazebo Sim finds package:// meshes
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = f'{gz_resource_path}:{pkg_parent}:{pkg_share}' if gz_resource_path else f'{pkg_parent}:{pkg_share}'

    ign_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = f'{ign_resource_path}:{pkg_parent}:{pkg_share}' if ign_resource_path else f'{pkg_parent}:{pkg_share}'

    urdf_file = os.path.join(pkg_share, 'urdf', 'hexapod_gazebo.urdf')
    rviz_file = os.path.join(pkg_share, 'rviz', 'hexapod.rviz')
    world_file = os.path.join(pkg_share, 'worlds', 'hexapod_world.sdf')
    controllers_file = os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')

    # Read URDF and dynamically resolve $(find hexapod_isaacsim) paths for Gazebo plugins
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    robot_desc = robot_desc.replace('$(find hexapod_isaacsim)', pkg_share)

    # Launch arguments
    rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='false',
        description='Set to true for Gazebo GUI, or false for headless simulation (recommended under WSL2)'
    )
    spawn_z_arg = DeclareLaunchArgument(
        name='z',
        default_value='0.15',
        description='Spawn Z position'
    )

    from launch.substitutions import PythonExpression
    gz_args = PythonExpression([
        f"'-r -v 3 {world_file}' if '",
        LaunchConfiguration('gui'),
        f"' == 'true' else '-r -v 3 -s {world_file}'"
    ])

    # 1. Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': gz_args}.items()
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True}
        ]
    )

    # 3. Spawn Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'hexapod',
            '-string', robot_desc,
            '-x', '0.0',
            '-y', '0.0',
            '-z', LaunchConfiguration('z'),
            '-allow_renaming', 'true'
        ]
    )

    # 4. ROS-Gazebo Bridges (Clock + IMU)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ]
    )

    # 5. Controllers Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )

    # Delay start of controllers until robot is spawned in Gazebo
    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                joint_trajectory_controller_spawner
            ]
        )
    )

    # 6. RViz2 (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        gui_arg,
        spawn_z_arg,
        gz_sim,
        gz_bridge,
        robot_state_publisher,
        spawn_robot,
        delay_controllers,
        rviz_node
    ])
