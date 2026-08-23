import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    package_name = 'picka_description'
    package_share = get_package_share_directory(package_name)
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    xacro_path = os.path.join(package_share, 'urdf', 'picka.xacro')
    world_path = os.path.join(package_share, 'worlds', 'empty.world.sdf')
    bridge_config = os.path.join(
        package_share, 'config', 'ros_gz_bridge_gazebo.yaml'
    )
    rviz_config = os.path.join(package_share, 'config', 'gazebo.rviz')

    robot_description = xacro.process_file(xacro_path).toxml()

    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    entity_name = LaunchConfiguration('entity_name')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = LaunchConfiguration('z')
    spawn_yaw = LaunchConfiguration('yaw')

    # package://picka_description/... resolves from the parent share directory.
    share_parent = os.path.dirname(package_share)
    ignition_resource_path = os.pathsep.join(
        value for value in [
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
            share_parent,
        ] if value
    )
    gz_resource_path = os.pathsep.join(
        value for value in [
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            share_parent,
        ] if value
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    gazebo_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        condition=IfCondition(gui),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        condition=UnlessCondition(gui),
        launch_arguments={
            'gz_args': ['-r -s -v 4 ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_picka',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', entity_name,
            '-allow_renaming', 'false',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
            '--service-call-timeout', '10',
            '--switch-timeout', '10',
        ],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='arm_controller_spawner',
        output='screen',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
            '--service-call-timeout', '10',
            '--switch-timeout', '10',
        ],
    )

    start_state_broadcaster_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    start_arm_controller_after_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Gazebo GUI as well as the server.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz with the simulation.',
        ),
        DeclareLaunchArgument(
            'entity_name',
            default_value='picka',
            description='Unique Gazebo entity name.',
        ),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        SetEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH', ignition_resource_path
        ),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        robot_state_publisher,
        gazebo_with_gui,
        gazebo_headless,
        clock_bridge,
        spawn_robot,
        start_state_broadcaster_after_spawn,
        start_arm_controller_after_state_broadcaster,
        rviz,
    ])
