import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

launch_args = [
    DeclareLaunchArgument(
        name='vehicle_namespace',
        default_value='ahab00',
        description='namespace for ros topics, tf frames and the gz model name'
    ),
    DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(
            get_package_share_directory('ahab_worlds'), 'worlds', 'empty.sdf'
        ),
        description='path to the Gazebo world sdf'
    ),
    DeclareLaunchArgument(
        name='headless',
        default_value='false',
        description='run the Gazebo server only, no GUI window [true | false]'
    ),
    DeclareLaunchArgument(name='x', default_value='0.0', description='spawn x [m]'),
    DeclareLaunchArgument(name='y', default_value='0.0', description='spawn y [m]'),
    DeclareLaunchArgument(name='z', default_value='0.3', description='spawn z [m]'),
    DeclareLaunchArgument(name='yaw', default_value='0.0', description='spawn yaw [rad]'),
]


def setup_launch(context, *args, **kwargs):
    vehicle_namespace = LaunchConfiguration('vehicle_namespace').perform(context)
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1')

    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )),
        launch_arguments={'gz_args': f'-r {"-s " if headless else ""}{world}'}.items(),
    )

    # robot_state_publisher with the xacro description
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ahab_description'), 'launch', 'description.launch.py'
        )),
        launch_arguments={
            'vehicle_namespace': vehicle_namespace,
            'sim': 'true',
            'use_sim_time': 'true',
        }.items(),
    )

    # Spawn the model from the robot_description topic
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', f'/{vehicle_namespace}/robot_description',
            '-name', vehicle_namespace,
            '-x', LaunchConfiguration('x').perform(context),
            '-y', LaunchConfiguration('y').perform(context),
            '-z', LaunchConfiguration('z').perform(context),
            '-Y', LaunchConfiguration('yaw').perform(context),
        ],
    )

    # ROS <-> gz bridge, with the namespace filled into the config template
    bridge_template = os.path.join(
        get_package_share_directory('ahab_description'), 'config', 'vehicle_gz_bridge.yaml'
    )
    with open(bridge_template, 'r') as f:
        bridge_config = f.read().replace('VEHICLE_NAMESPACE', vehicle_namespace)
    bridge_file = os.path.join(
        tempfile.gettempdir(), f'{vehicle_namespace}_gz_bridge.yaml'
    )
    with open(bridge_file, 'w') as f:
        f.write(bridge_config)

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_file, 'use_sim_time': True}],
    )

    return [gz_sim, description, spawn, bridge]


def generate_launch_description():
    return LaunchDescription([
        *launch_args,
        OpaqueFunction(function=setup_launch)
    ])
