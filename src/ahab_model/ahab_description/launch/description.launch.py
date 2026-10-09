from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

launch_args = [
    DeclareLaunchArgument(
        name='vehicle_namespace',
        default_value='ahab00',
        description='namespace for ros topics and tf frames of this vehicle'
    ),
    DeclareLaunchArgument(
        name='sim',
        default_value='true',
        description='include Gazebo plugins in the description [true | false]'
    ),
    DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='use /clock from Gazebo'
    ),
]


def generate_launch_description():
    vehicle_namespace = LaunchConfiguration('vehicle_namespace')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('ahab_description'), 'urdf', 'ahab.urdf.xacro']
    )
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' namespace:=', vehicle_namespace,
            ' sim:=', LaunchConfiguration('sim'),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=vehicle_namespace,
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([*launch_args, robot_state_publisher])
