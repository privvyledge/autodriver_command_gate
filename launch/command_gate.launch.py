from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    impl = LaunchConfiguration('impl').perform(context)
    config = LaunchConfiguration('config').perform(context)
    use_composition = LaunchConfiguration('use_composition').perform(context).lower()

    if impl == 'python':
        return [
            Node(
                package='command_gate',
                executable='command_gate_node_py',
                name='command_gate',
                parameters=[config],
                output='screen',
            )
        ]

    if use_composition == 'true':
        return [
            ComposableNodeContainer(
                name='command_gate_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='command_gate',
                        plugin='command_gate::CommandGateNode',
                        name='command_gate',
                        parameters=[config],
                    )
                ],
                output='screen',
            )
        ]

    return [
        Node(
            package='command_gate',
            executable='command_gate_node',
            name='command_gate',
            parameters=[config],
            output='screen',
        )
    ]


def generate_launch_description():
    pkg_share = FindPackageShare('command_gate')
    default_config = PathJoinSubstitution([pkg_share, 'config', 'command_gate.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'impl',
            default_value='cpp',
            description='Implementation to launch: cpp or python'),
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to parameter YAML file'),
        DeclareLaunchArgument(
            'use_composition',
            default_value='false',
            description='Load C++ node as ComposableNode inside a container'),
        OpaqueFunction(function=launch_setup),
    ])
