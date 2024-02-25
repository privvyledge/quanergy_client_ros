from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():
    host = LaunchConfiguration('host')
    ns = LaunchConfiguration('ns')
    topic = LaunchConfiguration('topic')
    frame = LaunchConfiguration('frame')
    return_type = LaunchConfiguration('return_type')

    host_arg = DeclareLaunchArgument(
        name='host',
        description='Host name or IP of the sensor.'
    )

    ns_arg = DeclareLaunchArgument(
        name='ns',
        default_value='quanergy',
        description='Namespace for the node.'
    )

    topic_arg = DeclareLaunchArgument(
        name='topic',
        default_value='points',
        description='ROS topic for publishing the point cloud.'
    )

    frame_arg = DeclareLaunchArgument(
        name='frame',
        default_value=ns,
        description='Frame name inserted in the point cloud.'
    )

    return_type_arg = DeclareLaunchArgument(
            name='return_type',
            default_value="all",
            description="Return selection. For 3 returns, 'all' creates an unorganized point cloud."
                        "For single return, explicitly setting a value produces an error if the selection "
                        "doesn't match the packet. "
                        "options: 0, 1, 2, all, or all_separate_topics "
    )

    client_node = Node(
        package='quanergy_client_ros',
        namespace=ns,
        executable='client_node',
        name='client_node',
        output='screen',
        arguments=[
                "--host", host,
                "--settings", PathJoinSubstitution(
                    [FindPackagePrefix('quanergy_client_ros'), 'settings', 'client.xml']),
                "--topic", topic,
                "--frame", frame,
                "--return", return_type
        ]
    )

    return LaunchDescription([
        host_arg,
        ns_arg,
        topic_arg,
        frame_arg,
        return_type_arg,
        client_node
    ])