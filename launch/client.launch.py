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
    calibrate = LaunchConfiguration('calibrate')
    frame_rate = LaunchConfiguration('frame_rate')
    return_type = LaunchConfiguration('return_type')
    lidar_config_file = LaunchConfiguration('lidar_config_file')

    lidar_config_file_path = PathJoinSubstitution(
                    [FindPackagePrefix('quanergy_client_ros'), 'settings', 'client.xml'])

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

    calibrate_arg = DeclareLaunchArgument(
            name='calibrate',
            default_value='False',
            description='whether to calculate the parameters from sensor data before applying'
    )

    frame_rate_arg = DeclareLaunchArgument(
            name='frame_rate',
            default_value='15.0',
            description='frame rate of the sensor; used when calibrate == true only'
    )

    return_type_arg = DeclareLaunchArgument(
            name='return_type',
            default_value="all",
            description="Return selection. For 3 returns, 'all' creates an unorganized point cloud."
                        "For single return, explicitly setting a value produces an error if the selection "
                        "doesn't match the packet. "
                        "options: 0, 1, 2, all, or all_separate_topics "
    )

    lidar_config_arg = DeclareLaunchArgument(
            name='lidar_config_file',
            default_value=lidar_config_file_path,
            description='File containing LIDAR settings.'
    )

    client_node = Node(
        package='quanergy_client_ros',
        namespace=ns,
        executable='client_node',
        name='client_node',
        output='screen',
        arguments=[
                "--host", host,
                "--settings", lidar_config_file,
                "--topic", topic,
                "--frame", frame,
                "--calibrate", calibrate,
                "--frameRate", frame_rate,
                "--return", return_type,
        ]
    )

    return LaunchDescription([
        host_arg,
        ns_arg,
        topic_arg,
        frame_arg,
        calibrate_arg,
        frame_rate_arg,
        return_type_arg,
        lidar_config_arg,
        client_node
    ])