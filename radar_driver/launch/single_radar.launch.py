from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    name = LaunchConfiguration('name')
    iface = LaunchConfiguration('iface')
    port = LaunchConfiguration('port')
    packet_filter = LaunchConfiguration('filter')
    capture = LaunchConfiguration('capture')
    cpath = LaunchConfiguration('cpath')
    scan_mode = LaunchConfiguration('scanMode')
    unfiltered_topic = LaunchConfiguration('uTopic')

    def launch_nodes(context):
        publisher_args = ['-e', iface, '-p', port, '-c', capture, '-l', cpath]
        filter_value = packet_filter.perform(context)
        if filter_value:
            publisher_args.extend(['-f', filter_value])

        return [
            Node(
                package='radar_driver',
                executable='radar_publisher',
                name='radar_publisher',
                namespace=name,
                arguments=publisher_args,
            ),
            Node(
                package='radar_driver',
                executable='radar_processor',
                name='radar_processor',
                namespace=name,
                arguments=['-s', scan_mode, '-t', unfiltered_topic],
            ),
            Node(
                package='radar_driver',
                executable='radar_visualizer',
                name='radar_visualizer',
                namespace=name,
                arguments=['-t', 'filtered_radar_packet', '-n', name],
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='radar'),
        DeclareLaunchArgument('iface', default_value='enx5c28868515e9'),
        DeclareLaunchArgument('port', default_value='31122'),
        DeclareLaunchArgument('filter', default_value=''),
        DeclareLaunchArgument('capture', default_value='1'),
        DeclareLaunchArgument('cpath', default_value='radarCap5.pcap'),
        DeclareLaunchArgument('scanMode', default_value='0'),
        DeclareLaunchArgument('uTopic', default_value='unfiltered_radar_packet'),
        OpaqueFunction(function=launch_nodes),
    ])
