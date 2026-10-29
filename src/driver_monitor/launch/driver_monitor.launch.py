"""Launch the complete intelligent driver monitoring pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
        [FindPackageShare("driver_monitor"), "config", "driver_monitor.yaml"]
    )
    enable_can = LaunchConfiguration("enable_can")
    enable_web = LaunchConfiguration("enable_web")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_can", default_value="true", description="Start SocketCAN bridge"
            ),
            DeclareLaunchArgument(
                "enable_web", default_value="true", description="Start MJPEG web server"
            ),
            Node(
                package="driver_monitor",
                executable="camera_node",
                name="camera_node",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="driver_monitor",
                executable="driver_monitor_node",
                name="driver_monitor_node",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="driver_monitor",
                executable="warning_manager_node",
                name="warning_manager_node",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="driver_monitor",
                executable="socketcan_bridge_node",
                name="socketcan_bridge_node",
                parameters=[config],
                output="screen",
                condition=IfCondition(enable_can),
            ),
            Node(
                package="driver_monitor",
                executable="web_stream_node",
                name="web_stream_node",
                parameters=[config],
                output="screen",
                condition=IfCondition(enable_web),
            ),
        ]
    )
