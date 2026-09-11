"""One-time mapping run. Drive the pit once, save the map, never run this again.

    ros2 launch excavator_bringup mapping.launch.py
    # drive the machine over the whole working area, then:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/pit_map

Afterwards use navigation.launch.py, which localises against the saved map with
AMCL instead of building a new one. Mapping once and localising thereafter
avoids accumulating SLAM drift during every shift.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("excavator_description"), "launch", "description.launch.py",
        ]))
    )

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("excavator_bringup"), "launch", "sensors.launch.py",
        ]))
    )

    odometry = Node(
        package="excavator_control",
        executable="track_odometry",
        name="track_odometry",
        output="screen",
        parameters=[{"publish_tf": True}],
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "odom_frame": "odom",
            "map_frame": "map",
            "base_frame": "base_footprint",
            "scan_topic": "/scan",
            "mode": "mapping",
            "resolution": 0.05,
            "max_laser_range": 8.0,
            "minimum_travel_distance": 0.2,
            "minimum_travel_heading": 0.2,
        }],
    )

    return LaunchDescription([description, sensors, odometry, slam])
