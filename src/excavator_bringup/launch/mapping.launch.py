"""One-time mapping run. Drive the pit once, save the map, never run this again.

    ros2 launch excavator_bringup mapping.launch.py
    # drive the machine over the whole working area, then:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/pit_map

Afterwards use navigation.launch.py, which localises against the saved map with
AMCL instead of building a new one. Mapping once and localising thereafter
avoids accumulating SLAM drift during every shift.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = get_package_share_directory("excavator_bringup")
    default_ekf = os.path.join(bringup, "config", "ekf.yaml")
    use_ekf = LaunchConfiguration("use_ekf")
    ekf_file = LaunchConfiguration("ekf_file")

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
        parameters=[{"publish_tf": NotSubstitution(use_ekf)}],
    )

    # Map quality is bounded by odometry quality: every yaw error during the
    # mapping run is baked permanently into the map you then localise against.
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=IfCondition(use_ekf),
        parameters=[ekf_file],
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

    return LaunchDescription([
        DeclareLaunchArgument("use_ekf", default_value="true"),
        DeclareLaunchArgument("ekf_file", default_value=default_ekf),
        description, sensors, odometry, ekf, slam,
    ])
