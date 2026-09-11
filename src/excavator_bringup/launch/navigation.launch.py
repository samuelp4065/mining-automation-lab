"""Runtime navigation: localise against the saved map and avoid obstacles.

No SLAM and no RTAB-Map. The map is built once by mapping.launch.py; from then
on AMCL localises against it, which is both lighter and drift-free compared to
running SLAM continuously.

    ros2 launch excavator_bringup navigation.launch.py map:=/home/ae/maps/pit_map.yaml

Obstacle avoidance is height-aware: see the voxel_layer comments in
config/nav2_params.yaml for the bands that separate a diggable limestone pile
from a genuine obstacle.

Odometry comes from an EKF fusing track encoders (translation) with the D435i
gyro (heading) -- see config/ekf.yaml for why that split matters on a
skid-steer machine. Pass use_ekf:=false to fall back to raw track odometry,
which will drift in yaw on every turn.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, NotSubstitution, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = get_package_share_directory("excavator_bringup")
    default_params = os.path.join(bringup, "config", "nav2_params.yaml")
    default_ekf = os.path.join(bringup, "config", "ekf.yaml")

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    ekf_file = LaunchConfiguration("ekf_file")
    use_ekf = LaunchConfiguration("use_ekf")

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

    # Exactly one node may own odom -> base_footprint. When the EKF runs it
    # takes that job, so the raw odometry must stop broadcasting or TF will
    # flip between two disagreeing transforms for the same frame pair.
    odometry = Node(
        package="excavator_control",
        executable="track_odometry",
        name="track_odometry",
        output="screen",
        parameters=[{"publish_tf": NotSubstitution(use_ekf)}],
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=IfCondition(use_ekf),
        parameters=[ekf_file],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py",
        ])),
        launch_arguments={
            "map": map_yaml,
            "params_file": params_file,
            "use_sim_time": "false",
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map", default_value=os.path.expanduser("~/maps/pit_map.yaml"),
            description="Saved map from the one-time mapping.launch.py run."),
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="nav2 parameters, including the height-banded costmaps."),
        DeclareLaunchArgument(
            "ekf_file", default_value=default_ekf,
            description="robot_localization EKF configuration."),
        DeclareLaunchArgument(
            "use_ekf", default_value="true",
            description="Fuse track odometry with the IMU. False uses raw track "
                        "odometry, which drifts in yaw on every skid-steer turn."),
        description,
        sensors,
        odometry,
        ekf,
        nav2,
    ])
