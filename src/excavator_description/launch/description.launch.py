"""Publish the excavator's TF tree from the URDF.

Every other piece of the navigation stack depends on this running: nav2's
height-banded obstacle layer measures against base_footprint, and AMCL needs
base_link -> camera to interpret the laser scan.

    ros2 launch excavator_description description.launch.py
    ros2 launch excavator_description description.launch.py joint_mode:=simulate
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("excavator_description")
    xacro_file = os.path.join(pkg, "urdf", "excavator.urdf.xacro")

    joint_mode = LaunchConfiguration("joint_mode")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ParameterValue(..., value_type=str) is required: without it launch tries
    # to parse the xacro output as YAML and dies on the XML.
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            "joint_mode", default_value="hardware",
            description="'hardware' reads the swingdrive over the LSS bus; "
                        "'simulate' publishes fixed angles with no serial port."),
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
        ),

        # swing_joint is revolute, so robot_state_publisher publishes no
        # camera transform at all until something supplies joint states.
        # excavator_control owns the servos, so it owns this.
        Node(
            package="excavator_control",
            executable="joint_state_bridge",
            name="joint_state_bridge",
            output="screen",
            parameters=[{
                "mode": joint_mode,
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
