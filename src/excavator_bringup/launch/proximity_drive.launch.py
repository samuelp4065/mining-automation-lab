"""Background nodes for manual drive with the proximity interlock.

Brings up the RealSense depth stream, the proximity monitor, and the motor
driver -- the three nodes that have no interactive terminal requirement and
can run under ros2 launch's log capture.

teleop_keyboard is deliberately NOT included here. It reads raw keystrokes
via termios, which needs a real attached terminal; ros2 launch redirects
stdin for its child processes, so termios.tcgetattr() fails with
"Inappropriate ioctl for device" the instant the node starts. Run teleop in
its own terminal instead:

    ros2 launch excavator_bringup proximity_drive.launch.py
    # in a second terminal:
    ros2 run excavator_control teleop_keyboard

The servos genuinely turn -- there is no max_rpm:=0 safety net baked in here.
Keep the machine clear to move.

    ros2 launch excavator_bringup proximity_drive.launch.py max_rpm:=15.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    max_rpm = LaunchConfiguration("max_rpm")
    stop_below = LaunchConfiguration("stop_below")
    release_above = LaunchConfiguration("release_above")
    patch = LaunchConfiguration("patch")

    # Depth stream only -- no IMU, no laserscan, no point cloud. Those are
    # nav2/SLAM inputs that proximity-only drive does not need.
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"), "launch", "rs_launch.py",
            ])
        ),
        launch_arguments={
            "align_depth.enable": "true",
        }.items(),
    )

    proximity_monitor = Node(
        package="excavator_perception",
        executable="proximity_monitor",
        name="proximity_monitor",
        output="screen",
        parameters=[{
            "stop_below": stop_below,
            "release_above": release_above,
            "patch": patch,
        }],
    )

    motor_node = Node(
        package="excavator_control",
        executable="motor_node",
        name="motor_node",
        output="screen",
        parameters=[{
            "max_rpm": max_rpm,
            "require_proximity_ok": True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "max_rpm", default_value="20.0",
            description="Hard ceiling on commanded wheel RPM."),
        DeclareLaunchArgument(
            "stop_below", default_value="1.00",
            description="Forward motion blocks below this distance, metres."),
        DeclareLaunchArgument(
            "release_above", default_value="1.20",
            description="Forward motion resumes above this distance, metres."),
        DeclareLaunchArgument(
            "patch", default_value="9",
            description="Depth sample patch size, pixels square."),
        realsense,
        proximity_monitor,
        motor_node,
    ])
