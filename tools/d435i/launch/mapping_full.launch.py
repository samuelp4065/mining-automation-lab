"""Camera + IMU filter + RTAB-Map in one launch (used by map.sh; headless by default)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])),
        launch_arguments={
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2",
            "align_depth.enable": "true",
            "enable_infra1": "true",
            "enable_infra2": "true",
            "rgb_camera.color_profile": "640x480x30",
            # depth and IR come from one stereo sensor and must share a resolution,
            # otherwise depth frames stall ("Frames didn't arrive within 5 seconds")
            "depth_module.depth_profile": "640x480x30",
            "depth_module.infra_profile": "640x480x30",
        }.items(),
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource("/ws/launch/mapping.launch.py"),
        launch_arguments={
            "database_path": LaunchConfiguration("database_path"),
            "viz": LaunchConfiguration("viz"),
            "use_infra": LaunchConfiguration("use_infra"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("database_path", default_value="/ws/maps/rtabmap.db"),
        DeclareLaunchArgument("viz", default_value="false"),
        DeclareLaunchArgument("use_infra", default_value="false"),
        camera,
        # give the camera a few seconds to start publishing before RTAB-Map subscribes
        TimerAction(period=5.0, actions=[mapping]),
    ])
