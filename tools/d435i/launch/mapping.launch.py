"""RTAB-Map RGB-D + IMU mapping for a RealSense D435i.

Run the camera separately first (see README-style commands in the chat), then:
    ros2 launch /ws/launch/mapping.launch.py
Override the database or camera height with launch args, e.g.:
    ros2 launch /ws/launch/mapping.launch.py database_path:=/ws/maps/mine.db cam_z:=0.4
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

# 2D grid is projected from the 3D map: only points between the ground band and
# MaxObstacleHeight (relative to base_link) become obstacles.
RTABMAP_ARGS = " ".join([
    "--delete_db_on_start",
    "--Grid/FromDepth true",
    "--Grid/3D false",
    "--Grid/RayTracing true",
    "--Grid/CellSize 0.05",
    "--Grid/RangeMax 4.0",           # D435i depth error ~7 cm at 4 m, ~15 cm at 6 m
    "--Grid/MinGroundHeight 0.0",
    "--Grid/MaxGroundHeight 0.1",
    "--Grid/MaxObstacleHeight 1.8",
    "--Grid/NormalsSegmentation true",
    # Add "--Reg/Force3DoF true" only for a wheeled robot on flat ground; it
    # distorts the map if the camera is moved handheld (pitch/roll/height change).
    "--Optimizer/GravitySigma 0.3",  # use IMU gravity to keep map level
    "--Vis/MinInliers 12",           # default 20; a run was lost at 11-12/20 inliers
    "--Vis/MaxFeatures 1000",        # more features per frame for rough, textured rock
])


def generate_launch_description():
    args = [
        DeclareLaunchArgument("database_path", default_value="/ws/maps/rtabmap.db"),
        DeclareLaunchArgument("viz", default_value="true"),
        DeclareLaunchArgument("cam_x", default_value="0.0"),
        DeclareLaunchArgument("cam_y", default_value="0.0"),
        DeclareLaunchArgument("cam_z", default_value="0.0"),
        # true: track the left IR (infra1) image instead of RGB. The projector's
        # dot pattern gives it texture on plain walls/low-contrast rock that the
        # color image doesn't have. Depth is already native to infra1 (no
        # color-alignment step), and the IR sensor is global-shutter.
        # Needs enable_infra1/2:=true on the camera (see mapping_full.launch.py).
        DeclareLaunchArgument("use_infra", default_value="false"),
    ]

    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        output="screen",
        parameters=[{"use_mag": False, "publish_tf": False, "world_frame": "enu"}],
        remappings=[("imu/data_raw", "/camera/camera/imu"),
                    ("imu/data", "/imu/data")],
    )

    # base_link -> camera_link. Replace with the real mounting once on the robot.
    base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[LaunchConfiguration("cam_x"), LaunchConfiguration("cam_y"),
                   LaunchConfiguration("cam_z"), "0", "0", "0",
                   "base_link", "camera_link"],
    )

    common_args = {
        "imu_topic": "/imu/data",
        "frame_id": "base_link",
        "approx_sync": "false",
        "wait_imu_to_init": "true",
        "database_path": LaunchConfiguration("database_path"),
        "args": RTABMAP_ARGS,
        "rtabmap_viz": LaunchConfiguration("viz"),
        "rviz": "false",
    }

    rtabmap_rgb = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("rtabmap_launch"), "launch", "rtabmap.launch.py"])),
        launch_arguments={
            "rgb_topic": "/camera/camera/color/image_raw",
            "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            **common_args,
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("use_infra")),
    )

    # IR mode: left infra image + its native (unaligned) depth, both already in
    # the infra1 frame, so no color-to-depth alignment error is introduced.
    rtabmap_infra = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("rtabmap_launch"), "launch", "rtabmap.launch.py"])),
        launch_arguments={
            "rgb_topic": "/camera/camera/infra1/image_rect_raw",
            "depth_topic": "/camera/camera/depth/image_rect_raw",
            "camera_info_topic": "/camera/camera/infra1/camera_info",
            **common_args,
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_infra")),
    )

    return LaunchDescription(args + [imu_filter, base_to_camera, rtabmap_rgb, rtabmap_infra])
