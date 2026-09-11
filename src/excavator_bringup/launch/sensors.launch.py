"""RealSense + depth-to-scan conversion, shared by mapping and navigation.

The camera feeds two consumers with different needs:

  * depth image -> depthimage_to_laserscan -> /scan
        AMCL and slam_toolbox are 2D and require a LaserScan. The conversion
        throws height away, which is fine for localisation.

  * point cloud -> nav2 voxel layers
        Height is exactly what distinguishes a limestone pile from a wall, so
        obstacle marking uses the cloud, never the scan. Enabling the cloud
        costs CPU; the decimation filter below keeps it affordable on an Orin.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scan_height = LaunchConfiguration("scan_height")

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"), "launch", "rs_launch.py",
            ])
        ),
        launch_arguments={
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "decimation_filter.enable": "true",   # keeps the cloud affordable
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2",              # linear_interpolation
        }.items(),
    )

    # Madgwick turns the raw gyro/accel into an orientation estimate. Feed this
    # to robot_localization alongside track odometry: wheels carry translation,
    # the gyro carries heading. Skid-steer yaw from tracks alone is unreliable
    # by construction.
    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        output="screen",
        parameters=[{
            "use_mag": False,
            "publish_tf": False,
            "world_frame": "enu",
        }],
        remappings=[
            ("imu/data_raw", "/camera/camera/imu"),
            ("imu/data", "/imu/data"),
        ],
    )

    depth_to_scan = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan",
        output="screen",
        parameters=[{
            "output_frame": "camera_link_optical",
            "range_min": 0.30,      # D435i is blind closer than this
            "range_max": 8.0,
            "scan_height": scan_height,
            "scan_time": 0.033,
        }],
        remappings=[
            ("depth", "/camera/camera/aligned_depth_to_color/image_raw"),
            ("depth_camera_info", "/camera/camera/aligned_depth_to_color/camera_info"),
            ("scan", "/scan"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "scan_height", default_value="20",
            description="Rows of the depth image collapsed into the scan. Too "
                        "tall and a downward-tilted camera folds the ground "
                        "into the scan, making open floor look like a wall.",
        ),
        realsense,
        imu_filter,
        depth_to_scan,
    ])
