from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch", "rs_launch.py"
            ])
        ),
        launch_arguments={
            'align_depth.enable': 'true',
        }.items()
    )

    rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': 0.04,
        }],
        remappings=[
            ('rgb/image',       '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image',     '/camera/camera/aligned_depth_to_color/image_raw'),
        ]
    )

    # Visual odometry — replaces wheel encoders
    rgbd_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'camera_color_optical_frame',
            'approx_sync': True,
        }],
        remappings=[
            ('rgbd_image', '/rgbd_image'),
        ]
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'camera_color_optical_frame',
            'subscribe_rgbd': True,
            'subscribe_odom': True,        # ✅ now expects /odom from rgbd_odometry
            'Reg/Force3DoF': 'true',
            'Grid/3D': 'false',
        }],
        remappings=[
            ('rgbd_image', '/rgbd_image'),
            ('odom',       '/odom'),
        ],
        arguments=['--delete_db_on_start']
    )

    return LaunchDescription([
        realsense_launch,
        rgbd_sync_node,
        rgbd_odometry_node,
        rtabmap_node,
    ])


# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.substitutions import FindPackageShare
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.actions import Node

# def generate_launch_description():

#     aruco_detection_node = Node(
#         package='robot',
#         executable='aruco_detection_node',
#         name='aruco_detection_node',
#         output='screen'
#     )

#     # -----------------------
#     # RTAB-Map SLAM
#     # -----------------------
#     rtabmap_node = Node(
#     package='rtabmap_ros',
#     executable='rtabmap',
#     output='screen',
#     parameters=[{
#         'frame_id': 'camera_color_optical_frame',
#         'subscribe_rgbd': True,
#         'rgbd_sync': True,
#         'wait_imu_to_init': False,
#         'Reg/Force3DoF': True,
#     }],
#     remappings=[
#         ('rgb/image', '/camera/camera/color/image_raw'),
#         ('rgb/camera_info', '/camera/camera/color/camera_info'),
#         ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
#     ],
#     arguments=['--delete_db_on_start']
#     )

#     rviz2 = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         output='screen',
#     )

#     first_node = Node(
#         package='robot',
#         executable='test_node',
#         name='first_node',
#         output='screen'
#     )

#     realsense_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([
#                 FindPackageShare("realsense2_camera"),
#                 "launch",
#                 "rs_launch.py"
#             ])
#         ))

#     return LaunchDescription([
#         # rtabmap_node,
#         realsense_launch,
#         # aruco_detection_node,
#         #  first_node,
#         # rviz2  
#     ])
