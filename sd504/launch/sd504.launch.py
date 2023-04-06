import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():

    # RealSense
    realsense_config_file_path = os.path.join(
        get_package_share_directory('sd504'),
        'config', 'realsense.yaml'
    )  

    realsense_node = ComposableNode(
        namespace="camera",
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[realsense_config_file_path],
    )

    realsense_splitter_node = ComposableNode(
        namespace="camera",
        name='realsense_splitter_node',
        package='realsense_splitter',
        plugin='nvblox::RealsenseSplitterNode',
        parameters=[{
                    'input_qos': 'SENSOR_DATA',
                    'output_qos': 'SENSOR_DATA'
        }],
        remappings=[('input/infra_1', '/camera/infra1/image_rect_raw'),
                    ('input/infra_1_metadata', '/camera/infra1/metadata'),
                    ('input/infra_2', '/camera/infra2/image_rect_raw'),
                    ('input/infra_2_metadata', '/camera/infra2/metadata'),
                    ('input/depth', '/camera/depth/image_rect_raw'),
                    ('input/depth_metadata', '/camera/depth/metadata'),
                    ('input/pointcloud', '/camera/depth/color/points'),
                    ('input/pointcloud_metadata', '/camera/depth/metadata'),
        ]
    )

    realsense_container = ComposableNodeContainer(
        name='realsense_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            realsense_node,
            realsense_splitter_node
        ],
        output='screen'
    )

    # VSLAM
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'enable_imu': True,
                    'input_imu_frame': 'camera_link',
                    'enable_rectified_pose': True,
                    'denoise_input_images': True,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/vslam',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'enable_localization_n_mapping': True,
                    'publish_odom_to_base_tf': True,
                    'publish_map_to_odom_tf': True,
                    'image_qos': 'SENSOR_DATA'
        }],
        remappings=[('stereo_camera/left/image', '/camera/realsense_splitter_node/output/infra_1'),
                    ('stereo_camera/left/camera_info', '/camera/infra1/camera_info'),
                    ('stereo_camera/right/image', '/camera/realsense_splitter_node/output/infra_2'),
                    ('stereo_camera/right/camera_info', '/camera/infra2/camera_info'),
                    ('visual_slam/imu', '/camera/imu')]
    )

    vslam_container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )

    base_link_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '1', '0', '0.15', '0', '0.1088669', '0', '0.9940563',
            'base_link', 'camera_link']
    )

    # Nvblox
    nvblox_config = DeclareLaunchArgument(
        'nvblox_config', default_value=os.path.join(
            get_package_share_directory(
                'sd504'), 'config', 'nvblox.yaml'
        )
    )

    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        parameters=[LaunchConfiguration('nvblox_config')],
        remappings=[
            ("depth/camera_info", "/camera/depth/camera_info"),
            ("depth/image", "/camera/realsense_splitter_node/output/depth"),
            ("color/camera_info", "/camera/color/camera_info"),
            ("color/image", "/camera/color/image_raw")
        ]
    )

    nvblox_container = ComposableNodeContainer(
        name='nvblox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            nvblox_node
        ],
        output='screen'
    )

    # RVIZ
    rviz_config_path = os.path.join(get_package_share_directory(
        'sd504'), 'config', 'gui.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen')
    
    goal_point = Node(
        package='sd504_nav_planner',
        executable='goal_publisher',
        output='screen')

    nav_planner = Node(
        package='sd504_nav_planner',
        executable='planner',
        output='screen'
    )

    motor_controller = Node(
        package='sd504_motor_controller',
        executable='motor_controller_node',
        output='screen'
    )

    return LaunchDescription([
        nvblox_config,
        realsense_container,
        vslam_container,
        nvblox_container,
        base_link_tf_node,
        rviz,
        goal_point,
        nav_planner,
        motor_controller
    ])
