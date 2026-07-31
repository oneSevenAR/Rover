import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():

    nav2_config_path = os.path.join(
        get_package_share_directory('rover_description'),
        'config',
        'nav2_params.yaml'
    )

    pkg_share = get_package_share_directory('rover_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'stereo_camera.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # 1. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 2. Gazebo Sim (Using the guaranteed working default world)
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
        output='screen'
    )

    # Force the absolute path so Gazebo doesn't lose the texture
    workspace_dir = os.path.abspath(os.getcwd())
    checker_texture = os.path.join(workspace_dir, "src", "checkerboard.png")
    
    material_xml = f"""
    <material>
    <ambient>1 1 1 1</ambient>
    <diffuse>1 1 1 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <pbr>
        <metal>
        <albedo_map>{checker_texture}</albedo_map>
        </metal>
    </pbr>
    </material>
    """

    red_box_xml = f"""<?xml version="1.0"?>
    <sdf version="1.8">
    <model name="red_box">
        <static>true</static>
        <link name="link">
        <visual name="visual">
            <geometry>
            <box>
                <size>1 1 1</size>
            </box>
            </geometry>
            {material_xml}
        </visual>
        <collision name="collision">
            <geometry>
            <box>
                <size>1 1 1</size>
            </box>
            </geometry>
        </collision>
        </link>
    </model>
    </sdf>"""

    blue_box_xml = f"""<?xml version="1.0"?>
    <sdf version="1.8">
    <model name="blue_box">
        <static>true</static>
        <link name="link">
        <visual name="visual">
            <geometry>
            <box>
                <size>1 2 1</size>
            </box>
            </geometry>
            {material_xml}
        </visual>
        <collision name="collision">
            <geometry>
            <box>
                <size>1 2 1</size>
            </box>
            </geometry>
        </collision>
        </link>
    </model>
    </sdf>"""

    green_cylinder_xml = f"""<?xml version="1.0"?>
    <sdf version="1.8">
    <model name="green_cylinder">
        <static>true</static>
        <link name="link">
        <visual name="visual">
            <geometry>
            <cylinder>
                <radius>0.5</radius>
                <length>1.0</length>
            </cylinder>
            </geometry>
            {material_xml}
        </visual>
        <collision name="collision">
            <geometry>
            <cylinder>
                <radius>0.5</radius>
                <length>1.0</length>
            </cylinder>
            </geometry>
        </collision>
        </link>
    </model>
    </sdf>"""

    yellow_sphere_xml = f"""<?xml version="1.0"?>
    <sdf version="1.8">
    <model name="yellow_sphere">
        <static>true</static>
        <link name="link">
        <visual name="visual">
            <geometry>
            <sphere>
                <radius>0.6</radius>
            </sphere>
            </geometry>
            {material_xml}
        </visual>
        <collision name="collision">
            <geometry>
            <sphere>
                <radius>0.6</radius>
            </sphere>
            </geometry>
        </collision>
        </link>
    </model>
    </sdf>"""

    wall_box_xml = f"""<?xml version="1.0"?>
    <sdf version="1.8">
    <model name="wall_box">
        <static>true</static>
        <link name="link">
        <visual name="visual">
            <geometry>
            <box>
                <size>0.5 3 1</size>
            </box>
            </geometry>
            {material_xml}
        </visual>
        <collision name="collision">
            <geometry>
            <box>
                <size>0.5 3 1</size>
            </box>
            </geometry>
        </collision>
        </link>
    </model>
    </sdf>"""

    # Spawn Checkered Cube
    node_spawn_red_box = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_red_box',
        arguments=[
            '-string', red_box_xml,
            '-name', 'red_box',
            '-x', '2.0',
            '-y', '1.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # Spawn Checkered Rectangular Box
    node_spawn_blue_box = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_blue_box',
        arguments=[
            '-string', blue_box_xml,
            '-name', 'blue_box',
            '-x', '3.0',
            '-y', '-1.5',
            '-z', '0.5',
            '-Y', '0.5'
        ],
        output='screen'
    )

    # Spawn Cylinder
    node_spawn_cylinder = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_cylinder',
        arguments=['-string', green_cylinder_xml, '-name', 'green_cylinder', '-x', '4.0', '-y', '1.5', '-z', '0.5'],
        output='screen'
    )

    # Spawn Sphere
    node_spawn_sphere = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_sphere',
        arguments=['-string', yellow_sphere_xml, '-name', 'yellow_sphere', '-x', '5.0', '-y', '-1.0', '-z', '0.6'],
        output='screen'
    )

    # Spawn Wall
    node_spawn_wall = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_wall',
        arguments=['-string', wall_box_xml, '-name', 'wall_box', '-x', '2.0', '-y', '-2.5', '-z', '0.5'],
        output='screen'
    )

    # Spawn Camera Rig
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_stereo_rig',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'stereo_rig',
            '-z', '0.5'
        ],
        output='screen'
    )

    # 4. Master Unified Bridge (Clean and Stable)
    node_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/stereo/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/stereo/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/stereo/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/stereo/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom_raw@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU', 
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        remappings=[
            ('/stereo/right/camera_info', '/stereo/right/camera_info_raw')
        ],
        parameters=[{
            'use_sim_time': True,
            'qos_overrides./stereo/left/camera_info.publisher.durability': 'transient_local',
            'qos_overrides./stereo/right/camera_info.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # 5. Camera Info Fixer
    node_camera_info_fixer = Node(
        package='rover_description',
        executable='camera_info_fixer.py',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 6. Stereo Processing Container
    stereo_container = ComposableNodeContainer(
        name='stereo_container',
        namespace='stereo',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='left_rectify_node',
                namespace='stereo/left',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('image', 'image_raw'),
                    ('image_rect', 'image_rect')
                ]
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='right_rectify_node',
                namespace='stereo/right',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('image', 'image_raw'),
                    ('image_rect', 'image_rect')
                ]
            ),
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::DisparityNode',
                name='disparity_node',
                namespace='stereo',
                parameters=[{
                    'use_sim_time': True,
                    'approximate_sync': True,
                    # --- NEW AGGRESSIVE FILTERING PARAMS ---
                    'uniqueness_ratio': 15.0,  
                    'texture_threshold': 100,  
                    'speckle_size': 1000,      
                    'speckle_range': 31        
                }],
                remappings=[
                    ('left/image_rect', 'left/image_rect'),
                    ('left/camera_info', 'left/camera_info'),
                    ('right/image_rect', 'right/image_rect'),
                    ('right/camera_info', 'right/camera_info'),
                    ('disparity', 'disparity')
                ]
            ),
            # Restored PointCloudNode that was missing in your provided script
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::PointCloudNode',
                name='point_cloud_node',
                namespace='stereo',
                parameters=[{
                    'use_sim_time': True,
                    'approximate_sync': True,
                    'use_color': False,
                    'queue_size': 100
                }],
                remappings=[
                    ('left/image_rect_color', 'left/image_rect')
                ]
            ),
        ],
        output='screen'
    )

    # 7. RViz2 Node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 8. PointCloud to LaserScan Node
    node_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/stereo/points2'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'chassis',
            'transform_tolerance': 0.01,
            'min_height': 0.15,   
            'max_height': 0.25,   
            'range_min': 0.4,     
            'range_max': 3.5,     
            'angle_min': -0.7,  
            'angle_max': 0.7,   
            'use_inf': True,
            'use_sim_time': True
        }],
        output='screen'
    )

    # 9 & 10. Full Nav2 Navigation Stack (Planner, Controller, Recoveries)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    node_nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_config_path,
            'autostart': 'true'
        }.items()
    )

    joint_state_pub_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # 11. RTAB-Map configured for pure passive stereo (No RGB-D)
    node_rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        arguments=['-d'],
        parameters=[{
            'use_sim_time': True,
            'subscribe_stereo': True,
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'frame_id': 'chassis',
            'approx_sync': True,
            'wait_for_transform': 0.2,
            
            # Core tuning parameters for stereo visual odometry
            'Vis/EstimationType': '1', 
            'Vis/MinInliers': '10',     
            'Grid/Sensor': '1',
            'Stereo/MaxDisparity': '256',
            # --- NEW NOISE FILTERING PARAMS ---
            'Grid/NoiseFilteringRadius': '0.1',      
            'Grid/NoiseFilteringMinNeighbors': '5'   
        }],
        remappings=[
            ('left/image_rect', '/stereo/left/image_rect'),
            ('left/camera_info', '/stereo/left/camera_info'),
            ('right/image_rect', '/stereo/right/image_rect'),
            ('right/camera_info', '/stereo/right/camera_info'),
            ('odom', '/odom')  
        ],
        output='screen'
    )

    # 12. Robot Localization (EKF)
    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frequency': 30.0,
            'two_d_mode': True,
            'publish_tf': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'chassis',
            'world_frame': 'odom',
            # Feed in Gazebo's raw wheel odometry
            'odom0': '/odom_raw',
            'odom0_config': [True,  True,  False,   
                             False, False, True,    
                             True,  True,  False,   
                             False, False, True,    
                             False, False, False],
            # Feed in the IMU data
            'imu0': '/imu/data',
            'imu0_config': [False, False, False, 
                            False, False, True,     
                            False, False, False, 
                            False, False, True,     
                            True,  False, False]    
        }],
        remappings=[
            ('odometry/filtered', '/odom') 
        ]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gz_sim,
        node_spawn_entity,       
        node_spawn_red_box,      
        node_spawn_blue_box,
        node_spawn_cylinder,
        node_spawn_sphere,
        node_spawn_wall,     
        node_bridge,
        node_camera_info_fixer,
        stereo_container,
        node_rviz,
        node_laserscan,
        joint_state_pub_node,
        node_rtabmap,
        node_ekf,
        node_nav2_stack
    ])