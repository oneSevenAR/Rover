import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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

    # 2. Gazebo Sim
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
        output='screen'
    )

    # Force absolute paths for terrain assets
    workspace_dir = os.path.abspath(os.getcwd())
    terrain_visual_path = os.path.join(workspace_dir, "src", "rover_description", "rviz", "dev_2_terrain_for_rover_edited.obj")
    terrain_collision_path = os.path.join(workspace_dir, "src", "rover_description", "rviz", "dev_2_terrain_for_rover_edited.stl")
    terrain_texture_path = os.path.join(workspace_dir, "src", "rover_description", "rviz", "rocky_trail_02_diff_4k.jpg")
    
    terrain_xml = f"""<?xml version="1.0"?>
    <sdf version="1.8">
    <model name="custom_terrain">
        <static>true</static>
        <link name="link">
        <visual name="visual">
            <geometry>
                <mesh><uri>file://{terrain_visual_path}</uri></mesh>
            </geometry>
            <material>
                <ambient>0.35 0.35 0.35 1</ambient>
                <diffuse>1 1 1 1</diffuse>
                <pbr>
                    <metal>
                        <albedo_map>file://{terrain_texture_path}</albedo_map>
                        <roughness>0.8</roughness>
                        <metalness>0.0</metalness>
                    </metal>
                </pbr>
            </material>
        </visual>
        <collision name="collision">
            <geometry>
                <mesh><uri>file://{terrain_collision_path}</uri></mesh>
            </geometry>
        </collision>
        </link>
        <light name="terrain_sun" type="directional">
            <cast_shadows>true</cast_shadows>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <direction>-0.5 0.3 -0.8</direction>
        </light>
    </model>
    </sdf>"""

    # 3. Spawn Custom Terrain
    node_spawn_terrain = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_terrain',
        arguments=[
            '-string', terrain_xml,
            '-name', 'custom_terrain',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # 4. Spawn Camera Rig (Rover)
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_stereo_rig',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'stereo_rig',
            '-z', '5.0'
        ],
        output='screen'
    )

    # 5. Master Unified Bridge
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

    # 6. Camera Info Fixer
    node_camera_info_fixer = Node(
        package='rover_description',
        executable='camera_info_fixer.py',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 7. Stereo Processing & Terrain Filtration Container
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
                    'uniqueness_ratio': 5.0,
                    'texture_threshold': 10,
                    'min_disparity': 2,
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
            ComposableNode(
                package='rtabmap_util',
                plugin='rtabmap_util::ObstaclesDetection',
                name='obstacles_detection',
                namespace='stereo',
                parameters=[{
                    'use_sim_time': True,
                    'frame_id': 'chassis',
                    'map_frame_id': 'chassis',        # Tilts the 15cm cutoff to match the rover's angle
                    'ground_normal_angle': 3.14,      # CRITICAL: Bypasses RANSAC normal calculations completely
                    'max_ground_height': 0.15,        # Dirt < 15cm is painted green
                    'max_obstacles_height': 1.0,      # Objects > 15cm are painted red
                    'min_cluster_size': 20,           # Ignore single floating noise pixels
                    'wait_for_transform': 0.2
                }],
                remappings=[
                    ('cloud', '/stereo/points2'),
                    ('obstacles', '/terrain/obstacles'),
                    ('ground', '/terrain/ground')
                ]
            ),
        ],
        output='screen'
    )

    # 8. RViz2 Node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 9. Full Nav2 Navigation Stack 
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

    # 10. RTAB-Map
    node_rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        arguments=['-d'],
        parameters=[{
            'Grid/MaxGroundHeight': '0.15',      
            'Grid/MaxObstacleHeight': '1.0',     
            'Grid/NormalsSegmentation': 'false',
            'Grid/RangeMax': '3.5', 
            'use_sim_time': True,
            'subscribe_stereo': True,
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'frame_id': 'chassis',
            'approx_sync': True,
            'wait_for_transform': 0.2,
            
            'Vis/EstimationType': '1', 
            'Vis/MinInliers': '5',               # FIXED: Lowered from 10 to tolerate bad dirt matches
            'Grid/Sensor': '1',
            'Stereo/MaxDisparity': '512',        # FIXED: Doubled from 256 to allow calculation of very close pixels
            'Grid/NoiseFilteringRadius': '0.1',      
            'Grid/NoiseFilteringMinNeighbors': '5',
            
            'Kp/MaxFeatures': '1500',            # FIXED: Increased to grab more visual anchors
            'Vis/MaxFeatures': '1500',           # FIXED: Matched to Kp to ensure tracking pipeline doesn't choke
            'Vis/CorGuessWinSize': '40',     
            'Stereo/OpticalFlow': 'false',    
            'Optimizer/Strategy': '1',
            
            'Odom/Strategy': '1',                # ADDED: Better frame-to-frame odometry for continuous off-roading
            'Odom/GuessMotion': 'true',          # ADDED: Forces the system to guess its location if the camera goes blind for a frame
            
            'Reg/Force3DoF': 'true',
            'Odom/Holonomic': 'false'
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

    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frequency': 30.0,
            'two_d_mode': False, 
            'publish_tf': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'chassis',
            'world_frame': 'odom',
            
            'odom0': '/odom_raw',
            'odom0_config': [True,  True,  False,   
                             False, False, True,    
                             True,  True,  False,   
                             False, False, True,    
                             False, False, False],

            'imu0_relative': False,
                             
            'imu0': '/imu/data',
            'imu0_config': [False, False, False, 
                            True,  True,  False,  
                            False, False, False, 
                            True,  True,  False,  
                            False, False, False]
        }],
        remappings=[
            ('odometry/filtered', '/odom') 
        ]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gz_sim,
        node_spawn_terrain,      
        node_spawn_entity,          
        node_bridge,
        node_camera_info_fixer,
        stereo_container,
        node_rviz,
        joint_state_pub_node,
        node_rtabmap,
        node_ekf,
        node_nav2_stack
    ])