from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path
import os

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    robot_path = get_package_share_directory('robot')
    sdf_file = os.path.join(robot_path, 'models', 'robot', 'robot.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f"-r {os.path.join(robot_path,'worlds/track-1.sdf')}",
                              'use_sim_time': 'true'}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
           '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/track/model/vehicle_blue/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[(
            '/world/track/model/vehicle_blue/joint_state','/joint_states'
        )],
        output='screen'
    )

    keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'
    )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(robot_path, 'config', 'rviz_config.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    apriltags = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        remappings=[
            ('image_rect', '/camera'),
            ('camera_info', '/camera_info'),
        ],
        parameters=[robot_path+"/config/apriltags.yaml"]
    )    

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )
    
    world_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["1", "0", "0", "0", "0", "0", "world", "map"],
            parameters=[{'use_sim_time':True}]
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(robot_path, 'config/ekf.yaml'), {'use_sim_time': True}]
    )

    get_pose = Node(
        package='robot',
        executable='get_pose',
        name='pose_server',
        output='screen'
    )

    executive = Node(
        package='robot',
        executable='executive',
        output='screen'
    )

    nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'params_file': os.path.join(robot_path, 'config', 'nav2_params.yaml'),
                'use_sim_time': 'true'  # Adding use_sim_time argument
            }.items(),
        )

    slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_sync_launch.py')), #Uncomment this to create a new map
            # os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'localization_launch.py')), #Uncomment this to pull from pre-defined map
            launch_arguments={'use_sim_time': 'true', 'slam_params_file': os.path.join(robot_path,'config/mapper_params_localization.yaml')}.items(),
        )

    return LaunchDescription([
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=robot_path),
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=str(Path(os.path.join(robot_path)).parent.resolve())),
        world_tf,
        bridge,
        keyboard,
        robot_state_publisher,
        # apriltags,
        robot_localization_node,
        rviz,
        gz_sim,
        nav2,
        slam,
        get_pose,
        executive
    ])
