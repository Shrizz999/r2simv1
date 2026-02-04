import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_r2krishna = get_package_share_directory('r2krishna')
    pkg_arena = get_package_share_directory('arena_viz')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    robot_urdf = os.path.join(pkg_r2krishna, 'urdf', 'R2krishna.urdf')
    arena_world = os.path.join(pkg_arena, 'worlds', 'arena.world')

    with open(robot_urdf, 'r') as infp:
        robot_desc = infp.read()

    # Paths for Gazebo models
    workspace_paths = os.path.join(pkg_arena, '..') + ':' + os.path.join(pkg_r2krishna, '..')
    final_resource_path = f'{workspace_paths}:/usr/share/gz/models:/usr/share/gazebo-11/models'

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {arena_world}'}.items(),
    )

    # 2. Spawn Robot (UPDATED: x=5, y=2)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'r2krishna', '-x', '5', '-y', '2', '-z', '0.5'],
        output='screen'
    )

    # 3. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 4. Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/model/r2krishna/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/model/r2krishna/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/model/r2krishna/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'
        ],
        remappings=[
            ('/model/r2krishna/tf', '/tf'),
            ('/model/r2krishna/joint_state', '/joint_states')
        ],
        output='screen'
    )

    # 5. TF Fix for Lidar
    tf_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'laser_frame', '--child-frame-id', 'r2krishna/base_link/lidar'],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=final_resource_path),
        gazebo,
        spawn_robot,
        rsp,
        bridge,
        tf_fix
    ])