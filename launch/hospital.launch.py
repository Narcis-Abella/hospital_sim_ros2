import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'hospital_sim'
    pkg_share = get_package_share_directory(pkg_name)
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'tracer2.xacro')
    world_file = os.path.join(pkg_share, 'worlds', '2026_hospital_lite.sdf')
    
    install_dir = os.path.join(pkg_share, '..')

    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    sim_time = {'use_sim_time': True}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}, sim_time]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[sim_time]
    )

    node_tf_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen',
        parameters=[sim_time]
    )

    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'tracer_robot',
            '-topic', 'robot_description',
            '-x', '-13.0',
            '-y', '5.0',
            '-z', '0.0',
            '-Y', '-1.570796'
        ],
        parameters=[sim_time]
    )

    node_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
            # '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model' 
        ],
        output='screen'
    )

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[sim_time]
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=install_dir),
        launch_gazebo,
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_tf_footprint,
        node_spawn_entity,
        node_bridge,
        node_rviz
    ])
