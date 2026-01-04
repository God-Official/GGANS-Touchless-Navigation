import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    
    pkg_ur_description = 'turtlebot_description'
    urdf_path = 'urdf/turtlebot.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_ur_description), urdf_path)
    robo_des_raw = xacro.process_file(xacro_file).toxml()

    world_path = PathJoinSubstitution([
        FindPackageShare("turtlebot_description"),  # Replace with your package name
        "worlds",
        "model_world.sdf"
    ])

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("turtlebot_description"), "config", "display.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # === Nodes ===
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robo_des_raw},
            {'use_sim_time': True}
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robo_des_raw,
            "-name", "turtlebot3",
            "-allow_renaming", "true",
            "-x", "-2.0",
            "-y", "-0.5",
            "-z", "0.01",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0"
        ]
    )

    gz_launch_description_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"
        ]),
        launch_arguments={"gz_args": ["-r -v 4 ", world_path]}.items(),
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/odom/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            "/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        ],
        remappings=[
            ("/odom/tf", "tf"),
        ],
        output="screen",
    )

    depth_image_converter = Node(
        package="depth_image_proc",
        executable="convert_metric_node",
        name="depth_image_converter",
        output="screen",
        remappings=[
            ("image_raw", "/camera/depth_image"),
            ("camera_info", "/camera/camera_info"),
            ("image", "/camera/depth_image/image"),
            
        ]
    )

    map_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'])

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        gz_spawn_entity,
        gz_launch_description_with_gui,
        gz_sim_bridge,
        map_static_tf,
        depth_image_converter,
    ])