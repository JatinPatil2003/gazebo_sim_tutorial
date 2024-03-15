#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = parse(open(xacro_file_path))
    process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    autonav_des_path = get_package_share_directory("autonav_description")
    autonav__gazebo_path = get_package_share_directory("autonav_gazebo")
    world_file = LaunchConfiguration("world_file", default = join(autonav__gazebo_path, "worlds", "empty_world.sdf"))


    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time},
                    {'robot_description': Command( \
                    ['xacro ', join(autonav_des_path, 'urdf/autonav.xacro'),
                    ])}],
        remappings=[
            ('/joint_states', 'autonav/joint_states'),
        ]
    )
 
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression(["'", world_file, " -r'"])

        }.items()
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "autonav",
            "-allow_renaming", "true",
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            # "/kinect_camera@sensor_msgs/msg/Image[ignition.msgs.Image",
            # "/stereo_camera/left/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            # "stereo_camera/right/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            # "kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            # "stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            # "stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            # "/kinect_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            # "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/world/default/model/autonav/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"
        ],
        remappings=[
            ('/world/default/model/autonav/joint_state', 'autonav/joint_states'),
            ('/odom', 'autonav/odom'),
            ('/scan', 'autonav/scan'),
            # ('/kinect_camera', 'autonav/kinect_camera'),
            # ('/stereo_camera/left/image_raw', 'autonav/stereo_camera/left/image_raw'),
            # ('/stereo_camera/right/image_raw', 'autonav/stereo_camera/right/image_raw'),
            # ('/imu', 'autonav/imu'),
            ('/cmd_vel', 'autonav/cmd_vel'),
            # ('kinect_camera/camera_info', 'autonav/kinect_camera/camera_info'),
            # ('stereo_camera/left/camera_info', 'autonav/stereo_camera/left/camera_info'),
            # ('stereo_camera/right/camera_info', 'autonav/stereo_camera/right/camera_info'),
            # ('/kinect_camera/points', 'autonav/kinect_camera/points'),
        ]
    )

    # transform_publisher = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments = ["--x", "0.0",
    #                 "--y", "0.0",
    #                 "--z", "0.0",
    #                 "--yaw", "0.0",
    #                 "--pitch", "0.0",
    #                 "--roll", "0.0",
    #                 "--frame-id", "kinect_camera",
    #                 "--child-frame-id", "autonav/base_link/kinect_camera"]
    # )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument("world_file", default_value=world_file),
        robot_state_publisher,
        gz_sim, 
        gz_spawn_entity, 
        gz_ros2_bridge,
        # transform_publisher
    ])