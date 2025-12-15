import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

package_name = "gazebo_garden_simulation_example"


def generate_static_tf_publisher_node(parentFrame, childFrame):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_tf_publisher_{parentFrame}",
        arguments=["0", "0", "0", "0", "0", "0", parentFrame, childFrame],
        parameters=[{"use_sim_time": True}],
    )


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_path = get_package_share_directory(package_name)

    rviz_config_file = LaunchConfiguration("rviz_config")
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(pkg_path, "rviz", "basic.rviz"),
        description="Path to rviz config file",
    )

    model = os.path.join(pkg_path, "models", "tugbot_rviz", "model.sdf")

    with open(model, "r") as infp:
        robot_desc = infp.read().replace(
            "<uri>", f"<uri>package://{package_name}/models/tugbot_rviz/"
        )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py"),
        ),
        launch_arguments={
            "gz_args": f"-r \"/ros2_ws/src/Gazebo-Garden-Simulation/models/tugbot_warehouse/tugbot_warehouse.sdf\""
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="tugbot_bridge",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "/world/world_demo/model/tugbot/link/camera_front/sensor/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/world/world_demo/model/tugbot/link/camera_front/sensor/color/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/world/world_demo/model/tugbot/link/camera_front/sensor/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/world/world_demo/model/tugbot/link/camera_front/sensor/depth/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/world/world_demo/model/tugbot/link/camera_front/sensor/depth/depth_image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/model/tugbot/battery/linear_battery/state@sensor_msgs/msg/BatteryState[gz.msgs.BatteryState",
            "/world/world_demo/model/tugbot/link/imu_link/sensor/imu/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/model/tugbot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/tugbot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/world/world_demo/model/tugbot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/model/tugbot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/model/tugbot/pose@geometry_msgs/msg/TransformStamped[gz.msgs.Pose",
        ],
        remappings=[
            (
                "/world/world_demo/model/tugbot/link/camera_front/sensor/color/camera_info",
                "camera_front/camera_info",
            ),
            (
                "/world/world_demo/model/tugbot/link/camera_front/sensor/color/image",
                "camera_front/image",
            ),
            (
                "/world/world_demo/model/tugbot/link/camera_front/sensor/depth/camera_info",
                "camera_front/depth/camera_info",
            ),
            (
                "/world/world_demo/model/tugbot/link/camera_front/sensor/depth/depth_image",
                "camera_front/depth/image",
            ),
            (
                "/world/world_demo/model/tugbot/link/camera_front/sensor/depth/depth_image/points",
                "scan_3D",
            ),
            ("/model/tugbot/battery/linear_battery/state", "battery_state"),
            ("/model/tugbot/odometry", "odom"),
            ("/model/tugbot/cmd_vel", "cmd_vel"),
            (
                "/world/world_demo/model/tugbot/link/imu_link/sensor/imu/imu",
                "imu/data",
            ),
            (
                "/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan",
                "scan",
            ),
            ("/world/world_demo/model/tugbot/joint_state", "joint_states"),
        ],
        output="screen",
    )

    delayedNodes = TimerAction(
        period=2.5,
        actions=[rviz, bridge],
    )

    robotState = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
    )

    # RF2O Node
    rf2o = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('rf2o_laser_odometry'),'launch','rf2o_laser_odometry.launch.py')])
    )

    # EKF Node
    ekf = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ancle_pkg'),'launch','ekf_launch.py')])
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            gazebo,
            robotState,
            delayedNodes,
            generate_static_tf_publisher_node(
                "scan_omni", "tugbot/scan_omni/scan_omni"
            ),
            generate_static_tf_publisher_node(
                "camera_front", "tugbot/camera_front/color"
            ),
            generate_static_tf_publisher_node(
                "camera_front", "tugbot/camera_front/depth"
            ),
            rf2o,
            #ekf
        ]
    )