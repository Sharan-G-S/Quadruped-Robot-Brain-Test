"""
robot_dog_launch.py — ROS2 Launch File
========================================
Launches all QuadBot-AI nodes together.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ── Arguments ───────────────────────────────────────────
    imu_type_arg = DeclareLaunchArgument(
        "imu_type", default_value="simulation",
        description="IMU sensor type: simulation | mpu6050 | bno055"
    )
    dist_type_arg = DeclareLaunchArgument(
        "distance_type", default_value="simulation",
        description="Distance sensor: simulation | ultrasonic | lidar"
    )
    cam_device_arg = DeclareLaunchArgument(
        "camera_device", default_value="0",
        description="Camera device ID"
    )

    # ── Nodes ───────────────────────────────────────────────

    sensor_publisher = Node(
        package="robot_dog",
        executable="sensor_publisher",
        name="sensor_publisher",
        parameters=[{
            "imu_type": LaunchConfiguration("imu_type"),
            "distance_type": LaunchConfiguration("distance_type"),
            "camera_device": LaunchConfiguration("camera_device"),
            "publish_rate": 50.0,
        }],
        output="screen",
    )

    motor_controller = Node(
        package="robot_dog",
        executable="motor_controller",
        name="motor_controller",
        parameters=[{
            "control_rate": 50.0,
        }],
        output="screen",
    )

    ekf_node = Node(
        package="robot_dog",
        executable="ekf_node",
        name="ekf_node",
        parameters=[{
            "process_noise_pos": 0.01,
            "process_noise_vel": 0.1,
            "process_noise_ori": 0.01,
            "measurement_noise_accel": 0.5,
            "measurement_noise_gyro": 0.05,
            "publish_rate": 50.0,
        }],
        output="screen",
    )

    brain_node = Node(
        package="robot_dog",
        executable="brain_node",
        name="brain_node",
        parameters=[{
            "vision_interval": 2.0,
        }],
        output="screen",
    )

    return LaunchDescription([
        imu_type_arg,
        dist_type_arg,
        cam_device_arg,
        sensor_publisher,
        motor_controller,
        ekf_node,
        brain_node,
    ])
