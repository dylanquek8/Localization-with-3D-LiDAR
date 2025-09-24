from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu103',
            executable='imu103',
            name='imu103',
            output='screen',
            parameters=[{
                # Change these easily in launch file
                'input_topic': '/rslidar_imu_data',
                'calib_quaternion': [-0.701591, 0.712543, -0.00591461, -0.00427578],
                'calib_translation': [0.00425, 0.00418, -0.00446]
            }]
        )
    ])
