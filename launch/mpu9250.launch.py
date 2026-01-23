from launch import LaunchDescription
from launch_ros.actions import Node

#
#  colcon build; source install/setup.bash; ros2 launch mpu9250 mpu9250.launch.py
#

def generate_launch_description():

    return LaunchDescription([

        # IMU node - https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/MPU9250.md
        Node(
            package="mpu9250",
            executable="mpu9250",
            name="mpu9250",
            output='screen',
            respawn=True,
            respawn_delay=4,
            emulate_tty=True,
            parameters=[{
                "print": True,       # default False
                #"raw_only": True,    # default False ("fusing" mode). When True - only publish raw IMU data - /imu/data_raw and /imu/mag
                "frequency": 30,
                "temp_pub_rate_hz": 1.0,  # temperature publish rate in Hz
                "frame_id": "imu_link",
                "i2c_address": 0x68,  # also, 0x0C shows up for built-in AK8963 magnetometer
                "i2c_port": 1,        # a.k.a "bus". For Linux on Raspberry Pi Bus=1
                "acceleration_scale": [1.0, 1.0, 1.0],  # small adjustment of scale factors for each axis, should be around 1.0
                "acceleration_bias": [0.0, 0.0, 0.0],
                "gyro_bias": [0.0, 0.0, 0.0],
                # use tests/calibrate_mag.py to get mag calibration values
                "magnetometer_scale": [1.0, 1.0, 1.0],  # should be around 1.0
                "magnetometer_bias": [1.672994523195427e-05, 1.777942953037992e-05, 3.2817091139903744e-05],
                "magnetometer_transform": [
                    1.0160951390293467, 0.008597352199034276, -0.008498487872556243,
                    0.008597352199034368, 1.0040890425158557, 0.014842492476619326,
                    -0.008498487872556252, 0.014842492476619368, 0.980515572473782],
                "madgwick_beta": 0.01,       # 0.01 for faster settling after rotation
                "madgwick_use_mag": True
            }]
        ),

        # for experiments: RViz starts with "map" as Global Fixed Frame, provide a TF to see axes etc.
        Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments=[
                '--x', '0.0',     # X translation in meters
                '--y', '0.0',     # Y translation in meters
                '--z', '0.1',     # Z translation in meters
                '--roll', '0.0',  # Roll in radians
                '--pitch', '0.0', # Pitch in radians
                '--yaw', '0.0',   # Yaw in radians (e.g., 90 degrees)
                '--frame-id', 'map', # Parent frame ID
                '--child-frame-id', 'imu_link' # Child frame ID
            ]
        )
    ])
