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
                "verbose": True,       # default False
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
                #"magnetometer_scale": [1.0, 1.0, 1.0],  # should be 1.0 or omitted if "magnetometer_transform" is present
                "magnetometer_bias": [1.879474231677064e-05, 1.2669697764271128e-05, -3.0470527626723397e-05],
                "magnetometer_transform": [
                    1.0000000521217702, 1.2535309229370016e-08, -1.6163252600070903e-09,
                    1.2535309234403542e-08, 0.9999999234564466, 2.4705503080522403e-08,
                    -1.6163252598524059e-09, 2.470550306997046e-08, 1.000000024421789],
                "madgwick_beta": 0.1,       # beta is often in the 0.01–0.2 ballpark, weight of correction from accelerometer/magnetometer vs gyroscope
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
