from launch import LaunchDescription
from launch_ros.actions import Node

#
#  colcon build; source install/setup.bash; ros2 launch mpu9250 mpu9250_raw.launch.py
#

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="mpu9250",
            executable="mpu9250",
            name="mpu9250",
            parameters=[{
                "print": False,
                "raw_only": True,    # only publish raw IMU data - /imu/data_raw and /imu/mag
                "frequency": 30,
                "temp_pub_rate_hz": 1.0,  # temperature publish rate in Hz
                "frame_id": "imu_link",
                "i2c_address": 0x68,  # also, 0x0C shows up for built-in AK8963 magnetometer
                "i2c_port": 1,        # a.k.a "bus". For Linux on Raspberry Pi Bus=1
                "acceleration_scale": [1.0, 1.0, 1.0],  # small adjustment of scale factors for each axis, should be around 1.0
                "acceleration_bias": [0.0, 0.0, 0.0],
                "gyro_bias": [0.0, 0.0, 0.0],
                "magnetometer_scale": [1.0, 1.0, 1.0],  # should be around 1.0
                "magnetometer_bias": [0.0, 0.0, 0.0],
                "magnetometer_transform": [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]
                #"acceleration_scale": [1.0072387165748442, 1.0081436035838134, 0.9932769089604535],
                #"acceleration_bias": [0.17038044467587418, 0.20464685207217453, -0.12461014438322202],
                #"gyro_bias": [0.0069376404996494, -0.0619247665634732, 0.05717760948453845],
                #"magnetometer_bias": [0.4533159894397744, 3.4555818146055564, -5.984038606178013],
                #"magnetometer_transform": [   0.9983016121720226, 0.044890057238382707, 0.007231924972024632,
                #                    0.044890057238382707, 1.2981683205953654, -0.1173361838042438, 
                #                    0.007231924972024633, -0.11733618380424381, 0.7835617468652673]
                }],
        ),

        # Madgwick filter node to compute orientation quaternion from raw IMU data
        # publishes to "imu/data" topic
        # https://github.com/CCNYRoboticsLab/imu_tools
        # sudo apt install ros-${ROS_DISTRO}-imu-tools
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                "stateless": False,
                "use_mag": True,
                "publish_tf": True,
                "reverse_tf": False,
                "fixed_frame": "odom",
                "constant_dt": 0.0,
                "publish_debug_topics": False,
                "world_frame": "enu",
                "gain": 0.1,
                "zeta": 0.0,
                "mag_bias_x": 0.0,
                "mag_bias_y": 0.0,
                "mag_bias_z": 0.0,
                "orientation_stddev": 0.0
            }],
            #remappings=[("imu/mag", "imu/mag"), ("imu/data_raw", "imu/data_raw"), ("imu/data", "imu/data")],
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
                '--child-frame-id', 'odom' # Child frame ID
            ]
        )
    ])
