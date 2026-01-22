import smbus2 as smbus
import math
import numpy as np

from .imusensor.MPU9250 import MPU9250
from .imusensor.filters import kalman 
from .imusensor.filters import madgwick

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField, Temperature

from math import radians
import tf_transformations


class MPU9250Node(Node):
    def __init__(self):
        super().__init__("mpu9250_node")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('print', False),
                ('raw_only', False),
                ('frequency', 30),
                ('temp_pub_rate_hz', 1.0),
                ('frame_id', 'imu_link'),
                ('i2c_address', 0x68),
                ('i2c_port', 1),
                ('acceleration_scale', [1.0, 1.0, 1.0]),
                ('acceleration_bias', [0.0, 0.0, 0.0]),
                ('gyro_bias', [0.0, 0.0, 0.0]),
                ('magnetometer_scale', [1.0, 1.0, 1.0]),
                ('magnetometer_bias', [0.0, 0.0, 0.0]),
                ('magnetometer_transform', [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]),
            ]
        )

        self.logger = self.get_logger()

        address = self.get_parameter('i2c_address').value
        bus = smbus.SMBus(self.get_parameter('i2c_port').value)

        self.logger.info(f"   i2c_addr: 0x{address:X} on bus {bus.fd}")

        self.imu = MPU9250(bus, address)

        self.print = self.get_parameter('print').get_parameter_value().bool_value
        self.logger.info(f"   print: {self.print}")

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.logger.info(f"   frame_id: {self.frame_id}")

        self.raw_only = self.get_parameter('raw_only').get_parameter_value().bool_value
        self.logger.info(f"   raw_only: {self.raw_only}")

        self.pub_rate_hz = self.get_parameter("frequency").get_parameter_value().integer_value
        self.logger.info(f"   pub_rate_hz: {self.pub_rate_hz} Hz")
        
        temp_pub_rate_hz = float(self.get_parameter("temp_pub_rate_hz").value)
        self.logger.info(f"   temp_pub_rate_hz: {temp_pub_rate_hz} Hz")

        self.imu.Accels = np.asarray(self.get_parameter('acceleration_scale').get_parameter_value().double_array_value)
        self.imu.AccelBias = np.asarray(self.get_parameter('acceleration_bias').get_parameter_value().double_array_value)
        self.imu.GyroBias = np.asarray(self.get_parameter('gyro_bias').get_parameter_value().double_array_value)
        self.imu.Mags = np.asarray(self.get_parameter('magnetometer_scale').get_parameter_value().double_array_value)
        self.imu.MagBias = np.asarray(self.get_parameter('magnetometer_bias').get_parameter_value().double_array_value)
        self.imu.Magtransform = np.reshape(np.asarray(self.get_parameter('magnetometer_transform').get_parameter_value().double_array_value),(3,3))

        pub_rate_hz = self.get_parameter('frequency').get_parameter_value().integer_value
        self.timer_publish_imu_values_ = self.create_timer(1.0/pub_rate_hz, self.publish_imu_values)

        self.publisher_imu_values_ = self.create_publisher(Imu, "/imu/data_raw" if self.raw_only else "/imu/data", 10)  # raw or fused IMU values
        
        self.publisher_mag_values_ = self.create_publisher(MagneticField, "/imu/mag", 10)  # raw magnetometer values

        self.publisher_temperature = self.create_publisher(Temperature, "/imu/temp", 10)

        # Temperature averaging (accumulate at IMU rate, publish averaged at ~temp_pub_rate_hz)
        self._temp_sum_c = 0.0
        self._temp_count = 0
        # Divider: publish temperature every N IMU ticks
        self._temp_div = max(1, int(round(pub_rate_hz / max(0.1, temp_pub_rate_hz))))

        self.imu.begin()  # takes time for quick gyro calibration, gets factory mag calibration

        # expect MagScale: [1.79882812e-07 1.79882812e-07 1.73437500e-07]
        self.logger.info(f"Magnetometer Sensitivity Scales: {self.imu.MagScale}  LSB/Tesla")
        self.logger.info(f"Gyro Bias: {self.imu.GyroBias}  rad/s")

        if not self.raw_only:
            # Initialize sensor fusion algorithm:
            self.sensorfusion = kalman.Kalman()
            #self.sensorfusion = madgwick.Madgwick(0.5)

            # Initial read to set roll/pitch/yaw:
            self.imu.readSensor()    
            self.imu.computeOrientation()
            self.sensorfusion.roll = self.imu.roll
            self.sensorfusion.pitch = self.imu.pitch
            self.sensorfusion.yaw = self.imu.yaw

        self.lastTime = self.get_clock().now()

    def wrap_pi(self, angle):
        # Wraps the given angle(s) to +/- pi.
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def rotate_mag(self, mag_vals):
        """
        Rotate magnetometer reading to align with accel+gyro frame.
        Inputs:
            mag_values, a numpy.ndarray — a 1D array of 3 float64 values, magnetometer readings
        Returns:
            numpy.ndarray — a 1D array of 3 float64 values : rotated magnetometer readings
        """
        #   Accel/Gyro: X forward, Y left, Z up
        #   Mag:        X left, Y forward, Z down
        mx, my, mz = mag_vals
        mxr =  my
        myr =  mx
        mzr = -mz
        return np.array([mxr, myr, mzr])

    def publish_imu_values(self):
        self.imu.readSensor()
        now = self.get_clock().now()
        deltaTime = (now - self.lastTime).nanoseconds * 1e-9  # seconds (float)
        deltaTime = max(1e-4, min(deltaTime, 0.2))  # clamp to reasonable range
        self.lastTime = now
        frame_id = self.frame_id

        magVals_rotated = self.rotate_mag(self.imu.MagVals)

        if not self.raw_only:
            #self.imu.computeOrientation()
            #yaw = self.imu.yaw
            #pitch = self.imu.pitch
            #roll = self.imu.roll
            #computeAndUpdateRollPitchYaw

            self.sensorfusion.computeAndUpdateRollPitchYaw(
                self.imu.AccelVals[0], self.imu.AccelVals[1], self.imu.AccelVals[2],
                self.imu.GyroVals[0], self.imu.GyroVals[1], self.imu.GyroVals[2],
                magVals_rotated[0], magVals_rotated[1], magVals_rotated[2], deltaTime)

            # RPY should be in the ENU (East-North-Up) reference frame, in degrees
            # sensor frame (REP-103 body: x forward, y left, z up), world frame (ENU)
            roll = self.sensorfusion.roll
            pitch = self.sensorfusion.pitch
            yaw = self.sensorfusion.yaw

        msg_imu = Imu()
        msg_imu.header.stamp = now.to_msg()
        msg_imu.header.frame_id = frame_id

        if self.raw_only:
            # Raw measurements, unknown covariance
            msg_imu.linear_acceleration.x = self.imu.AccelVals[0]  # m/s^2
            msg_imu.linear_acceleration.y = self.imu.AccelVals[1]
            msg_imu.linear_acceleration.z = self.imu.AccelVals[2]
            msg_imu.linear_acceleration_covariance[0] = -1.0

            msg_imu.angular_velocity.x = self.imu.GyroVals[0]  # rad/s
            msg_imu.angular_velocity.y = self.imu.GyroVals[1]
            msg_imu.angular_velocity.z = self.imu.GyroVals[2]
            msg_imu.angular_velocity_covariance[0] = -1.0

            # No orientation in raw data
            msg_imu.orientation_covariance[0] = -1.0
        else:
            # Fused measurements with covariance
            msg_imu.linear_acceleration.x = self.imu.AccelVals[0]  # m/s^2
            msg_imu.linear_acceleration.y = self.imu.AccelVals[1]
            msg_imu.linear_acceleration.z = self.imu.AccelVals[2]
            msg_imu.linear_acceleration_covariance = [0.10, 0.0, 0.0, 0.0, 0.10, 0.0, 0.0, 0.0, 0.10]

            msg_imu.angular_velocity.x = (self.imu.GyroVals[0])  # rad/s
            msg_imu.angular_velocity.y = (self.imu.GyroVals[1])
            msg_imu.angular_velocity.z = (self.imu.GyroVals[2])
            msg_imu.angular_velocity_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]

            # Calculate euler angles, convert to quaternion and store in message
            # Convert to quaternion
            yaw_r = self.wrap_pi(radians(yaw))
            quat = tf_transformations.quaternion_from_euler(radians(roll), radians(pitch), yaw_r)
            msg_imu.orientation.x = quat[0]
            msg_imu.orientation.y = quat[1]
            msg_imu.orientation.z = quat[2]
            msg_imu.orientation.w = quat[3]
            msg_imu.orientation_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.10]

        self.publisher_imu_values_.publish(msg_imu)

        msg_mag = MagneticField()
        msg_mag.header.stamp = now.to_msg()
        msg_mag.header.frame_id = frame_id
        # mag covariance unknown for now - uncalibrated mag, no noise model
        msg_mag.magnetic_field_covariance[0] = -1.0

        msg_mag.magnetic_field.x = magVals_rotated[0]  # Tesla
        msg_mag.magnetic_field.y = magVals_rotated[1]
        msg_mag.magnetic_field.z = magVals_rotated[2]
        self.publisher_mag_values_.publish(msg_mag)

        # Accumulate temp for averaging
        self._temp_sum_c += self.imu.Temp
        self._temp_count += 1
        publish_temp_now = (self._temp_count % self._temp_div) == 0

        if publish_temp_now:
            avg_temp_c = self._temp_sum_c / float(self._temp_count)
            msg_temp = Temperature()
            msg_temp.header.stamp = msg_imu.header.stamp
            msg_temp.header.frame_id = frame_id
            msg_temp.temperature = round(avg_temp_c, 2)  # Celsius
            msg_temp.variance = 0.25 # +- 0.5 degrees C squared
            self.publisher_temperature.publish(msg_temp)
            # Reset accumulator for next window
            self._temp_sum_c = 0.0
            self._temp_count = 0

        # print not too often:
        if self.print and publish_temp_now:
            MagVals_uT = magVals_rotated * 1e6  # convert to microTesla

            self.logger.info(f"MagVals:   x={MagVals_uT[0]:8.2f} y={MagVals_uT[1]:8.2f} z={MagVals_uT[2]:8.2f} micro Tesla")

            AccelVals = self.imu.AccelVals
            self.logger.info(f"AccelVals: x={AccelVals[0]:8.4f} y={AccelVals[1]:8.4f} z={AccelVals[2]:8.4f} m/s²")

            GyroVals = self.imu.GyroVals
            self.logger.info(f"GyroVals:  x={GyroVals[0]:8.4f} y={GyroVals[1]:8.4f} z={GyroVals[2]:8.4f} rad/s")

            # print only when fusion is enabled, and orientation is valid:
            if (not self.raw_only) and self._orientation_valid:
                #roll, pitch, yaw = self.filter.quaternion_rpy()      # ENU frame, yaw=0 East
                roll, pitch, yaw = self.filter.quaternion_rpy_nav()  # Navigation frame, yaw=0 North
                self.logger.info(
                    f"Orientation: roll={math.degrees(roll):.2f}, pitch={math.degrees(pitch):.2f}, yaw={math.degrees(yaw):.2f} degrees North"
                )

def main(args=None):
    rclpy.init(args=args)
    node = MPU9250Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl-C received, shutting down...")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()