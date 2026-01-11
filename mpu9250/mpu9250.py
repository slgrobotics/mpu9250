import smbus2 as smbus
import math
import numpy as np

from .imusensor.MPU9250 import MPU9250
from .imusensor.filters import kalman 
from .imusensor.filters import madgwick

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField, Temperature

from math import sin, cos, radians
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
                ('magnetometer_bias', [1.0, 1.0, 1.0]),
                ('magnetometer_transform', [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]),
            ]
        )

        address = self.get_parameter('i2c_address')._value
        bus = smbus.SMBus(self.get_parameter('i2c_port')._value)
        self.imu = MPU9250.MPU9250(bus, address)
        self.raw_only = self.get_parameter('raw_only')._value

        self.imu.Accels = np.asarray(self.get_parameter('acceleration_scale')._value)
        self.imu.AccelBias = np.asarray(self.get_parameter('acceleration_bias')._value)
        self.imu.GyroBias = np.asarray(self.get_parameter('gyro_bias')._value)
        self.imu.Mags = np.asarray(self.get_parameter('magnetometer_scale')._value)
        self.imu.MagBias = np.asarray(self.get_parameter('magnetometer_bias')._value)
        self.imu.Magtransform = np.reshape(np.asarray(self.get_parameter('magnetometer_transform')._value),(3,3))

        pub_rate_hz = self.get_parameter('frequency')._value
        self.timer_publish_imu_values_ = self.create_timer(1.0/pub_rate_hz, self.publish_imu_values)

        self.publisher_imu_values_ = self.create_publisher(Imu, "/imu/data_raw" if self.raw_only else "/imu/data", 10)  # raw or fused IMU values
        
        self.publisher_mag_values_ = self.create_publisher(MagneticField, "/imu/mag", 10)  # raw magnetometer values

        temp_pub_rate_hz = float(self.get_parameter("temp_pub_rate_hz").value)
        self.publisher_temperature = self.create_publisher(Temperature, "/imu/temp", 10)

        # Temperature averaging (accumulate at IMU rate, publish averaged at ~temp_pub_rate_hz)
        self._temp_sum_c = 0.0
        self._temp_count = 0
        # Divider: publish temperature every N IMU ticks
        self._temp_div = max(1, int(round(pub_rate_hz / max(0.1, temp_pub_rate_hz))))

        if not self.raw_only:
            self.sensorfusion = kalman.Kalman()
            #self.sensorfusion = madgwick.Madgwick(0.5)
            self.imu.begin()
            self.imu.readSensor()    
            self.imu.computeOrientation()
            self.sensorfusion.roll = self.imu.roll
            self.sensorfusion.pitch = self.imu.pitch
            self.sensorfusion.yaw = self.imu.yaw

        self.deltaTime = 0
        self.lastTime = self.get_clock().now()

    def wrap_pi(self, angle):
        # Wraps the given angle(s) to +/- pi.
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def publish_imu_values(self):
        self.imu.readSensor()
        now = self.get_clock().now()
        deltaTime = (now - self.lastTime).nanoseconds * 10e9
        self.lastTime = now
        frame_id = self.get_parameter('frame_id')._value

        if not self.raw_only:
            #self.imu.computeOrientation()
            #yaw = self.imu.yaw
            #pitch = self.imu.pitch
            #roll = self.imu.roll
            #computeAndUpdateRollPitchYaw

            self.sensorfusion.computeAndUpdateRollPitchYaw(\
                self.imu.AccelVals[0], self.imu.AccelVals[1], self.imu.AccelVals[2],\
                self.imu.GyroVals[0], self.imu.GyroVals[1], self.imu.GyroVals[2],\
                self.imu.MagVals[0], self.imu.MagVals[1], self.imu.MagVals[2], deltaTime)
            yaw = self.sensorfusion.yaw
            pitch = self.sensorfusion.pitch
            roll = self.sensorfusion.roll

            # Convert yaw to the ENU (East-North_Up):
            yaw += 90.0

        msg_imu = Imu()
        msg_imu.header.stamp = now.to_msg()
        msg_imu.header.frame_id = frame_id
        if self.raw_only:
            # Raw measurements, unknown covariance
            msg_imu.linear_acceleration.x = self.imu.RawAccelVals[0]
            msg_imu.linear_acceleration.y = self.imu.RawAccelVals[1]
            msg_imu.linear_acceleration.z = self.imu.RawAccelVals[2]
            msg_imu.linear_acceleration_covariance[0] = -1.0

            msg_imu.angular_velocity.x = self.imu.RawGyroVals[0]
            msg_imu.angular_velocity.y = self.imu.RawGyroVals[1]
            msg_imu.angular_velocity.z = self.imu.RawGyroVals[2]
            msg_imu.angular_velocity_covariance[0] = -1.0

            # No orientation in raw data
            msg_imu.orientation_covariance[0] = -1.0
        else:
            # Fused measurements with covariance
            msg_imu.linear_acceleration.x = self.imu.AccelVals[0]
            msg_imu.linear_acceleration.y = self.imu.AccelVals[1]
            msg_imu.linear_acceleration.z = self.imu.AccelVals[2]
            msg_imu.linear_acceleration_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]

            msg_imu.angular_velocity.x = (self.imu.GyroVals[0]) #TODO this is acceleration not velocity (?!)
            msg_imu.angular_velocity.y = (self.imu.GyroVals[1]) #TODO this is acceleration not velocity (?!)
            msg_imu.angular_velocity.z = (self.imu.GyroVals[2]) #TODO this is acceleration not velocity (?!)
            msg_imu.angular_velocity_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]

            # Calculate euler angles, convert to quaternion and store in message
            # Convert to quaternion
            yaw_r = self.wrap_pi(radians(yaw))
            quat = tf_transformations.quaternion_from_euler(radians(roll), radians(pitch), yaw_r)
            msg_imu.orientation.x = quat[0]
            msg_imu.orientation.y = quat[1]
            msg_imu.orientation.z = quat[2]
            msg_imu.orientation.w = quat[3]
            msg_imu.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]

        self.publisher_imu_values_.publish(msg_imu)

        msg_mag = MagneticField()
        msg_mag.header.stamp = now.to_msg()
        msg_mag.header.frame_id = frame_id
        # mag covariance unknown for now - uncalibrated mag, no noise model
        msg_mag.magnetic_field_covariance[0] = -1.0
        msg_mag.magnetic_field.x = self.imu.RawMagVals[0]
        msg_mag.magnetic_field.y = self.imu.RawMagVals[1]
        msg_mag.magnetic_field.z = self.imu.RawMagVals[2]
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
            msg_temp.temperature = round(avg_temp_c, 2)
            msg_temp.variance = 0.0 # 0 means unknown
            self.publisher_temperature.publish(msg_temp)
            # Reset accumulator for next window
            self._temp_sum_c = 0.0
            self._temp_count = 0

        # print not too often:
        if self.get_parameter('print')._value and publish_temp_now:
            #print("roll: {:8.2f} \tpitch : {:8.2f} \tyaw : {:8.2f}".format(self.sensorfusion.roll, self.sensorfusion.pitch, self.sensorfusion.yaw))
            #print("roll: {:8.2f} \tpitch : {:8.2f} \tyaw : {:8.2f}".format(roll, pitch, yaw_r))
            print("yaw : {:8.2f}".format(yaw_r))

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

def main(args=None):
    rclpy.init(args=args)
    node = MPU9250Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()