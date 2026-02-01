import smbus2 as smbus
import math
import numpy as np

from .imusensor.MPU9250 import MPU9250
from .imusensor.filters import madgwick

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField, Temperature

class MPU9250Node(Node):
    def __init__(self):
        super().__init__("mpu9250_node")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('verbose', False),
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
                ("madgwick_beta", 0.05),   # 0.04-0.2 typical, 0.01 for faster settling after rotation
                ("madgwick_use_mag", True)
            ]
        )

        self.logger = self.get_logger()

        address = self.get_parameter('i2c_address').value
        port = self.get_parameter('i2c_port').value
        bus = smbus.SMBus(port)

        self.logger.info(f"   i2c_addr: 0x{address:X} on I2C bus {port}")

        self.imu = MPU9250(bus, address)

        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        self.logger.info(f"   verbose: {self.verbose}")

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
        self.imu.MagScale = np.asarray(self.get_parameter('magnetometer_scale').get_parameter_value().double_array_value)
        self.imu.MagBias = np.asarray(self.get_parameter('magnetometer_bias').get_parameter_value().double_array_value)
        self.imu.Magtransform = np.reshape(np.asarray(self.get_parameter('magnetometer_transform').get_parameter_value().double_array_value),(3,3))

        pub_rate_hz = self.get_parameter('frequency').get_parameter_value().integer_value
        self.pub_clk = self.create_timer(1.0/pub_rate_hz, self.publish_imu_values)

        self.imu_raw_pub = self.create_publisher(Imu, "/imu/data_raw", 10)  # raw IMU values
        self.mag_pub = self.create_publisher(MagneticField, "/imu/mag", 10)  # raw magnetometer values
        self.temp_pub = self.create_publisher(Temperature, "/imu/temp", 10)

        # Temperature averaging (accumulate at IMU rate, publish averaged at ~temp_pub_rate_hz)
        self._temp_sum_c = 0.0
        self._temp_count = 0
        # Divider: publish temperature every N IMU ticks
        self._temp_div = max(1, int(round(pub_rate_hz / max(0.1, temp_pub_rate_hz))))

        # Madgwick-specific variables:
        self._orientation_valid = False
        self._madgwick_updates = 0

        self._last_stamp = None
        self._shutting_down = False

        # --- Preallocate messages (avoid per-tick allocations) ---
        self._imu_raw_msg = Imu()
        self._imu_raw_msg.header.frame_id = self.frame_id
        self._imu_raw_msg.orientation_covariance[0] = -1.0
        self._imu_raw_msg.linear_acceleration_covariance[0] = -1.0
        self._imu_raw_msg.angular_velocity_covariance[0] = -1.0

        self._mag_msg = MagneticField()
        self._mag_msg.header.frame_id = self.frame_id
        # mag covariance unknown for now - uncalibrated mag, no noise model
        self._mag_msg.magnetic_field_covariance[0] = -1.0

        self._temp_msg = Temperature()
        self._temp_msg.header.frame_id = self.frame_id
        self._temp_msg.variance = 0.25 # +- 0.5 degrees C squared

        self._imu_msg = None

        self.imu.begin()  # takes time for quick gyro calibration, gets factory mag calibration

        # expect MagScale: [1.79882812e-07 1.79882812e-07 1.73437500e-07]
        self.logger.info(f"   Magnetometer Sensitivity Scales: {self.imu.MagScale}  LSB/Tesla")
        self.logger.info(f"   Gyro Bias: {self.imu.GyroBias}  rad/s")

        if self.raw_only:
            self.logger.info("   IMU node configured for raw data only; no orientation filter will be used.")
        else:
            self.logger.info("   IMU node configured to provide fused orientation data; Madgwick filter used.")

            self._imu_msg = Imu()
            self._imu_msg.header.frame_id = self.frame_id

            # Defaults for "unknown"; overwrite when valid
            self._imu_msg.orientation_covariance[0] = -1.0
            self._imu_msg.orientation_covariance[4] = 0.0
            self._imu_msg.orientation_covariance[8] = 0.0

            # Static accel/gyro covariances (tune later)
            self._imu_msg.linear_acceleration_covariance[0] = 0.10
            self._imu_msg.linear_acceleration_covariance[4] = 0.10
            self._imu_msg.linear_acceleration_covariance[8] = 0.10
            self._imu_msg.angular_velocity_covariance[0] = 0.02
            self._imu_msg.angular_velocity_covariance[4] = 0.02
            self._imu_msg.angular_velocity_covariance[8] = 0.02

            # Madgwick params
            self.madgwick_beta = float(self.get_parameter("madgwick_beta").value)
            self.madgwick_use_mag = self.get_parameter("madgwick_use_mag").get_parameter_value().bool_value

            self.logger.info(f"   madgwick_beta: {self.madgwick_beta}")
            self.logger.info(f"   madgwick_use_mag: {self.madgwick_use_mag}")

            # Fused data publisher (with orientation):
            self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)

            # Initialize sensor fusion algorithm:
            self.filter = madgwick.Madgwick(self.madgwick_beta)  # usually beta around 0.1

            # preallocate vectors for Madgwick filter once to avoid doing it every tick:
            self._gyro_vec = [0.0, 0.0, 0.0]
            self._acc_vec  = [0.0, 0.0, 0.0]
            self._mag_vec  = [0.0, 0.0, 0.0]

            # ---- Initialize filter if enabled ----
            if (not self.raw_only) and self.madgwick_use_mag:
                try:
                    # Initial read to set roll/pitch/yaw:
                    self.imu.readSensor()    
                    #self.rotate_mag()       # rotate to align with accel/gyro frame

                    # Use bias-corrected accel values:
                    self._acc_vec[0]  = self.imu.AccelVals[0]; self._acc_vec[1]  = self.imu.AccelVals[1]; self._acc_vec[2]  = self.imu.AccelVals[2]

                    if self.madgwick_use_mag:
                        self._mag_vec[0] = self.imu.MagVals[0]
                        self._mag_vec[1] = self.imu.MagVals[1]
                        self._mag_vec[2] = self.imu.MagVals[2]

                    # use method that rotates mag values to align with accel/gyro frame
                    rpy = self.filter.initialize_from_accel_mag(
                            self._acc_vec[0], self._acc_vec[1], self._acc_vec[2],
                            self._mag_vec[0], self._mag_vec[1], self._mag_vec[2])

                    if rpy[0] is None:
                        self.logger.warning("Madgwick init failed (invalid accel/mag). Keeping identity quaternion.")
                    else:
                        #roll, pitch, yaw = rpy  # ENU frame, yaw=0 East
                        roll, pitch, yaw = self.filter.quaternion_rpy_nav()  # Navigation frame, yaw=0 North
                        self.logger.info(
                            "Madgwick init:"
                            f" roll={np.degrees(roll): .2f} deg,"
                            f" pitch={np.degrees(pitch): .2f} deg,"
                            f" yaw={np.degrees(yaw): .2f} deg"
                        )
                except Exception as e:
                    self._orientation_valid = False
                    if self._imu_msg is not None:
                        self._imu_msg.orientation_covariance[0] = -1.0
                        self._imu_msg.orientation_covariance[4] = 0.0
                        self._imu_msg.orientation_covariance[8] = 0.0
                    self.logger.warning(f"Madgwick init threw exception: {e}")


        self._last_stamp = self.get_clock().now()

        self.logger.info("OK: MPU9250 Node: init successful")

    def rotate_mag(self):
        """
        Rotate magnetometer reading in place to align with accel+gyro frame.
        Inputs:
            mag_values, a numpy.ndarray — a 1D array of 3 float64 values, magnetometer readings
        Returns:
            numpy.ndarray — a 1D array of 3 float64 values : rotated magnetometer readings
        """

        #   Accel/Gyro: X forward, Y left, Z up
        #   Mag:        X left, Y forward, Z down
        mx, my, mz = self.imu.MagVals

        self.imu.MagVals[0] =  my
        self.imu.MagVals[1] =  mx
        self.imu.MagVals[2] = -mz

    def publish_imu_values(self):
        """
        Publishes:
        /imu/data_raw : accel+gyro only, orientation unknown (orientation_cov[0] = -1)
        /imu/data     : accel+gyro + orientation (Madgwick) when raw_only == False
        /imu/mag      : magnetometer (MagneticField)
        /imu/temp     : averaged temperature
        """

        if self._shutting_down:
            return

        # publish only fresh
        #if not self.imu.dataReady():
        #    return

        now = self.get_clock().now()
        try:
            self.imu.readSensor()
            #self.rotate_mag()       # rotate to align with accel/gyro frame
        except Exception as e:
            self.logger.error(f"MPU9250 getAgmt() failed: {e}")
            return

        stamp = now.to_msg()

        # Update headers (stamp changes every tick)
        self._imu_raw_msg.header.stamp = stamp
        self._mag_msg.header.stamp = stamp
        if self._imu_msg is not None:
            self._imu_msg.header.stamp = stamp

        # --- dt for filter ---
        dt = 1.0 / float(self.pub_rate_hz)
        if self._last_stamp is not None:
            dt = (now - self._last_stamp).nanoseconds * 1e-9
            if not (0.0 < dt <= 0.2):
                dt = 1.0 / float(self.pub_rate_hz)
        self._last_stamp = now

        # Raw measurements, unknown covariance
        self._imu_raw_msg.linear_acceleration.x = self.imu.AccelVals[0]  # m/s^2
        self._imu_raw_msg.linear_acceleration.y = self.imu.AccelVals[1]
        self._imu_raw_msg.linear_acceleration.z = self.imu.AccelVals[2]

        self._imu_raw_msg.angular_velocity.x = self.imu.GyroVals[0]  # rad/s
        self._imu_raw_msg.angular_velocity.y = self.imu.GyroVals[1]
        self._imu_raw_msg.angular_velocity.z = self.imu.GyroVals[2]

        # No orientation in raw data
        self._imu_raw_msg.orientation_covariance[0] = -1.0

        self.imu_raw_pub.publish(self._imu_raw_msg)

        if not self.raw_only:
            try:
                self.filter.setSamplePeriod(dt)

                # Use bias-corrected accel+gyro values:
                self._gyro_vec[0] = self.imu.GyroVals[0]; self._gyro_vec[1] = self.imu.GyroVals[1]; self._gyro_vec[2] = self.imu.GyroVals[2]
                self._acc_vec[0]  = self.imu.AccelVals[0]; self._acc_vec[1]  = self.imu.AccelVals[1]; self._acc_vec[2]  = self.imu.AccelVals[2]

                if self.madgwick_use_mag:
                    self._mag_vec[0] = self.imu.MagVals[0]
                    self._mag_vec[1] = self.imu.MagVals[1]
                    self._mag_vec[2] = self.imu.MagVals[2]
                    self.filter.update(self._gyro_vec, self._acc_vec, self._mag_vec)
                else:
                    self.filter.update(self._gyro_vec, self._acc_vec)

                self._madgwick_updates += 1
                if self._madgwick_updates >= 20:
                    self._orientation_valid = True

                if self._orientation_valid:
                    qx, qy, qz, qw = self.filter.quaternion_xyzw()
                    self._imu_msg.orientation.x = qx
                    self._imu_msg.orientation.y = qy
                    self._imu_msg.orientation.z = qz
                    self._imu_msg.orientation.w = qw
                    self._imu_msg.orientation_covariance[0] = 0.05
                    self._imu_msg.orientation_covariance[4] = 0.05
                    self._imu_msg.orientation_covariance[8] = 0.10

            except Exception as e:
                self._orientation_valid = False
                self._imu_msg.orientation_covariance[0] = -1.0
                self._imu_msg.orientation_covariance[4] = 0.0
                self._imu_msg.orientation_covariance[8] = 0.0
                self.logger.warning(f"Madgwick update failed: {e}")

            # Fused measurements with covariance
            self._imu_msg.linear_acceleration.x = self.imu.AccelVals[0]  # m/s^2
            self._imu_msg.linear_acceleration.y = self.imu.AccelVals[1]
            self._imu_msg.linear_acceleration.z = self.imu.AccelVals[2]

            self._imu_msg.angular_velocity.x = (self.imu.GyroVals[0])  # rad/s
            self._imu_msg.angular_velocity.y = (self.imu.GyroVals[1])
            self._imu_msg.angular_velocity.z = (self.imu.GyroVals[2])

            self.imu_pub.publish(self._imu_msg)

        self._mag_msg.magnetic_field.x = self.imu.MagVals[0]  # Tesla
        self._mag_msg.magnetic_field.y = self.imu.MagVals[1]
        self._mag_msg.magnetic_field.z = self.imu.MagVals[2]
        self.mag_pub.publish(self._mag_msg)

        # Accumulate temp for averaging
        self._temp_sum_c += self.imu.Temp
        self._temp_count += 1
        publish_temp_now = (self._temp_count % self._temp_div) == 0

        if publish_temp_now:
            avg_temp_c = self._temp_sum_c / float(self._temp_count)
            self._temp_msg.header.stamp = stamp
            self._temp_msg.temperature = round(avg_temp_c, 2)  # Celsius
            self.temp_pub.publish(self._temp_msg)
            # Reset accumulator for next window
            self._temp_sum_c = 0.0
            self._temp_count = 0

        # print not too often:
        if self.verbose and publish_temp_now:
            MagVals_uT = self.imu.MagVals * 1e6  # convert to microTesla

            self.logger.info(f"MagVals:   x={MagVals_uT[0]:8.2f} y={MagVals_uT[1]:8.2f} z={MagVals_uT[2]:8.2f} micro Tesla")

            AccelVals = self.imu.AccelVals
            self.logger.info(f"AccelVals: x={AccelVals[0]:8.4f} y={AccelVals[1]:8.4f} z={AccelVals[2]:8.4f} m/s²")

            GyroVals = self.imu.GyroVals
            self.logger.info(f"GyroVals:  x={GyroVals[0]:8.4f} y={GyroVals[1]:8.4f} z={GyroVals[2]:8.4f} rad/s")

            # print only when fusion is enabled, and orientation is valid:
            if not self.raw_only:
                #roll, pitch, yaw = self.filter.quaternion_rpy()     # ENU frame, yaw=0 East
                roll, pitch, yaw = self.filter.quaternion_rpy_nav()  # Navigation frame, yaw=0 North
                self.logger.info(
                    f"Orientation roll={math.degrees(roll):.2f}, pitch={math.degrees(pitch):.2f}, yaw={math.degrees(yaw):.2f} degrees North"
                )

    def destroy_node(self):
        self._shutting_down = True
        try:
            if hasattr(self, "pub_clk") and self.pub_clk is not None:
                self.pub_clk.cancel()
        except Exception:
            pass
        return super().destroy_node()

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
