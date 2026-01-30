import sys
import os
import signal
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from smbus2 import SMBus
from mpu9250.imusensor.MPU9250 import MPU9250

#
# see https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/MPU9250.md#calibration
#     https://github.com/niru-5/imusensor/blob/master/README.md#basic-usage
#


def read_orientation(imu, count, message=""):
    """Read and print IMU orientation for specified number of iterations."""
    if message:
        print(message)
    
    for i in range(count):
        imu.readSensor()  # applies calibration internally, when MagBias, Mags, Magtransform are filled after the calibration step
        imu.computeOrientation()

        MagVals_uT = imu.MagVals * 1e6  # convert to microTesla

        print(f"MagVals: x={MagVals_uT[0]:8.2f} y={MagVals_uT[1]:8.2f} z={MagVals_uT[2]:8.2f} uT    roll:{imu.roll:8.2f}     pitch:{imu.pitch:8.2f}     yaw:{imu.yaw:8.2f} degrees")

        time.sleep(0.2)

def print_calibration():

    print()
    print("Calibration results: copy this and paste into your ROS2 launch file:")
    print()
    print("\"magnetometer_scale\": [" + ", ".join(f"{x}" for x in imu.Mags) + "],  # should be around 1.0")
    print("\"magnetometer_bias\": [" + ", ".join(f"{x}" for x in imu.MagBias) + "],")
    if imu.Magtransform is not None:
        print("\"magnetometer_transform\": [")
        for i, row in enumerate(imu.Magtransform):
            row_str = ", ".join(f"{x}" for x in row)
            if i < len(imu.Magtransform) - 1:
                print(f"    {row_str},")
            else:
                print(f"    {row_str}],")
    print()

def main():
    address = 0x68
    bus = SMBus(1)

    print(f"IP: Initializing MPU9250 at address {hex(address)} on I2C bus 1")

    imu = MPU9250(bus, address)
    imu.begin()

    # Note: initial values for biases and scales in imu object:
    #		MagBias = np.array([0.0, 0.0, 0.0])
    #		Mags = np.array([1.0, 1.0, 1.0])      # optional magnetometer scale adjustment
    #       Magtransform = None  # magnetometer calibration is unknown. A 3x3 matrix, calculated in calibrateMagPrecise()

    print("OK: IMU initialized")

    # imu.calibrateGyro()
    # imu.calibrateAccelerometer()
    # or load your own calibration file
    #imu.loadCalibDataFromFile("/tmp/calib_icm_20948.json")

    read_orientation(imu, 5, "IP: Reading initial mag values and orientation")

    print("calibrating - rotate the sensor in 8 shape and cover all the pitch and roll angles")

    # Note: at this point the initial values for biases and scales in imu object:
    #		MagBias = np.array([0.0, 0.0, 0.0])
    #		Mags = np.array([1.0, 1.0, 1.0])      # optional magnetometer scale adjustment
    #       Magtransform = None  # magnetometer calibration is unknown. A 3x3 matrix, calculated in calibrateMagPrecise()

    #imu.calibrateMagApprox()
    imu.calibrateMagPrecise()

    # Here we have the calculated calibration values in imu object. Print them for a ROS2 launch file:
    print_calibration()

    input("\nCalibration complete. Press Enter to see calibrated values...")

    """
    Rotate the robot in place.
    The published values should roughly conform to the following matrix:

            |   x   |   y   |   z   |
    ----------------------------------
    North  |  20   |   0   |  -40  |
    East   |   0   |  20   |  -40  |
    South  | -20   |   0   |  -40  |
    West   |   0   |  -20  |  -40  |
    ----------------------------------
    """

    read_orientation(bus, 10000, "IP: Reading mag values and orientation after calibration")

    print("Done")

def signal_handler(sig, frame):
    print_calibration()
    print('\nExiting...')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


if __name__ == "__main__":
    main()
