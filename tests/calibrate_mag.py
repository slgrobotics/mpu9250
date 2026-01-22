import time
from smbus2 import SMBus

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mpu9250.imusensor.MPU9250 import MPU9250

#
# see https://github.com/niru-5/imusensor/blob/master/README.md#basic-usage
#


def read_orientation(imu, count, message=""):
    """Read and print IMU orientation for specified number of iterations."""
    if message:
        print(message)
    
    for i in range(count):
        imu.readSensor()
        imu.computeOrientation()

        MagVals_uT = imu.MagVals * 1e6  # convert to microTesla

        print(f"MagVals: x={MagVals_uT[0]:8.2f} y={MagVals_uT[1]:8.2f} z={MagVals_uT[2]:8.2f} uT    roll:{imu.roll:8.2f}     pitch:{imu.pitch:8.2f}     yaw:{imu.yaw:8.2f} degrees")

        time.sleep(0.2)


def main():
    address = 0x68
    bus = SMBus(1)

    print(f"IP: Initializing MPU9250 at address {hex(address)} on I2C bus 1")

    imu = MPU9250.MPU9250(bus, address)
    imu.begin()

    # Note: initial values for biases and scales in imu object:
    #		MagBias = np.array([0.0, 0.0, 0.0])
    #		Mags = np.array([1.0, 1.0, 1.0])      # optional magnetometer scale adjustment
    #       Magtransform = None  # magnetometer calibration is unknown. A 3x3 matrix, calculated in calibrateMagPrecise()

    print("OK: IMU initialized")

    # imu.calibrateGyro()
    # imu.calibrateAccelerometer()
    # or load your own calibration file
    #imu.loadCalibDataFromFile("/home/pi/calib_real_bolder.json")

    # Note: Make sure you rotate the sensor in 8 shape and cover all the pitch and roll angles.

    read_orientation(imu, 5, "IP: Reading initial mag values and orientation")

    print("calibrating - rotate the sensor in 8 shape and cover all the pitch and roll angles")

    #imu.calibrateMagApprox()
    imu.calibrateMagPrecise()

    print()
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
                print(f"    {row_str}]")
    print()

    read_orientation(imu, 10, "IP: Reading mag values and orientation after calibration")

    print("Done")


if __name__ == "__main__":
    main()
