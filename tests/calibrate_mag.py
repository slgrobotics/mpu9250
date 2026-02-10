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

imu = None

def read_orientation(count, message=""):

    global imu

    """Read and print IMU orientation for specified number of iterations."""
    if message:
        print(message)
    
    for i in range(count):
        imu.readSensor()  # applies calibration internally, when MagBias, MagScale, Magtransform are filled after the calibration step
        imu.computeOrientation()

        MagVals_uT = imu.MagVals * 1e6  # convert to microTesla

        print(f"MagVals: x={MagVals_uT[0]:8.2f} y={MagVals_uT[1]:8.2f} z={MagVals_uT[2]:8.2f} uT    roll:{imu.roll:8.2f}     pitch:{imu.pitch:8.2f}     yaw:{imu.yaw:8.2f} degrees")

        time.sleep(0.2)

def print_calibration():

    print("--------------------------------------------------------------------------------")
    print()
    print("---- Calibration results: copy this and paste into your ROS2 launch file:")
    print()
    print("#\"magnetometer_scale\": [" + ", ".join(f"{x}" for x in imu.MagScale) + "],  # should be 1.0 or omitted if \"magnetometer_transform\" is present")
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
    print("---- Calibration results in Python for direct assignment (e.g. into read_mag.py)")
    print()
    # MagBias
    bias_str = ", ".join(f"{v:.16e}" for v in imu.MagBias)
    print(f"imu.MagBias = np.array([{bias_str}])")

    # Magtransform
    print("imu.Magtransform = np.array([")
    for i, row in enumerate(imu.Magtransform):
        row_str = ", ".join(f"{v:.16e}" for v in row)
        if i < len(imu.Magtransform) - 1:
            print(f"    [{row_str}],")
        else:
            print(f"    [{row_str}]")
    print("])")
    print("--------------------------------------------------------------------------------")

def main():

    global imu

    address = 0x68
    bus = SMBus(1)

    print(f"IP: Initializing MPU9250 at address {hex(address)} on I2C bus 1")

    imu = MPU9250(bus, address)
    imu.begin()

    # Note: initial values for biases and scales in imu object:
    #		MagBias = np.array([0.0, 0.0, 0.0])
    #		MagScale = np.array([1.0, 1.0, 1.0])      # should be 1.0 or omitted if "magnetometer_transform" is present
    #       Magtransform = None  # magnetometer calibration is unknown. A 3x3 matrix, calculated in calibrateMagPrecise()

    print("OK: IMU initialized")

    # imu.calibrateGyro()
    # imu.calibrateAccelerometer()
    # or load your own calibration file
    #imu.loadCalibDataFromFile("/tmp/calib_icm_20948.json")

    read_orientation(5, "IP: Reading initial mag values and orientation")

    print("calibrating - rotate the sensor in 8 shape and cover all the pitch and roll angles")

    # Note: at this point the initial values for biases and scales in imu object:
    #		MagBias = np.array([0.0, 0.0, 0.0])
    #		MagScale = np.array([1.0, 1.0, 1.0])      # should be 1.0 or omitted if "magnetometer_transform" is present
    #       Magtransform = None  # magnetometer calibration is unknown. A 3x3 matrix, calculated in calibrateMagPrecise()

    #imu.calibrateMagApprox()
    imu.calibrateMagPrecise()

    # Here we have the calculated calibration values in imu object. Print them for a ROS2 launch file:
    print_calibration()

    input("\nCalibration complete. Press Enter to see calibrated values...")

    """
    Run tests/read_mag.py
    Rotate the robot in place.
    The published values should roughly conform to the following matrix:

        ENU    |    x    |    y    |    z    |
      ----------------------------------------   When robot rotates in place:
        North  |   +20   |     0   |   -40   |     N -> S  x changes from + to - (y stays the same around 0)
        East   |     0   |   +20   |   -40   |     E -> W  y changes from + to - (x stays the same around 0)
        South  |   -20   |     0   |   -40   |     z axis is Up; Earth field in the US typically has negative z (points down into Earth)
        West   |     0   |   -20   |   -40   |     z shouldn't change much
      ----------------------------------------
    values are in microTesla (µT), Earth's field is about 25 to 65 µT depending on location
    See https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?#igrfwmm - magnetic field by location (microTesla, NED frame)
    """

    read_orientation(10000, "IP: Reading mag values and orientation after calibration")

    print("Done")

def signal_handler(sig, frame):
    print_calibration()
    print('\nExiting...')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


if __name__ == "__main__":
    main()
