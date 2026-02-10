import time
import numpy as np
from smbus2 import SMBus

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mpu9250.imusensor.MPU9250 import MPU9250

#
# see https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/MPU9250.md#calibration
#     https://github.com/niru-5/imusensor/blob/master/README.md#basic-usage
#

address = 0x68
bus = SMBus(1)

print(f"IP: Initializing MPU9250 at address {hex(address)} on I2C bus 1")

imu = MPU9250(bus, address)
imu.begin()

# Note: initial values for biases and scales in imu object:
#		MagBias = np.array([0.0, 0.0, 0.0])
#		MagScale = np.array([1.0, 1.0, 1.0])      # optional magnetometer scale adjustment
#       Magtransform = None  # magnetometer calibration is unknown. A 3x3 matrix, calculated in calibrateMagPrecise()

print("OK: IMU initialized")

"""
imu.MagBias = np.array([3.14156455e-05, 6.44644552e-06, 2.43798173e-05])
imu.MagScale = np.array([1., 1., 1.])
imu.Magtransform = np.array([[ 1.05464768, -0.03982411, -0.01269338],
                            [-0.03982411,  0.93736574, -0.01455147],
                            [-0.01269338, -0.01455147,  1.01356064]])
"""

print(f"MagBias: {imu.MagBias}")
print(f"MagScale: {imu.MagScale}")
print(f"Magtransform: {imu.Magtransform}")

print("IP: Reading mag values and orientation")

while True:
    imu.readSensor()  # applies calibration internally, if you initialized MagBias, MagScale, Magtransform as in the comment above
    imu.computeOrientation()

    MagVals_uT = imu.MagVals * 1e6  # convert to microTesla

    print(f"MagVals:   x={MagVals_uT[0]:8.2f} y={MagVals_uT[1]:8.2f} z={MagVals_uT[2]:8.2f} uT    roll:{imu.roll:8.2f}     pitch:{imu.pitch:8.2f}     yaw:{imu.yaw:8.2f} degrees")

    AccelVals = imu.AccelVals
    print(f"AccelVals: x={AccelVals[0]:8.4f} y={AccelVals[1]:8.4f} z={AccelVals[2]:8.4f} m/s²")
    
    GyroVals = imu.GyroVals
    print(f"GyroVals:  x={GyroVals[0]:8.4f} y={GyroVals[1]:8.4f} z={GyroVals[2]:8.4f} rad/s")

    time.sleep(0.2)

print("Done")

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
