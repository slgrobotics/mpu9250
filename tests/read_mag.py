import time
import numpy as np
from smbus2 import SMBus

from mpu9250.imusensor.MPU9250 import MPU9250

#
# see https://github.com/niru-5/imusensor/blob/master/README.md#basic-usage
#

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


imu.MagBias = np.array([3.14156455e-05, 6.44644552e-06, 2.43798173e-05])
imu.Mags = np.array([1., 1., 1.])
imu.Magtransform = np.array([[ 1.05464768, -0.03982411, -0.01269338],
                            [-0.03982411,  0.93736574, -0.01455147],
                            [-0.01269338, -0.01455147,  1.01356064]])

print(f"MagBias: {imu.MagBias}")
print(f"Mags: {imu.Mags}")
print(f"Magtransform: {imu.Magtransform}")

print("IP: Reading mag values and orientation")

while True:
    imu.readSensor()
    imu.computeOrientation()

    print(f"MagVals: {imu.MagVals}     roll:{imu.roll:8.2f}     pitch:{imu.pitch:8.2f}     yaw:{imu.yaw:8.2f}")

    time.sleep(0.2)

print("Done")
