
import time
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

print("OK: IMU initialized")

# imu.calibrateGyro()
# imu.calibrateAccelerometer()
# or load your own calibration file
#imu.loadCalibDataFromFile("/home/pi/calib_real_bolder.json")

# Note: Make sure you rotate the sensor in 8 shape and cover all the pitch and roll angles.

#imu.calibrateMagPrecise()
#imu.calibrateMagApprox()

while True:
    imu.readSensor()
    imu.computeOrientation()

    print(f"roll:{imu.roll:8.2f}     pitch:{imu.pitch:8.2f}     yaw:{imu.yaw:8.2f}")

    time.sleep(0.2)
