
import time
import smbus

from mpu9250.imusensor.MPU9250 import MPU9250

#
# see https://github.com/niru-5/imusensor/blob/master/README.md#basic-usage
#

address = 0x68
bus = smbus.SMBus(1)

imu = MPU9250.MPU9250(bus, address)
imu.begin()

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

	print ("roll: {0} ; pitch : {1} ; yaw : {2}".format(imu.roll, imu.pitch, imu.yaw))
	time.sleep(0.2)
