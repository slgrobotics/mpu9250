import os
import time
from json import JSONEncoder
import json
import smbus2 as smbus

from easydict import EasyDict as edict
import numpy as np

def getConfigVals():

	cfg = edict()

	# declaring register values. Don't change them as long as you are damn sure.

	cfg.Address = 0x68

	cfg.AccelOut = 0x3B # getting the accelerometer data.
	cfg.ExtSensData00 = 0x49
	cfg.AccelConfig = 0x1C
	cfg.AccelRangeSelect2G = 0x00
	cfg.AccelRangeSelect4G = 0x08
	cfg.AccelRangeSelect8G = 0x10
	cfg.AccelRangeSelect16G = 0x18

	cfg.GyroConfig = 0x1B
	cfg.GyroRangeSelect250DPS = 0x00
	cfg.GyroRangeSelect500DPS = 0x08
	cfg.GyroRangeSelect1000DPS = 0x10
	cfg.GyroRangeSelect2000DPS = 0x18

	cfg.AccelConfig2 = 0x1D
	cfg.AccelLowPassFilter184 = 0x01
	cfg.AccelLowPassFilter92 = 0x02
	cfg.AccelLowPassFilter41 = 0x03
	cfg.AccelLowPassFilter20 = 0x04
	cfg.AccelLowPassFilter10 = 0x05
	cfg.AccelLowPassFilter5 = 0x06

	cfg.GyroConfig2 = 0x1A
	cfg.GyroLowPassFilter184 = 0x01
	cfg.GyroLowPassFilter92 = 0x02
	cfg.GyroLowPassFilter41 = 0x03
	cfg.GyroLowPassFilter20 = 0x04
	cfg.GyroLowPassFilter10 = 0x05
	cfg.GyroLowPassFilter5 = 0x06

	cfg.SMPDivider = 0x19
	cfg.InitPinConfig = 0x37
	cfg.InitEnable = 0x38
	cfg.InitDisable = 0x00
	cfg.InitPulse50us = 0x00
	cfg.InitWakeOnMotionEnable = 0x40
	cfg.InitRawReadyEnable = 0x01

	cfg.PowerManagement1 = 0x6B
	cfg.PowerCycle = 0x20
	cfg.PowerReset = 0x80
	cfg.ClockPLL = 0x01

	cfg.PowerManagement2 = 0x6C
	cfg.SensorEnable = 0x00
	cfg.DisableGyro = 0x07

	cfg.UserControl = 0x6A
	cfg.I2CMasterEnable = 0x20
	cfg.I2CMasterClock = 0x0D

	cfg.I2CMasterControl = 0x24
	cfg.I2CSlave0Address = 0x25
	cfg.I2CSlave0Register = 0x26
	cfg.I2CSlave0Do = 0x63
	cfg.I2CSlave0Control = 0x27
	cfg.I2CSlave0Enable = 0x80
	cfg.I2CReadFlad = 0x80

	cfg.MotionDetectControl = 0x69
	cfg.AccelIntelEnable = 0x80
	cfg.AccelIntelMode = 0x40

	cfg.LowPassAccelODR = 0x1E
	cfg.LP_ACCEL_ODR_0_24HZ = 0
	cfg.LP_ACCEL_ODR_0_49HZ = 1
	cfg.LP_ACCEL_ODR_0_98HZ = 2
	cfg.LP_ACCEL_ODR_1_95HZ = 3
	cfg.LP_ACCEL_ODR_3_91HZ = 4
	cfg.LP_ACCEL_ODR_7_81HZ = 5
	cfg.LP_ACCEL_ODR_15_63HZ = 6
	cfg.LP_ACCEL_ODR_31_25HZ = 7
	cfg.LP_ACCEL_ODR_62_50HZ = 8
	cfg.LP_ACCEL_ODR_125HZ = 9
	cfg.LP_ACCEL_ODR_250HZ = 10
	cfg.LP_ACCEL_ODR_500HZ = 11

	cfg.WakeOnMotionThreshold = 0x1F
	cfg.WhoAmI = 0x75

	# Ak8963 registers
	cfg.Ak8963I2CAddress = 0x0C
	cfg.Ak8963HXL = 0x03
	cfg.Ak8963CNTL1 = 0x0A
	cfg.Ak8963PowerDown = 0x00
	cfg.Ak8963ContinuosMeasurment1 = 0x12  # 16-bit resolution (0.15 microTesla/LSB) and Continuous Measurement Mode 1 (8 Hz)
	cfg.Ak8963ContinuosMeasurment2 = 0x16  # 16-bit resolution (0.15 microTesla/LSB) and Continuous Measurement Mode 2 (100 Hz)
	cfg.Ak8963FuseROM = 0x0F
	cfg.Ak8963CNTL2 = 0x0B
	cfg.Ak8963Reset = 0x01
	cfg.Ak8963ASA = 0x10
	cfg.Ak8963WhoAmI = 0x00

	# end of register declaration

	"""
	The MPU-9250 accelerometer and gyroscope share the same body-frame axis orientation (they are on the same die).

	The onboard magnetometer (AK8963/AK09916) is physically on a separate die, mounted with a different internal
	  orientation relative to the accel/gyro die within the chip, so its raw axes do not align with the body frame.

	As a result, the magnetometer readings must be remapped (axis swaps and sign flips) in software
	  to match the accelerometer/gyroscope frame before sensor fusion (e.g., Madgwick or EKF).

	The exact mapping is board- and implementation-dependent, but a common transformation:
	 - swaps X/Y and
	 - flips Z	
	"""

	cfg.transformationMatrixAG  = np.array([[0.0,1.0,0.0],[1.0,0.0,0.0],[0.0,0.0,-1.0]]).astype(np.int16)
	cfg.transformationMatrixMag = np.array([[0.0,1.0,0.0],[1.0,0.0,0.0],[0.0,0.0,-1.0]]).astype(np.int16)

	cfg.I2CRate = 400000
	cfg.TempScale = 333.87
	cfg.TempOffset = 21.0
	cfg.Gravity = 9.807
	cfg.Degree2Radian = np.pi/180.0

	cfg.acc_t_matrix_ned = np.array([[0.0,-1.0,0.0],[-1.0,0.0,0.0],[0.0,0.0,1.0]]) #np.array([-1.0,1.0,1.0])
	cfg.gyro_t_matrix_ned = np.array([[0.0,1.0,0.0],[1.0,0.0,0.0],[0.0,0.0,-1.0]]) #np.array([1.0,-1.0,-1.0])
	cfg.mag_t_matrix_ned = np.array([[0.0,-1.0,0.0],[1.0,0.0,0.0],[0.0,0.0,1.0]])

	return cfg

class MPU9250:
	"""
	An interface between MPU9250 and rpi using I2C protocol

	It has various fuctions from calibration to computing orientation

	"""

	def __init__(self, bus, address):
		"""
		Sets up the basic variables like scale and bias of sensors.

		"""

		self.cfg = getConfigVals()
		self.cfg.Address = address
		self.Bus = bus
		self.AccelBias = np.array([0.0, 0.0, 0.0])
		self.Accels = np.array([1.0, 1.0, 1.0])
		self.MagBias = np.array([0.0, 0.0, 0.0])   # magnetometer bias derived during calibration
		self.Mags = np.array([1.0, 1.0, 1.0])      # optional magnetometer scale adjustment
		self.GyroBias = np.array([0.0, 0.0, 0.0])  # will be set in begin() after short gyro calibration
		self.Magtransform = None  # magnetometer calibration is unknown

	def begin(self):
		"""
		Initializes various registers of MPU9250.

		It also sets ranges of accelerometer and gyroscope and also the frequency of low 
		pass filter.

		"""

		self.__writeRegister(self.cfg.PowerManagement1, self.cfg.ClockPLL)
		self.__writeRegister(self.cfg.UserControl, self.cfg.I2CMasterEnable)
		self.__writeRegister(self.cfg.I2CMasterControl, self.cfg.I2CMasterClock)

		self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963PowerDown)
		#self.__writeRegister(self.cfg.PowerManagement1, self.cfg.PowerReset) # power is not reseting
		time.sleep(0.01)
		#self.__writeAK8963Register(self.cfg.Ak8963CNTL2, self.cfg.Ak8963Reset) # AK8963 is not resetting
		self.__writeRegister(self.cfg.PowerManagement1, self.cfg.ClockPLL)

		name = self.__whoAmI()
		if not (name[0] == 113 or name[0] == 115 ):
			print ("The name is wrong {0}".format(name))
		self.__writeRegister(self.cfg.PowerManagement2, self.cfg.SensorEnable)

		self.setAccelRange("AccelRangeSelect2G")

		self.setGyroRange("GyroRangeSelect250DPS")  # 250 degrees per second range for ground robots

		self.setLowPassFilterFrequency("AccelLowPassFilter184")

		# with DLPF enabled, the gyro output rate is 1 kHz

		self.__writeRegister(self.cfg.SMPDivider, 0x00)
		self.CurrentSRD = 0x00
		self.setSRD(self.CurrentSRD) # set Acc rate also to 1 kHz, 100 Hz for Mag

		self.__writeRegister(self.cfg.UserControl, self.cfg.I2CMasterEnable)
		self.__writeRegister(self.cfg.I2CMasterControl, self.cfg.I2CMasterClock)

		magName = self.__whoAmIAK8963() # mag name seems to be different
		if magName[0] != 72:
			print ("The mag name is different and it is {0}".format(magName))

		self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963FuseROM)
		time.sleep(0.1)
		self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963ContinuosMeasurment2)  # 100 Hz mag rate
		time.sleep(0.1)

		# Accessing the mag's Factory Calibration (Fuse ROM):
		self.MagScale = self.__readAK8963Registers(self.cfg.Ak8963ASA, 3)
		self.MagScale = np.array(self.MagScale)
		# This "MagScale" official formula converts the register "bits" to Tesla:
		self.MagScale = (((self.MagScale - 128.0)/256.0) + 1.0)*0.15/1000000.0  # approx 0.15*e-6 Tesla per LSB, on 3 axes

		#print(f"Magnetometer Sensitivity Scales: {self.MagScale}  LSB/Tesla")  # MagScale: [1.79882812e-07 1.79882812e-07 1.73437500e-07]

		self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963PowerDown)
		time.sleep(0.1)
		self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963ContinuosMeasurment2)  # 100 Hz
		time.sleep(0.1)

		self.__writeRegister(self.cfg.PowerManagement1, self.cfg.ClockPLL)
		self.__readAK8963Registers(self.cfg.Ak8963HXL, 7)

		# Calibrating Gyro 
		self.calibrateGyro()

		return 1

	def setSRD(self, data):
		"""
		Sets the frequency of getting data from Accelerometer, matches it for magnetometer

		Parameters
		----------
		data : int
			This number is between 1 to 19 and decides the Acc rate of sample collection relative to Gyro rate:

		AccOutputRate = GyroOutputRate / (1 + data)

		with DLPF enabled, the gyro output rate is 1 kHz

		Acc/ Gyro "Sample Rate Divider":
		Desired internal rate	SMPLRT_DIV (With DLPF enabled)
		    1 kHz	0
		  250 Hz	3
		  200 Hz	4
		  100 Hz	9
		   50 Hz	19
		"""

		# temporary "slow mode" switch while reconfiguring the AK8963 rate
		self.__writeRegister(self.cfg.SMPDivider, 19)  # forces ~50 Hz temporarily (with DLPF)

		if data > 9:
			# for lower Acc/Gyro rate, set mag to 8 Hz:
			self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963PowerDown)
			time.sleep(0.1)
			self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963ContinuosMeasurment1)  # 8 Hz Mag rate
			time.sleep(0.1)
			self.__readAK8963Registers(self.cfg.Ak8963HXL, 7)
		else:
			# for higher Acc/Gyro rate, set mag to 100 Hz:
			self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963PowerDown)
			time.sleep(0.1)
			self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963ContinuosMeasurment2)  # 100 Hz Mag rate
			time.sleep(0.1)
			self.__readAK8963Registers(self.cfg.Ak8963HXL, 7)

		self.CurrentSRD = data
		self.__writeRegister(self.cfg.SMPDivider, data)  # set the desired Acc rate

	def setAccelRange(self, accelRange):
		"""Sets the range of accelerometer

		Parameters
		----------
		accelRange : str
			The supported ranges are as following ->
			2g  -> AccelRangeSelect2G
			4g  -> AccelRangeSelect4G
			8g  -> AccelRangeSelect8G
			16g -> AccelRangeSelect16G

		"""
		
		try:
			self.__writeRegister(self.cfg.AccelConfig, self.cfg[accelRange])
			self.AccelRange = accelRange
		except:
			raise RuntimeError(f"{accelRange} is not a proper value for accelerometer range")

		accelVal = float(accelRange.split('t')[1].split('G')[0])
		self.AccelScale = self.cfg.Gravity*accelVal/32767.5
		return 1

	def setGyroRange(self, gyroRange):
		"""Sets the range of gyroscope

		Parameters
		----------
		gyroRange : str
			The supported ranges are as following ->
			250DPS  -> GyroRangeSelect250DPS
			500DPS  -> GyroRangeSelect500DPS
			1000DPS -> GyroRangeSelect1000DPS
			2000DPS -> GyroRangeSelect2000DPS

			DPS means degrees per freedom

		"""

		try:
			self.__writeRegister(self.cfg.GyroConfig, self.cfg[gyroRange])
			self.GyroRange = gyroRange
		except:
			raise RuntimeError(f"{gyroRange} is not a proper value for gyroscope range")

		gyroVal = float(gyroRange.split('t')[1].split('D')[0])
		self.GyroScale = self.cfg.Degree2Radian*(gyroVal/32767.5)
		return 1

	def setLowPassFilterFrequency(self, frequency):
		"""Sets the frequency of internal low pass filter

		This is common for both accelerometer and gyroscope

		Parameters
		----------
		frequency : str
			The supported frequencies are as following ->
			250DPS  -> GyroRangeSelect250DPS
			500DPS  -> GyroRangeSelect500DPS
			1000DPS -> GyroRangeSelect1000DPS
			2000DPS -> GyroRangeSelect2000DPS

			DPS means degrees per freedom

		"""

		try:
			self.__writeRegister(self.cfg.AccelConfig2, self.cfg[frequency])
			self.__writeRegister(self.cfg.GyroConfig2, self.cfg[frequency])
			self.Frequency = frequency
		except:
			raise RuntimeError(f"{frequency} is not a proper value for low pass filter")

		return 1

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

	def readSensor(self):
		"""Read accel/gyro/mag + apply calibration + optional transforms."""

		# Read 21 bytes; last byte is usually status/unused -> drop it
		data = np.asarray(self.__readRegisters(self.cfg.AccelOut, 21)[:-1], dtype=np.uint8)

		# Interpret as signed 16-bit big-endian pairs
		# data layout: [H,L,H,L,...]
		vals_u16 = (data[0::2].astype(np.uint16) << 8) | data[1::2].astype(np.uint16)
		vals = vals_u16.view(np.int16)
		# Extract raw vectors (still int16, signed, Big Endian) - head of the stream
		a_raw = vals[0:3]
		temp_raw = vals[3]
		g_raw = vals[4:7]

		# Mag bytes (signed, Little Endian) are in the tail of the original byte stream (offset 14)
		mag_bytes = data[14:]
		mag_u16 = (mag_bytes[1::2].astype(np.uint16) << 8) | mag_bytes[0::2].astype(np.uint16)
		magvals = mag_u16.view(np.int16)

		m_raw = magvals[0:3]

		# ---- Calibrate in sensor frame (scale then bias) ----
		# Assumption: Bias arrays are in scaled units (same units as raw*scale)
		a_cal = (a_raw.astype(np.float64) * self.AccelScale - self.AccelBias) * self.Accels
		g_cal = (g_raw.astype(np.float64) * self.GyroScale - self.GyroBias)  # GyroBias is calibrated in begin()
		# Calibration was done on original (x,y,z) readings and don't assume any further frame rotations.
		m_cal = (m_raw.astype(np.float64) * self.MagScale  - self.MagBias) * self.Mags  # converted to Tesla and adjusted (before rotation)

		m_cal = self.rotate_mag(m_cal)  # rotate to align with accel/gyro frame

		# ---- Apply hardcoded axis transform to accel/gyro ----
		T = self.cfg.transformationMatrixAG    # expected shape (3,3)
		Tm = self.cfg.transformationMatrixMag  # expected shape (3,3)
		# This is faster/cleaner than dot + squeeze + transpose
		self.AccelVals = T @ a_cal
		self.GyroVals  = T @ g_cal

		# ---- Mag optional transform from "magnetometer_transform" parameter ----
		m_out = Tm @ m_cal
		if self.Magtransform is None:
			self.MagVals = m_out
		else:
			# Magtransform is 3x3, do matrix multiply - values will be corrected for soft-iron distortion
			self.MagVals = self.Magtransform @ m_out

		self.Temp = (temp_raw / self.cfg.TempScale) + self.cfg.TempOffset


	def calibrateGyro(self):
		"""Calibrates gyroscope by finding the bias sets the gyro bias"""

		currentGyroRange = self.GyroRange
		currentFrequency = self.Frequency
		currentSRD = self.CurrentSRD
		self.setGyroRange("GyroRangeSelect250DPS")
		self.setLowPassFilterFrequency("AccelLowPassFilter20")
		self.setSRD(19)  # forces acc data rate to ~50 Hz temporarily (with DLPF)

		gyroBias1 = np.array([0.0,0.0,0.0])
		for i in range(100):
			self.readSensor()
			gyroBias1 += self.GyroVals
			time.sleep(0.02)

		self.GyroBias = gyroBias1/100.0

		self.setGyroRange(currentGyroRange)
		self.setLowPassFilterFrequency(currentFrequency)
		self.setSRD(currentSRD)

	def calibrateAccelerometer(self):
		"""Calibrate Accelerometer by positioning it in 6 different positions
		
		This function expects the user to keep the imu in 6 different positions while calibration. 
		It gives cues on when to change the position. It is expected that in all the 6 positions, 
		at least one axis of IMU is parallel to gravity of earth and no position is same. Hence we 
		get 6 positions namely -> +x, -x, +y, -y, +z, -z.
		"""

		currentAccelRange = self.AccelRange
		currentFrequency = self.Frequency
		currentSRD = self.CurrentSRD
		self.setAccelRange("AccelRangeSelect2G")
		self.setLowPassFilterFrequency("AccelLowPassFilter20")
		self.setSRD(19)  # forces acc data rate to ~50 Hz temporarily (with DLPF)

		xbias = []
		ybias = []
		zbias = []
		xscale = []
		yscale = []
		zscale = []

		print ("Acceleration calibration is starting and keep placing the IMU in 6 different directions based on the instructions below")
		time.sleep(2)
		for i in range(6):
			input("Put the IMU in {0} position. Press enter to continue..".format(i+1))
			time.sleep(3)
			meanvals = self.__getAccelVals()
			print (meanvals)
			xscale, xbias = self.__assignBiasOrScale(meanvals[0], xscale, xbias)
			yscale, ybias = self.__assignBiasOrScale(meanvals[1], yscale, ybias)
			zscale, zbias = self.__assignBiasOrScale(meanvals[2], zscale, zbias)
			print (xscale)
			print (yscale)
			print (zscale)

		if len(xscale) != 2 or len(yscale) != 2 or len(zscale) != 2:
			print ("It looks like there were some external forces on sensor and couldn't get proper values. Please try again")
			return


		self.AccelBias[0] = -1*(xscale[0] + xscale[1])/(abs(xscale[0]) + abs(xscale[1]))
		self.AccelBias[1] = -1*(yscale[0] + yscale[1])/(abs(yscale[0]) + abs(yscale[1]))
		self.AccelBias[2] = -1*(zscale[0] + zscale[1])/(abs(zscale[0]) + abs(zscale[1]))

		self.AccelBias = -1*self.cfg.Gravity*self.AccelBias

		self.Accels[0] = (2.0*self.cfg.Gravity)/(abs(xscale[0]) + abs(xscale[1]))
		self.Accels[1] = (2.0*self.cfg.Gravity)/(abs(yscale[0]) + abs(yscale[1]))
		self.Accels[2] = (2.0*self.cfg.Gravity)/(abs(zscale[0]) + abs(zscale[1]))
		
		self.setAccelRange(currentAccelRange)
		self.setLowPassFilterFrequency(currentFrequency)
		self.setSRD(currentSRD)

	def __getScale(self, scale):
		if len(scale) == 0:
			return 1
		else:
			return sum(scale)/(2*self.cfg.Gravity)

	def __assignBiasOrScale(self, val, scale, bias):

		if val > 6.0 or val < -6.0 :
			scale.append(val)
		else:
			bias.append(val)
		return scale, bias


	def __getAccelVals(self):

		accelvals = np.zeros((100,3))
		for samples in range(1,100):
			self.readSensor()
			vals = self.AccelVals/self.Accels + self.AccelBias
			accelvals[samples] = vals
			time.sleep(0.02)
		meanvals = np.array([accelvals[:,0].mean(), accelvals[:,1].mean(), accelvals[:,2].mean()])
		return meanvals

	def calibrateMagApprox(self):
		"""Calibrate Magnetometer
		
		This function uses basic methods like averaging and scaling to find the hard iron
		and soft iron effects.

		Note: Make sure you rotate the sensor in 8 shape and cover all the 
		pitch and roll angles.

		"""

		currentSRD = self.CurrentSRD
		self.setSRD(19)  # forces acc data rate to ~50 Hz temporarily (with DLPF)
		numSamples = 1000
		magvals = np.zeros((numSamples,3))
		for sample in range(1,numSamples):
			self.readSensor()
			magvals[sample] = self.MagVals/self.Mags + self.MagBias
			time.sleep(0.02)
			if sample % 10 == 0:
				print(f"Calibration progress: {sample}/{numSamples} samples", end='\r', flush=True)
		minvals = np.array([magvals[:,0].min(), magvals[:,1].min(), magvals[:,2].min()])
		maxvals = np.array([magvals[:,0].max(), magvals[:,1].max(), magvals[:,2].max()])

		self.MagBias = (minvals + maxvals)/2.0
		self.MagBias[2] = -self.MagBias[2]  # change in z bias
		averageRad = (((maxvals - minvals)/2.0).sum())/3.0
		self.Mags = ((maxvals - minvals)/2.0)*(1/averageRad)

		self.setSRD(currentSRD)

	def calibrateMagPrecise(self):
		"""Calibrate Magnetometer Use this method for more precise calculation
		
		This function uses ellipsoid fitting to get an estimate of the bias and
		transformation matrix required for mag data

		Note: Make sure you rotate the sensor in 8 shape and cover all the 
		pitch and roll angles.

		"""

		currentSRD = self.CurrentSRD
		self.setSRD(19)  # forces acc data rate to ~50 Hz temporarily (with DLPF)
		numSamples = 1000
		magvals = np.zeros((numSamples,3))
		for sample in range(1,numSamples):
			self.readSensor()
			magvals[sample] = self.MagVals/self.Mags + self.MagBias
			time.sleep(0.05)
			if sample % 10 == 0:
				print(f"Calibration progress: {sample}/{numSamples} samples", end='\r', flush=True)
		centre, evecs, radii, v = self.__ellipsoid_fit(magvals)

		a, b, c = radii
		r = (a * b * c) ** (1. / 3.)
		D = np.array([[r/a, 0., 0.], [0., r/b, 0.], [0., 0., r/c]])
		transformation = evecs.dot(D).dot(evecs.T)

		self.MagBias = centre
		self.MagBias[2] = -self.MagBias[2]  # change in z bias
		self.Magtransform = transformation

		self.setSRD(currentSRD)

	def __ellipsoid_fit(self, X):
		x = X[:, 0]
		y = X[:, 1]
		z = X[:, 2]
		D = np.array([x * x + y * y - 2 * z * z,
					x * x + z * z - 2 * y * y,
					2 * x * y,
					2 * x * z,
					2 * y * z,
					2 * x,
					2 * y,
					2 * z,
					1 - 0 * x])
		d2 = np.array(x * x + y * y + z * z).T # rhs for LLSQ
		u = np.linalg.solve(D.dot(D.T), D.dot(d2))
		a = np.array([u[0] + 1 * u[1] - 1])
		b = np.array([u[0] - 2 * u[1] - 1])
		c = np.array([u[1] - 2 * u[0] - 1])
		v = np.concatenate([a, b, c, u[2:]], axis=0).flatten()
		A = np.array([[v[0], v[3], v[4], v[6]],
					[v[3], v[1], v[5], v[7]],
					[v[4], v[5], v[2], v[8]],
					[v[6], v[7], v[8], v[9]]])

		center = np.linalg.solve(- A[:3, :3], v[6:9])

		translation_matrix = np.eye(4)
		translation_matrix[3, :3] = center.T

		R = translation_matrix.dot(A).dot(translation_matrix.T)

		evals, evecs = np.linalg.eig(R[:3, :3] / -R[3, 3])
		evecs = evecs.T

		radii = np.sqrt(1. / np.abs(evals))
		radii *= np.sign(evals)

		return center, evecs, radii, v

	def saveCalibDataToFile(self, filePath):
		""" Save the calibration values

		Parameters
		----------
		filePath : str
			Make sure the folder exists before giving the input.  The path 
			has to be absolute.
			Otherwise it doesn't save the values.

		"""

		calibVals = {}
		calibVals['Accels'] = self.Accels
		calibVals['AccelBias'] = self.AccelBias
		calibVals['GyroBias'] = self.GyroBias
		calibVals['Mags'] = self.Mags
		calibVals['MagBias'] = self.MagBias
		if self.Magtransform is not None:
			calibVals['Magtransform'] = self.Magtransform

		# check if folder of the path exists
		dirName = os.path.dirname(filePath)
		if not os.path.isdir(dirName):
			print ("Please provide a valid folder")
			return
		basename = os.path.basename(filePath)
		if basename.split('.')[-1] != 'json':
			print ("Please provide a json file")
			return

		with open(filePath, 'w') as outFile:
			json.dump(calibVals, outFile, cls =NumpyArrayEncoder)

	def loadCalibDataFromFile(self, filePath):
		""" Save the calibration values

		Parameters
		----------
		filePath : str
			Make sure the file exists before giving the input. The path 
			has to be absolute.
			Otherwise it doesn't save the values.
		
		"""

		#check if file path exists
		if not os.path.exists(filePath):
			print ("Please provide the correct path")

		with open(filePath, 'r') as jsonFile:
			calibVals = json.load(jsonFile)
			self.Accels = np.asarray(calibVals['Accels'])
			self.AccelBias = np.asarray(calibVals['AccelBias'])
			self.GyroBias = np.asarray(calibVals['GyroBias'])
			self.Mags = np.asarray(calibVals['Mags'])
			self.MagBias = np.asarray(calibVals['MagBias'])
			if 'Magtransform' in calibVals.keys():
				self.Magtransform = np.asarray(calibVals['Magtransform'])

	def computeOrientation(self):
		""" Computes roll, pitch and yaw

		The function uses accelerometer and magnetometer values
		to estimate roll, pitch and yaw. These values could be 
		having some noise, hence look at madgwick filter
		in filters folder to get a better estimate.
		
		"""

		self.roll = np.arctan2(self.AccelVals[1], self.AccelVals[2] + 0.05*self.AccelVals[0])
		self.pitch = np.arctan2(-1*self.AccelVals[0], np.sqrt(np.square(self.AccelVals[1]) + np.square(self.AccelVals[2]) ))
		magLength = np.sqrt(np.square(self.MagVals).sum())
		normMagVals = self.MagVals/magLength
		self.yaw = np.arctan2(np.sin(self.roll)*normMagVals[2] - np.cos(self.roll)*normMagVals[1],\
					np.cos(self.pitch)*normMagVals[0] + np.sin(self.roll)*np.sin(self.pitch)*normMagVals[1] \
					+ np.cos(self.roll)*np.sin(self.pitch)*normMagVals[2])

		self.roll = np.degrees(self.roll)
		self.pitch = np.degrees(self.pitch)
		self.yaw = np.degrees(self.yaw)

	def __writeRegister(self, subaddress, data):

		self.Bus.write_byte_data(self.cfg.Address, subaddress, data)
		time.sleep(0.01)  # small delay to avoid errors in __writeAK8963Register() 

		# Optionally, verify the write:
		val = self.__readRegisters(subaddress,1)
		if val[0] != data:
			print (f"Error: __writeRegister(): did not write the val=0x{data:X} to register=0x{subaddress:X}")
			return -1
		return 1

	def __readRegisters(self, subaddress, count):

		data = self.Bus.read_i2c_block_data(self.cfg.Address, subaddress, count)
		return data

	def __writeAK8963Register(self, subaddress, data):

		self.__writeRegister(self.cfg.I2CSlave0Address, self.cfg.Ak8963I2CAddress)
		self.__writeRegister(self.cfg.I2CSlave0Register, subaddress)
		self.__writeRegister(self.cfg.I2CSlave0Do, data)
		self.__writeRegister(self.cfg.I2CSlave0Control, self.cfg.I2CSlave0Enable | 1)

		val = self.__readAK8963Registers(subaddress, 1)

		if val[0] != data:
			print (f"Error: __writeAK8963Register(): looks like it did not write val=0x{data:X} to subaddress=0x{subaddress:X} properly")
		return 1

	def __readAK8963Registers(self, subaddress, count):

		self.__writeRegister(self.cfg.I2CSlave0Address, self.cfg.Ak8963I2CAddress | self.cfg.I2CReadFlad)
		self.__writeRegister(self.cfg.I2CSlave0Register, subaddress)
		self.__writeRegister(self.cfg.I2CSlave0Control, self.cfg.I2CSlave0Enable | count)

		time.sleep(0.01)
		data = self.__readRegisters(self.cfg.ExtSensData00, count)
		return data

	def __whoAmI(self):

		data = self.__readRegisters(self.cfg.WhoAmI, 1)
		return data

	def __whoAmIAK8963(self):

		data = self.__readAK8963Registers(self.cfg.Ak8963WhoAmI, 1)
		return data

	@property
	def roll(self):
		return self._roll

	@roll.setter
	def roll(self, roll):
		self._roll = roll

	@property
	def pitch(self):
		return self._pitch

	@pitch.setter
	def pitch(self, pitch):
		self._pitch = pitch

	@property
	def yaw(self):
		return self._yaw

	@yaw.setter
	def yaw(self, yaw):
		self._yaw = yaw

	@property
	def Bus(self):
		return self._Bus

	@Bus.setter
	def Bus(self, Bus):
		if isinstance(Bus, smbus.SMBus):
			self._Bus = Bus
		else:
			raise Exception("Please provide the object created by smbus")


class NumpyArrayEncoder(JSONEncoder):
	def default(self, obj):
		if isinstance(obj, np.ndarray):
			return obj.tolist()
		return JSONEncoder.default(self, obj)