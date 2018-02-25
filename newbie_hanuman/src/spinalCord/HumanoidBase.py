#!/usr/bin/env python

from RobotisProtocolBase import RobotisProtocol
import rospy

RegisterNames = [
					'REG_FIRMWARE_VERSION_L', 
					'REG_FIRMWARE_VERSION_H', 
					'REG_LOCOMOTION_COMMAND',
					'reserved', 
					'REG_LOCOMOTION_STATUS_L', 
					'REG_LOCOMOTION_STATUS_H', 
					'REG_COMPASS_L', 
					'REG_COMPASS_H', 
					'REG_GYRO_ROT_X_L', 
					'REG_GYRO_ROT_X_H', 
					'REG_GYRO_ROT_Y_L', 
					'REG_GYRO_ROT_Y_H', 
					'REG_GYRO_ROT_Z_L', 
					'REG_GYRO_ROT_Z_H', 
					'REG_TILT_STATUS',
					'REG_RELOAD_REGISTER',
					'REG_ROBOT_LINEAR_VELOCITY_X',
					'REG_ROBOT_LINEAR_VELOCITY_Y',
					'REG_ROBOT_ANGULAR_VELOCITY', 
					'REG_MAGNETO_X_L',
					'REG_MAGNETO_X_H',
					'REG_MAGNETO_Y_L',
					'REG_MAGNETO_Y_H',
					'REG_MAGNETO_Z_L',
					'REG_MAGNETO_Z_H',
				]

RegisterAddrDict = {}
for address in range( 0, len( RegisterNames ) ):
	RegisterAddrDict[ RegisterNames[address] ] = address
	
# Locomotion Controller ID
LocomotionControllerID = 1

# constant mask/value for locomotion status
TiltStatusMask = 0x0003
TiltStatus_Normal = 0
TiltStatus_FallDown_FaceDown = 1
TiltStatus_FallDown_FaceUp = 2

# command buttons mask for locomotion status
CommandButtonsMask = 0xFF00
CommandButtonsShiftBits = 8
CommandButtonsStartBitMask = 0x4000
CommandButtonsStopBitMask = 0x8000

CommandButtonsModeSelectorBitMask = 0x0F00
CommandButtonsModeSelectorShiftBits = 8

class HanumanProtocol(RobotisProtocol):

	GYRO_X = 0
	GYRO_Y = 1
	GYRO_Z = 2

	def __init__(self, serial):
		super(HanumanProtocol, self).__init__(serial)
		self.__robotStatus = None

	def read_AllLowLevelData(self, timeout = None):
		numB = len(RegisterNames)
		robotStatus = self.readRegister(
								LocomotionControllerID, 
								RegisterAddrDict["REG_FIRMWARE_VERSION_L"], 
								numB,
								timeout = timeout)
		# print "robotStatus", robotStatus
		self.__robotStatus = self.__get_robotStatusData(robotStatus)

	def get_locomotion_command(self, timeout = None):
		return self.readRegister(	LocomotionControllerID, 
									RegisterAddrDict["REG_LOCOMOTION_COMMAND"],
									1,
									timeout = timeout)

	def set_locomotion_command(self, command, timeout = None):
		paramN = [RegisterAddrDict["REG_LOCOMOTION_COMMAND"], command]
		return self.writeNBytes(LocomotionControllerID, 
								paramN, 
								timeout = timeout)

	def set_locomotion_status_H(self, value, timeout = None):
		paramN = [RegisterAddrDict["REG_LOCOMOTION_STATUS_H"], value]
		return self.writeNBytes(LocomotionControllerID, 
								paramN,
								timeout = timeout)

	def get_locomotion_status(self, timeout = None):
		return self.readRegister(LocomotionControllerID,
								RegisterAddrDict["REG_LOCOMOTION_STATUS_L"],
								2,
								timeout = timeout)

	def get_compass_value(self, timeout = None):
		return self.readRegister(LocomotionControllerID,
								RegisterAddrDict["REG_COMPASS_L"], 
								2,
								timeout = timeout)

	def get_gyro_value(self, axis, timeout = None):
		register = None
		if axis == self.GYRO_X:
			register = RegisterAddrDict["REG_GYRO_ROT_X_L"]
		elif axis == self.GYRO_Y:
			register = RegisterAddrDict["REG_GYRO_ROT_Y_L"]
		elif axis == self.GYRO_Z:
			register = RegisterAddrDict["REG_GYRO_ROT_Z_L"]
		else:
			rospy.logwarn("HanumanProtocol.get_gyro_value() : Unknown axis.")
			return None

		return self.readRegister(LocomotionControllerID,
								register,
								2,
								timeout = timeout)

	## UNDONE : Should change package into something's easily used by user.
	#			Such as, list of [gx, gy, gz].
	def get_all_gyro_value(self, timeout = None):
		return self.readRegister(LocomotionControllerID,
								RegisterAddrDict["REG_GYRO_ROT_X_L"],
								6,
								timeout = timeout)

	## UNDONE : Shuld change package into something's easily used by user.
	#			Such as, list of [vx, vy, omega].
	def get_locomotion_velocity(self, timeout = None):
		return self.readRegister(LocomotionControllerID,
							RegisterAddrDict["REG_ROBOT_LINEAR_VELOCITY_X"],
							3,
							timeout = timeout)

	def set_locomotion_velocity(self, velX, velY, omgZ,timeout = None):
		paramN = [	RegisterAddrDict["REG_ROBOT_LINEAR_VELOCITY_X"],
					velX,
					velY,
					omgZ]
		return self.writeNBytes(LocomotionControllerID, 
								paramN,
								timeout = timeout)

	## UNDONE : Shuld change package into something's easily used by user.
	#			Such as, list of [mx, my, mz].
	def get_all_magneto_values(self, timeout = None):
		return self.readRegister(LocomotionControllerID,
								RegisterAddrDict["REG_MAGNETO_X_L"],
								6,
								timeout = timeout)

	def get_locomotionStatus(self):
		index = RegisterAddrDict["REG_LOCOMOTION_STATUS_L"]
		# print "robotStatus",self.__robotStatus
		if self.__robotStatus is None:
			return
		elif len(self.__robotStatus) != len(RegisterNames):
			return
		return self.__robotStatus[index] | (self.__robotStatus[index+1]<<8) 

	def __get_robotStatusData(self, robotStatus):
		nB = len(RegisterNames)
		if robotStatus is None:
			return
		if len(robotStatus) >= 6+nB:
			robotStatus = robotStatus[5:-1]
			return robotStatus
		else:
			return None