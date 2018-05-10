#!/usr/bin/env python

from SpinalCordBase import SpinalCordBase
import rospy

class RobotisProtocol(SpinalCordBase):
	'''
		Parent class : spinalCord.SpinalCordBase.SpinalCordBase
		Static Attribute
			PING 			register for ping.
			READ_DATA		register for read data.
			WRITE_DATA		register for write data.
			REG_WRITE		register for reg write.
			ACTION 			register for action register.
			RESET 			register for reset register.
		Method
			computeChecksum(package)
			writeNBytes(id_, paramN[, timeout])
			writeREGNBytes(id_, paramN[, timeout])
			pingMotor(id_[, timeout])
			readRegister(id_, reg, numData[, timeout])
		Class Method
			broadcastingAction()
			pingMotorID(id_[, timeout])
	'''
	PING		= 0x01
	READ_DATA	= 0x02
	WRITE_DATA	= 0x03
	REG_WRITE	= 0x04
	ACTION		= 0x05
	RESET		= 0x06

	def __init__(self, serial):
		'''
			Initial function
			Parameters
				serial		serial.Serial instance.
		'''
		super(RobotisProtocol, self).__init__(serial)

	def createPackage(self, package):
		## create robotis package.
		package.insert(3, len(package)-2)
		package.append( self.computeChecksum(package) )
		return package

	@staticmethod
	def computeChecksum(package):
		## compute robotis checksum
		chksum = 0
		for i in range(2, len(package)):
			chksum += package[i]
		chksum %= 256
		return (255 - chksum)%256

	def writeNBytes(self, id_, paramN, timeout = None):
		'''
				Write data with instruction WRITE_DATA to robotis device with
			id as id_ and parameters as paramN then wait for error package for
			timeout secs. if no error package arrive in time, return None
			otherwise return error package.
			Parameters
				id_			ID for device.
				paramN		Parameters for package.
				Optional
					timeout 		if timeout is None, this function wait
									until package arrive or wait forever if no 
									package arrive.
			Return
				lastest package arrive if there are any package arrive in time.
				Otherwise return None.
		'''
		package = [255, 255, id_, self.WRITE_DATA] + paramN
		return self.requestData( self.createPackage(package) , timeout)

	def writeREGNBytes(self, id_, paramN, timeout = None):
		'''
				Write data with instruction REG_WRITE to robotis device with
			id as id_ and parameters as paramN then wait for error package for
			timeout secs. if no error package arrive in time, return None
			otherwise return error package.
			Parameters
				id_			ID of device.
				paramN		Parameters for package.
				Optional
					timeout 		if timeout is None, this function wait
									until package arrive or wait forever if no 
									package arrive. Defualt: None
			Return
				lastest package arrive if there are any package arrive in time.
				Otherwise return None.
		'''
		package = [255, 255, id_, self.REG_WRITE] + paramN
		return self.requestData( self.createPackage(package), timeout)

	def broadcastingAction(self):
		'''
				Broad cast instruction ACTION to all robotis device in same bus
				Return
					True		if success
					False		if fail
		'''
		package = [255, 255, 0xFE, self.ACTION]
		return self.writeData(self.createPackage( package))

	def pingMotor(self, id_, timeout = None):
		'''
				Ping motor with id = id_ wait for error pakage util timeout.
			if timeout exceed, return None otherwise return error message.
			Parameters
				id_			ID of device.
				Optional
					timeout 		if timeout is None, this function wait
									until package arrive or wait forever if no 
									package arrive. Defualt: None
			Return
				None if fail otherwise error package.
		'''
		package = [255, 255, id_, self.PING]
		## UDONE 
		## this function is consider whether error package is belong to
		## target device or not. Need to be check before return.
		return self.requestData(self.createPackage(package), timeout)

	def pingMotorID(self, id_, timeout = None):
		'''
				Ping motor with id = id_ wait for error pakage util timeout.
			if timeout exceed, return None otherwise return error message.
			Parameters
				id_			ID of device.
				Optional
					timeout 		if timeout is None, this function wait
									until package arrive or wait forever if no 
									package arrive. Defualt: None
			Return
				None if fail otherwise error package.
		'''
		package = [255, 255, id_, self.PING]
		## UDONE 
		## this function is consider whether error package is belong to
		## target device or not. Need to be check before return.
		return self.requestData(self.createPackage(package), timeout)

	def readRegister(self, id_, reg, numData, timeout = None):
		'''
				Write data with instruction READ_DATA to robotis device with
			id as id_ and register as reg then wait for response package for
			timeout secs. if no response package arrive in time, return None
			otherwise return response package.
			Parameters
				id_			ID for device
				reg			Target register
				numData		Number of byte to be read
				Optional
					timeout 		if timeout is None, this function wait
									until package arrive or wait forever if no 
									package arrive.
			Return
				lastest package arrive if there are any package arrive in time.
				Otherwise return None.
		'''
		package = [255, 255, id_, self.READ_DATA, reg, numData]
		## UNDONE
		## This function not check whether package is belong to a target device
		## or not and it also not check whether package is belong to target 
		## register or not.
		return self.requestData( self.createPackage(package) , timeout)

class ServoProtocol( RobotisProtocol ):
	'''
		Parent class : spinalCord.RobotisProtocolBase.RobotisProtocol
		Attribute
			REG_CW_ANGLE_LIMIT_L	
			REG_CW_ANGLE_LIMIT_H	
			REG_CCW_ANGLE_LIMIT_L
			REG_CCW_ANGLE_LIMIT_H
			REG_MAX_TORQUE_L	
			REG_MAX_TORQUE_H	

			REG_GOAL_POSITION_L	
			REG_GOAL_POSITION_H	
			REG_MOVING_SPEED_L	
			REG_MOVING_SPEED_H	
			REG_TORQUE_LIMIT_L	
			REG_TORQUE_LIMIT_H	

			REG_PRESENT_POSITION_L
			REG_PRESENT_POSITION_H
			REG_PRESENT_SPEED_L	
			REG_PRESENT_SPEED_H	
			REG_PRESENT_LOAD_L	
			REG_PRESENT_LOAD_H	
			REG_VOLTAGE			
			REG_TEMPERATURE		
		Method
			get_CW_angle_limit(id_[, timeout])
			get_CCW_angle_limit(id_[, timeout])
			get_max_torque(id_[, timeout])
			get_goal_position(id_[, timeout])
			set_goal_position(id_, position[,speed[,timeout]])
			set_goal_position_REGACTION(id_, position[, speed[, timeout]])
			get_moving_speed(id_[, timeout])
			set_moving_speed(id_[, timeout])
			get_torque_limit(id_[, timeout])
			get_position(id_[, timeout])
			get_speed(id_[, timeout])
			get_position_and_speed(id_[, timeout])
			get_load(id_[, timeout])
			get_voltage(id_[, timeout])
			get_temperature(id_[, timeout])
	'''

	def __init__(self, serial):
		'''
			Initial function
			Parameters
				serial		serial.Serial instance.
		'''
		super(ServoProtocol, self).__init__(serial)
		#-------------------------------------------
		# register addresses of the robotis motor
		#-------------------------------------------

		self.REG_CW_ANGLE_LIMIT_L	= 0x06
		self.REG_CW_ANGLE_LIMIT_H	= 0x07
		self.REG_CCW_ANGLE_LIMIT_L	= 0x08
		self.REG_CCW_ANGLE_LIMIT_H	= 0x09

		self.REG_MAX_TORQUE_L		= 0x0E
		self.REG_MAX_TORQUE_H		= 0x0F

		self.REG_GOAL_POSITION_L	= 0x1E
		self.REG_GOAL_POSITION_H	= 0x1F
		self.REG_MOVING_SPEED_L		= 0x20
		self.REG_MOVING_SPEED_H		= 0x21
		self.REG_TORQUE_LIMIT_L		= 0x22
		self.REG_TORQUE_LIMIT_H		= 0x23

		self.REG_PRESENT_POSITION_L	= 0x24
		self.REG_PRESENT_POSITION_H	= 0x25
		self.REG_PRESENT_SPEED_L	= 0x26
		self.REG_PRESENT_SPEED_H	= 0x27
		self.REG_PRESENT_LOAD_L		= 0x28
		self.REG_PRESENT_LOAD_H		= 0x29
		self.REG_VOLTAGE			= 0x2A
		self.REG_TEMPERATURE		= 0x2B

	def get_CW_angle_limit(self, id_, timeout = None):
		return self.readRegister(id_, self.REG_CW_ANGLE_LIMIT_L, 2, timeout)

	def get_CCW_angle_limit(self, id_, timeout = None):
		return self.readRegister(id_, self.REG_CCW_ANGLE_LIMIT_L, 2, timeout)

	def get_max_torque(self, id_, timeout = None):
		return self.readRegister(id_, self.REG_MAX_TORQUE_L, 2, timeout)

	def get_goal_position(self, id_, timeout = None):
		return self.readRegister(id_, self.REG_GOAL_POSITION_L, 2, timeout)

	def set_goal_position(self, id_, position, speed = 0, timeout = None):
		paramN = [self.REG_GOAL_POSITION_L, position&0xFF, (position&0xFF00)>>8, speed&0xFF, (speed&0xFF00)>>8]
		return self.writeNBytes(id_, paramN, timeout)

	def set_goal_position_REGACTION(self, id_, position, speed=0,timeout=None):
		paramN = [self.REG_GOAL_POSITION_L, position&0xFF, (position&0xFF00)>>8, speed&0xFF, (speed&0xFF00)>>8]
		return self.writeREGNBytes(id_, paramN, timeout)

	def get_moving_speed(self, id_, timeout = None):
		return self.readRegister(id_, self.REG_MOVING_SPEED_L, 2, timeout)

	def set_moving_speed(self, id_, data, timeout = None):
		paramN = [self.REG_MOVING_SPEED_L, data&0xFF, (data&0xFF00)>>8]
		return self.writeNBytes(id_, paramN, timeout)

	def get_torque_limit(self, id_, timeout = None):
		return self.readRegister(id_, self.REG_TORQUE_LIMIT_L, 2, timeout)

	def get_position( self, id_, timeout = None):
		return self.readRegister(id_, self.REG_PRESENT_POSITION_L, 2, timeout)

	def get_speed(self, id_, timeout = None):
		return self.readRegister(id_, self.REG_PRESENT_SPEED_L, 2, timeout)

	def get_position_and_speed(self, id_, timeout = None):
		return self.readRegister(id_, self.REG_PRESENT_POSITION_L, 4, timeout)

	def get_load(self, id_, timeout = None):
		return self.readRegister(id_, self.REG_PRESENT_LOAD_L, 2, timeout)

	def get_voltage(self, id_, timeout = None):
		return self.readRegister(id_, self.REG_VOLTAGE, 1, timeout)

	def get_temperature(self, id_, timeout = None):
		return self.readRegister(id_, self.REG_TEMPERATURE, 1, timeout)