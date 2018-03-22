#!/usr/bin/env python

from RobotisProtocolBase import ServoProtocol


import math

class ServoBase(object):
	fullSpeed = 0
	def __init__(self, serial, motorID, maxAngleSpan, maxOmegaSpan, maxAngleRegs, maxOmegaRegs, maxAngleRads, minAngleRads, timeout = None, Direction = 1):

		self.spinal_cord = ServoProtocol(serial)
		assert (type(motorID) == int) and (0 < motorID < 255)
		self.__motorID = motorID
		self.__maxAngleSpan = abs(maxAngleSpan)
		self.__maxOmegaSpan = abs(maxOmegaSpan)
		
		self.__maxAngleRegs = abs(maxAngleRegs)
		self.__maxOmegaRegs = abs(maxOmegaRegs)
		
		self.__maxAngleRads = None
		self.__minAngleRads = None
		self.setAngleLimitRads(maxAngleRads, minAngleRads)

		self.__regsPerRads = self.__maxAngleRegs / self.__maxAngleSpan
		self.__radsPerRegs = self.__maxAngleSpan / self.__maxAngleRegs
		self.__regsPerOmega = self.__maxOmegaRegs / self.__maxOmegaSpan
		self.__omegaPerRegs = self.__maxOmegaSpan / self.__maxOmegaRegs
		self.__centerRegs = self.__maxAngleRegs / 2

		self.timeout = timeout
		self.__direction = 1 if Direction >= 0 else -1

	def setAngleLimitRads(self, max_, min_):
		print "PP",max_ - min_, self.__maxAngleSpan
		assert abs(max_ - min_) <= self.__maxAngleSpan
		assert max_ != min_
		self.__maxAngleRads = max(max_, min_)
		self.__minAngleRads = min(max_, min_)

	def setTimeOut(self, value):
		self.timeout = value

	def set_inverse_direction(self):
		self.__direction = -1

	def set_normal_direction(self):
		self.__direction = 1

	def moveFromRad(self, position, speed = fullSpeed):

		position = max(self.__minAngleRads, min(position, self.__maxAngleRads))
		position = self.__direction * position
		angleRegs = self.__angle2reg(position)
		speed = self.__omega2reg(speed)
		return self.spinal_cord.set_goal_position(self.__motorID, angleRegs, speed, self.timeout)

	def moveFromRad_REGACTION(self, position, speed = fullSpeed):
		position = max(self.__minAngleRads, min(position, self.__maxAngleRads))
		position = self.__direction * position
		angleRegs = self.__angle2reg(position)
		speed = self.__omega2reg(speed)
		return self.spinal_cord.set_goal_position_REGACTION(self.__motorID, angleRegs, speed, self.timeout)

	def broadcastingAction(self):
		self.spinal_cord.broadcastingAction()

	def get_position_reg(self):
		position = self.spinal_cord.get_position(self.__motorID, self.timeout)
		if position is not None and len(position) >= 7:
			print position[6]<<8, position[5]
			angleReg = position[5] + (position[6]<<8)
		else:
			angleReg = None
		return angleReg

	def get_position_rad(self):
		angleReg = self.get_position_reg()
		print angleReg
		if angleReg is None:
			return
		else:
			return self.__reg2angle(angleReg)

	def get_speed_reg(self):
		speed = self.spinal_cord.get_speed(self.__motorID, self.timeout)
		if len(speed) >= 7:
			speedReg = speed[5] + speed[6]<<8
		else:
			speedReg = None
		return speedReg

	def get_speed_rad(self):
		speedReg = self.get_position_reg()
		if speedReg is None:
			return
		else:
			return self.__reg2omega(speedReg)

	def ping_this_motor(self):
		return self.spinal_cord.pingMotor(self.__motorID, self.timeout)

	def clampAngleRad(self, value):
		return min(self.__maxAngleRads, max(self.__minAngleRads, value))

	def clampAngleReg(self, value):
		return min(self.__maxAngleRegs, max(0, value))

	def clampOmegaReg(self, value):
		return min(self.__maxOmegaRegs, max(0, value))

	def __angle2reg(self, angle):
		reg = int( self.__centerRegs + (angle * self.__regsPerRads) )
		return max(0, min(self.__maxAngleRegs, reg))

	def __reg2angle(self, regs):
		angles = (regs - self.__centerRegs) * self.__radsPerRegs 
		return angles

	def __omega2reg(self, omega):
		reg = int(omega * self.__regsPerOmega)
		return min(self.__maxOmegaRegs, max(0, reg))

	def __reg2omega(self, regs):
		omega = int(regs * self.__omegaPerRegs)
		return omega

	def __str__(self):
		return "ServoMotor"

	def __repr__(self):
		return self.__str__()

class AX12A(ServoBase):
	"""docstring for AX12A"""
	def __init__(self, serial, motorID, **kwarg):

		# maximum angle span
		MAX_ANGLE_REG_VALUE = 1023
		MAX_ANGLE_SPAN_RADS = math.radians( 300 )

		# maximum omega span
		#	according to the datasheet, AX12A support speed up to 114.0 rpm.
		#	however, the maximum controllable speed under the supply voltage 12V is about 59.0 rpm.
		MAX_OMEGA_REG_VALUE = 1023 
		MAX_OMEGA_SPAN_RADSPS = math.radians( 114./60.*360. )
		MAX_CONTROLABLE_OMEGA_RADSPS = math.radians( 59.0/60.*360. )

		MAX_ANGLE_RAD = kwarg.get("maxAngleRads", MAX_ANGLE_SPAN_RADS / 2)
		MIN_ANGLE_RAD = kwarg.get("minAngleRads", -MAX_ANGLE_SPAN_RADS / 2)
		timeout = kwarg.get("timeout", None)
		direction = kwarg.get("Direction", 1)

		super(AX12A, self).__init__(serial,
									motorID,
									MAX_ANGLE_SPAN_RADS,
									MAX_OMEGA_SPAN_RADSPS,
									MAX_ANGLE_REG_VALUE,
									MAX_OMEGA_REG_VALUE,
									MAX_ANGLE_RAD,
									MIN_ANGLE_RAD,
									timeout,
									direction
									)
	def __str__(self):
		return "ax12a"

	def __repr__(self):
		return self.__str__()

class MX28(ServoBase):
	"""docstring for AX12A"""
	def __init__(self, serial, motorID, **kwarg):

		# maximum angle span
		MAX_ANGLE_REG_VALUE = 4095
		MAX_ANGLE_SPAN_RADS = math.radians( 360 )

		# maximum omega span
		#	according to the datasheet, RX28 support speed up to 117.07 rpm.
		#	however, the maximum controllable speed under the supply voltage 14.8V is about 67 rpm.
		MAX_OMEGA_REG_VALUE = 1023 
		MAX_OMEGA_SPAN_RADSPS = math.radians( 117.07/60.*360. )
		MAX_CONTROLABLE_OMEGA_RADSPS = math.radians( 67./60.*360. )

		MAX_ANGLE_RAD = kwarg.get("maxAngleRads", MAX_ANGLE_SPAN_RADS / 2)
		MIN_ANGLE_RAD = kwarg.get("minAngleRads", -MAX_ANGLE_SPAN_RADS / 2)

		timeout = kwarg.get("timeout", None)
		direction = kwarg.get("Direction", 1)

		super(MX28, self).__init__(serial,
									motorID,
									MAX_ANGLE_SPAN_RADS,
									MAX_OMEGA_SPAN_RADSPS,
									MAX_ANGLE_REG_VALUE,
									MAX_OMEGA_REG_VALUE,
									MAX_ANGLE_RAD,
									MIN_ANGLE_RAD,
									timeout,
									direction
									)
	def __str__(self):
		return "mx28"

	def __repr__(self):
		return self.__str__()

class RX28(ServoBase):
	"""docstring for AX12A"""
	def __init__(self, serial, motorID, **kwarg):

		# maximum angle span
		MAX_ANGLE_REG_VALUE = 1023
		MAX_ANGLE_SPAN_RADS = math.radians( 300 )

		# maximum omega span
		#	according to the datasheet, RX28 support speed up to 114 rpm.
		#	however, the maximum controllable speed under the supply voltage 16V is about 79.4 rpm.
		MAX_OMEGA_REG_VALUE = 1023 
		MAX_OMEGA_SPAN_RADSPS = math.radians( 114./60.*360. )
		MAX_CONTROLABLE_OMEGA_RADSPS = math.radians( 79.4/60.*360. )

		MAX_ANGLE_RAD = kwarg.get("maxAngleRads", MAX_ANGLE_SPAN_RADS / 2)
		MIN_ANGLE_RAD = kwarg.get("minAngleRads", -MAX_ANGLE_SPAN_RADS / 2)

		timeout = kwarg.get("timeout", None)
		direction = kwarg.get("Direction", 1)

		super(RX28, self).__init__(serial,
									motorID,
									MAX_ANGLE_SPAN_RADS,
									MAX_OMEGA_SPAN_RADSPS,
									MAX_ANGLE_REG_VALUE,
									MAX_OMEGA_REG_VALUE,
									MAX_ANGLE_RAD,
									MIN_ANGLE_RAD,
									timeout,
									direction
									)
	def __str__(self):
		return "rx28"

	def __repr__(self):
		return self.__str__()