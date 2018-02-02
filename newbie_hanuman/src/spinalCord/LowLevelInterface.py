#!/usr/bin/env python
from HumanoidBase import HanumanProtocol
from Queue import PriorityQueue

import time

SpecialPostureTable =	{
				'ReadyToMove'		:	'2', 
				'StandStill'		:	'n',
				'LeftKick'			:	'j', 
				'RightKick'			:	'k', 
				'Standup'			:	'h', 
				'TouchBall'			:	'y',
				'LeftTouch'			:	'z', 
				'RightTouch'		:	'x',
				'LeftSideKick'		:	'9', 
				'RightSideKick'		:	'0',
				'Happy'				:	'a', 
				'Upset'				:	'b', 
				
				'ContinuousWalk' 	:	'o', 
				
				#
				# Goalie commands (do not use with ordinary robot)
				#
				'GoalieSaveRight'	:	'f',
				'GoalieSaveLeft'	:	'g',
				
				#
				# The following commands are temporary used, 
				#	till the omni-walk firmware can handle 
				#	the side direction movement properly
				#
				'OldStyleLeftSlide'			:	'd', 
				'OldStyleRightSlide'		:	'c', 
				'OldStyleLeftCurveSlide'	:	's', 
				'OldStyleRightCurveSlide'	:	'r',
				
				#
				# we use this when vx & vy == 0, but has some v_omega
				#
				'OldStyleTurnLeft'			:	'p',
				'OldStyleTurnRight'			:	'q',
}

for n,v in SpecialPostureTable.items():
	SpecialPostureTable[n] = ord(v)

class LowLevelInterfaceException(Exception):
	pass 

class CommandQueue(object):

	def __init__(self, timeTolerance = None):
		self.__commandQueue = PriorityQueue()
		self.__timeTolerance = timeTolerance

	def add_command(self, timeStamp, command):
		if self.__commandQueue.full():
			self.__commandQueue.get()
		self.__commandQueue.put( (timeStamp, command) )

	def get_command(self, currentTime):
		command = None
		while 1==1:
			if self.__commandQueue.empty():
				break
			command = self.__commandQueue.get()
			if 	timeTolerance is None 	or 
				currentTime - command[0] <= self.__timeTolerance:
				break
			command = None
		return command

	def setTimeTolerance(self, value):
		if value > 0:
			self.__timeTolerance = value

class LocomotionCommand(object):

	def __init__(self, velX, velY, omgZ):
		self.velX = velX
		self.velY = velY
		self.omgZ = omgZ

	def tupleForm(sefl):
		return (self.velX, self.velY, self.omgZ)

	def __hash__(self):
		return hash(self.tupleForm())

	def __eq__(self, othr):
		try:
			return (self.velX==othr.velX and 
					self.velY==othr.velY and 
					self.omgZ==othr.omgZ)
		except AttributeError:
			return False

	def __ne__(self, othr):
		return not self.__eq__(othr)

class LocomotionSpecialCommand(object):

	def __init__(self, command):
		assert type(command) == str
		self.__command = command

	def stringForm(self):
		return self.__command

	def __eq__(self, othr):
		try:
			return self.__command == othr
		except TypeError:
			return False

	def __ne__(self, othr):
		return not self.__eq__(othr)


class LowLevelInterface(object):

	def __init__(self, serial,
			XWorldTranMax, XWorldTranMin, YWorldTranMax, YWorldTranMin, 
			ZWorldAnglMax, ZWorldAnglMin, XRegisTranMAx, XRegisTranMin, 
			YRegisTranMax, YRegisTranMin, ZRegisAnglMax, ZRegisAnglMin,
			timeout = None, timeTolerance = None):

		self.spinal_cord = HanumanProtocol(serial)

		self.__XWorldTranMax = XWorldTranMax
		self.__XWorldTranMin = XWorldTranMin
		self.__XRegisTranMAx = XRegisTranMAx
		self.__XRegisTranMin = XRegisTranMin

		self.__YWorldTranMax = YWorldTranMax
		self.__YWorldTranMin = YWorldTranMin
		self.__YRegisTranMax = YRegisTranMax
		self.__YRegisTranMin = YRegisTranMin

		self.__ZWorldAnglMax = ZWorldAnglMax
		self.__ZWorldAnglMin = ZWorldAnglMin
		self.__ZRegisAnglMax = ZRegisAnglMax
		self.__ZRegisAnglMin = ZRegisAnglMin

		self.__timeout = timeout
		self.__timeTolerance = timeTolerance
		self.__commandQueue = CommandQueue(self.__timeTolerance)

		self.__nextCommand = None
		self.__prevCommand = None

		self.__startCommandTime = None
		self.__stopCommandTime = None

	@staticmethod
	def convertLocomotionRegToWorld(regis, regisMax, regisMin, worldMax, worldMin):
		return max( min( ((worldMax-worldMin)/(regisMax-regisMin))*(regis-regisMin)+worldMin, worldMax) , worldMin)

	@staticmethod
	def convertLocomotionWorldToReg(world, regisMax, regisMin, worldMax, worldMin):
		return max( min( ((regisMax-regisMin)/(worldMax-worldMin))*(world-worldMin)+regisMin, regisMax) , regisMin)

	def convertVelocityToRegister(self, v_x, v_y, a_z):
		reg_x = self.convertLocomotionWorldToReg(v_x
												 self.__XRegisTranMAx,
												 self.__XRegisTranMin,
												 self.__XWorldTranMax,
												 self.__XWorldTranMin)
		reg_y = self.convertLocomotionWorldToReg(v_y,
												 self.__YRegisTranMax,
												 self.__YRegisTranMin,
												 self.__YWorldTranMax,
												 self.__YWorldTranMin)
		reg_z = self.convertLocomotionWorldToReg(a_z, 
												 self.__ZRegisAnglMax, 
												 self.__ZRegisAnglMin,
												 self.__ZWorldAnglMax,
												 self.__ZWorldAnglMin)
		return (reg_x, reg_y, reg_z)

	def convertRegisterToVelocity(self, r_x, r_y, r_z):
		vel_x = self.convertLocomotionRegToWorld(r_x,
												 self.__XRegisTranMAx,
												 self.__XRegisTranMin,
												 self.__XWorldTranMax,
												 self.__XWorldTranMin)
		vel_y = self.convertLocomotionRegToWorld(r_y,
												 self.__YRegisTranMax,
												 self.__YRegisTranMin,
												 self.__YWorldTranMax,
												 self.__YWorldTranMin)
		vel_z = self.convertLocomotionRegToWorld(r_z,
												 self.__ZRegisAnglMax,
												 self.__ZRegisAnglMin,
												 self.__ZWorldAnglMax,
												 self.__ZWorldAnglMin)
		return (vel_x, vel_y, vel_z)

	def recieveCommand(self, command, timeStamp = None):
		if isinstance(command, LocomotionCommand):
			actualCommand = self.convertVelocityToRegister(	command.velX, 
															command.velY, 
															command.omgZ)
		elif isinstance(command, LocomotionSpecialCommand):
			actualCommand = command.stringForm()
		else:
			raise LowLevelInterfaceException("Command is neither LocomotionSpecialCommand nor LocomotionSpecialCommand.")

		if timeStamp is None:
			self.__commandQueue.add_command(time.time(), actualCommand)
		else:
			self.__commandQueue.add_command(timeStamp, actualCommand)

	def doCurrentCommand(self, currentTime = None):
		if currentTime is None:
			currentTime = time.time()

		self.__nextCommand = self.__commandQueue.get_command(currentTime)

	def clearIntegration(self):
		pass

	def __executeCommand(self):
		isSuccess = False
		if isinstance(self.__nextCommand, tuple):
			isSuccess = self.__executeOmniwalk(self.__nextCommand)

		elif isinstance(self.__nextCommand, str):
			isSuccess = self.__executeSpecial(self.__nextCommand)
		if isSuccess:
			self.__prevCommand = self.__nextCommand

		# else:
		# 	raise LowLevelInterfaceException("Command is neither LocomotionSpecialCommand nor LocomotionSpecialCommand.")

	def __executeOmniwalk(self, command):
		response = self.spinal_cord.set_locomotion_command(SpecialPostureTable["ContinuousWalk"], timeout = self.__timeout)
		if response is None:
			return False
		response = self.spinal_cord.set_locomotion_velocity(command[0], 
															command[1], 
															command[2], 
													timeout = self.__timeout)
		return True if response is not None else False

	def __executeSpecial(self, commad):
		if not SpecialPostureTable.has_key(command):
			raise LowLevelInterfaceException("Error excecute command \""+command+"\". No such a command.")
		self.spinal_cord.set_locomotion_command(SpecialPostureTable[command],
												timeout = self.__timeout)

	def __integrateDistance(self):
		pass
