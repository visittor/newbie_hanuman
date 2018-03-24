#!/usr/bin/env python
from HumanoidBase import *
from Queue import PriorityQueue, Queue

import time
import math

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
		self.__commandQueue = Queue()
		self.__timeTolerance = timeTolerance

		self.__finalCommand = None
		self.__finalCommandTimeStamp = None

	def add_command(self, timeStamp, command, ignorable = True):
		if self.__commandQueue.full():
			self.__commandQueue.get()
			self.__commandQueue.task_done()
		if command == self.__finalCommand:
			return
		self.__commandQueue.put( (timeStamp, command, ignorable) )
		self.__finalCommand = command
		self.__finalCommandTimeStamp = timeStamp

	def getFinalCommandInfo(self):
		return self.__finalCommand, self.__finalCommandTimeStamp

	def get_command(self, currentTime):
		command = None
		if currentTime is None:
			currentTime = time.time()
		while 1==1:
			if self.__commandQueue.empty():
				self.__finalCommand = None
				self.__finalCommandTimeStamp = None
				break
			command = self.__commandQueue.get()
			if 	self.__timeTolerance is None or currentTime - command[0] <= self.__timeTolerance or not command[2]:
				break
			command = None
		return command

	def task_done(self):
		self.__commandQueue.task_done()

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

	def __str__(self):
		return str(self.velX)+","+str(self.velY)+","+str(self.omgZ)

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

	def __str__(self):
		return self.__command

class HanumanStatus(object):

	def __init__(self, statusPackage):
		self.statusDict = {}
		self.statusDict["firmware"] = HanumanProtocol.get_robotStatus(
													statusPackage, 
													"REG_FIRMWARE_VERSION_L")
		self.statusDict["firmware"] += HanumanProtocol.get_robotStatus(
												statusPackage, 
												"REG_FIRMWARE_VERSION_H")<<8
		self.statusDict["lococommand"] = HanumanProtocol.get_robotStatus(
												statusPackage, 
												"REG_LOCOMOTION_COMMAND")
		self.statusDict["locostatus"] = HanumanProtocol.get_robotStatus(
												statusPackage, 
												"REG_LOCOMOTION_STATUS_L")
		self.statusDict["locostatus"] += HanumanProtocol.get_robotStatus(
												statusPackage,
												"REG_LOCOMOTION_STATUS_H")<<8
		self.statusDict["backFallDown"]=HanumanInterface.is_backFallDown(
												self.statusDict["locostatus"])

		self.statusDict["frontFallDown"]=HanumanInterface.is_frontFallDown(
												self.statusDict["locostatus"])
		
		self.statusDict["compass"] = HanumanProtocol.get_robotStatus(
														statusPackage,
														"REG_COMPASS_L")
		self.statusDict["compass"] += HanumanProtocol.get_robotStatus(
														statusPackage, 
														"REG_COMPASS_H")<<8
		self.statusDict["gyroX"] = HanumanProtocol.get_robotStatus(
													statusPackage,
													"REG_GYRO_ROT_X_L")
		self.statusDict["gyroX"] += HanumanProtocol.get_robotStatus(
													statusPackage, 
													"REG_GYRO_ROT_X_H")<<8
		self.statusDict["gyroY"] = HanumanProtocol.get_robotStatus(
													statusPackage, 
													"REG_GYRO_ROT_Y_L")
		self.statusDict["gyroY"] += HanumanProtocol.get_robotStatus(
													statusPackage, 
													"REG_GYRO_ROT_Y_H")<<8
		self.statusDict["gyroZ"] = HanumanProtocol.get_robotStatus(
													statusPackage,
													"REG_GYRO_ROT_Z_L")
		self.statusDict["gyroZ"] += HanumanProtocol.get_robotStatus(
													statusPackage,
													"REG_GYRO_ROT_Z_H")<<8
		self.statusDict["linearVelX"] = HanumanProtocol.get_robotStatus(
											statusPackage,
											"REG_ROBOT_LINEAR_VELOCITY_X")
		self.statusDict["linearVelY"] = HanumanProtocol.get_robotStatus(
											statusPackage, 
											"REG_ROBOT_LINEAR_VELOCITY_Y")
		self.statusDict["angularVel"] = HanumanProtocol.get_robotStatus(
											statusPackage,
											"REG_ROBOT_ANGULAR_VELOCITY")
		self.statusDict["magnetoX"] = HanumanProtocol.get_robotStatus(
											statusPackage,
											"REG_MAGNETO_X_L")
		self.statusDict["magnetoX"] += HanumanProtocol.get_robotStatus(
											statusPackage,
											"REG_MAGNETO_X_H")<<8
		self.statusDict["magnetoY"] = HanumanProtocol.get_robotStatus(
											statusPackage,
											"REG_MAGNETO_Y_L")
		self.statusDict["magnetoY"] += HanumanProtocol.get_robotStatus(
											statusPackage,
											"REG_MAGNETO_Y_H")<<8
		self.statusDict["magnetoZ"] = HanumanProtocol.get_robotStatus(
											statusPackage,
											"REG_MAGNETO_Z_L")
		self.statusDict["magnetoZ"] += HanumanProtocol.get_robotStatus(
											statusPackage,
											"REG_MAGNETO_Z_H")<<9

class HanumanInterface(object):

	def __init__(self, serial,
			XWorldTranMax, XWorldTranMin, YWorldTranMax, YWorldTranMin, 
			ZWorldAnglMax, ZWorldAnglMin, XRegisTranMax, XRegisTranMin, 
			YRegisTranMax, YRegisTranMin, ZRegisAnglMax, ZRegisAnglMin,
			timeout = None, timeTolerance = None, transitionTime = 1.0):

		self.spinal_cord = HanumanProtocol(serial)

		self.__XWorldTranMax = XWorldTranMax
		self.__XWorldTranMin = XWorldTranMin
		self.__XRegisTranMAx = XRegisTranMax
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

		self.__nextCommand = LocomotionSpecialCommand("StandStill")
		self.__prevCommand = LocomotionSpecialCommand("StandStill")

		# self.__commandQueue.add_command(time.time(), self.__nextCommand)

		self.__startCommandTime = time.time()
		self.__stopCommandTime = time.time()

		self.__startCommandPos = (0.0,0.0,0.0)
		self.__currentPosition = (0.0,0.0,0.0)

		self.__stopCommad = self.convertVelocityToRegister(	0.0, 
															0.0, 
															0.0)
		self.__transitionTime = transitionTime
		self.__waitTransition = False

	@staticmethod
	def convertLocomotionRegToWorld(regis, regisMax, regisMin, worldMax, worldMin):
		return max( min( ((worldMax-worldMin)/(regisMax-regisMin))*(regis-regisMin)+worldMin, worldMax) , worldMin)

	@staticmethod
	def convertLocomotionWorldToReg(world, regisMax, regisMin, worldMax, worldMin):
		world = min(max(world,-1), 1)
		world *= worldMax
		return max( min( ((regisMax-regisMin)/(worldMax-worldMin))*(world-worldMin)+regisMin, regisMax) , regisMin)

	def convertVelocityToRegister(self, v_x, v_y, a_z):
		reg_x = self.convertLocomotionWorldToReg(v_x,
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
		return LocomotionCommand(reg_x, reg_y, reg_z)

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
		return LocomotionCommand(vel_x, vel_y, vel_z)

	def getCommand(self, command, timeStamp = None, ignorable = True):
		if isinstance(command, LocomotionCommand):
			actualCommand = self.convertVelocityToRegister(	command.velX, 
															command.velY, 
															command.omgZ)
		elif isinstance(command, LocomotionSpecialCommand):
			actualCommand = command
		else:
			raise LowLevelInterfaceException("Command is neither LocomotionSpecialCommand nor LocomotionSpecialCommand.")

		# if actualCommand == self.__stopCommad:
		# 	ignorable = False

		finalCommand = self.__commandQueue.getFinalCommandInfo()
		self.__filterCommand(actualCommand, finalCommand[0], timeStamp)
		# print finalCommand[0]
		# print actualCommand

		if timeStamp is None:
			self.__commandQueue.add_command(time.time(), 
											actualCommand, 
											ignorable = ignorable)
		else:
			# print "Should add command here"
			self.__commandQueue.add_command(timeStamp, 
											actualCommand,
											ignorable = ignorable)

	def forceStop(self, currentTime = None):
		self.__nextCommand = self.__stopCommad
		self.__executeCommand()

	def forceStandup(self, currentTime = None):
		prevCommand = self.__prevCommand
		self.__nextCommand = LocomotionSpecialCommand("Standup")
		self.__executeCommand()

	def doCurrentCommand(self, currentTime = None):
		if currentTime is None:
			currentTime = time.time()

		if self.__waitTransition and time.time() - self.__startCommandTime <= self.__transitionTime:
			return
		self.__waitTransition = False
		command = self.__commandQueue.get_command(currentTime)
		self.__nextCommand = command[1] if command is not None else None
		self.__executeCommand()
		if self.__nextCommand is not None:
			self.__commandQueue.task_done()
		rospy.loginfo("Do "+str(self.__nextCommand))

	def clearIntegration(self):
		self.__currentPosition = (0.0,0.0,0.0)
		self.__startCommandPos = (0.0,0.0,0.0)
		self.__startCommandTime = time.time()

	def doIntegration(self):
		self.__integrateDistance()
		return self.__currentPosition

	def getCurrentPosition(self):
		return self.doIntegration()

	def __shouldStopBeforeDoCommand(self, command, finalCommand):
		if isinstance(command, LocomotionSpecialCommand):
			return True

		elif isinstance(command, LocomotionCommand) and isinstance(finalCommand, LocomotionCommand):
			# Change direction in X direction.
			if command.velX * finalCommand.velX < 0:
				return True
			# Change direction in Y direction.
			elif command.velY * finalCommand.velY < 0:
				return True
			# Change rotation direction.
			elif command.omgZ * finalCommand.omgZ < 0:
				return True

	def __filterCommand(self, command, finalCommand, timeStamp):
		if command == finalCommand:
			return

		if self.__shouldStopBeforeDoCommand(command, finalCommand):
			if timeStamp is None:
				self.__commandQueue.add_command(time.time(),
												self.__stopCommad,
												ignorable = False)
			else:
				self.__commandQueue.add_command(timeStamp,
												self.__stopCommad,
												ignorable = False)
			return

	def __executeCommand(self):
		isSuccess = False
		# print self.__nextCommand
		if self.__nextCommand is None:
			isSuccess = False
		elif isinstance(self.__nextCommand, LocomotionCommand):
			isSuccess = self.__executeOmniwalk(self.__nextCommand)

		elif isinstance(self.__nextCommand, LocomotionSpecialCommand):
			isSuccess = self.__executeSpecial(self.__nextCommand)
			self.__waitTransition = isSuccess

		else:
			raise LowLevelInterfaceException("Command is neither LocomotionSpecialCommand nor LocomotionSpecialCommand.")

		self.__integrateDistance()

		if isSuccess:
			self.__prevCommand = self.__nextCommand
			self.__startCommandTime = time.time()
			self.__startCommandPos = self.__currentPosition

	def __executeOmniwalk(self, command):
		response = self.spinal_cord.set_locomotion_velocity(command.velX, 
															command.velY, 
															command.omgZ, 
													timeout = self.__timeout)
		if response is None:
			return False
		response = self.spinal_cord.set_locomotion_command(SpecialPostureTable["ContinuousWalk"], timeout = self.__timeout)
		return True if response is not None else False

	def __executeSpecial(self, command):
		if not SpecialPostureTable.has_key(command.stringForm()):
			raise LowLevelInterfaceException("Error excecute command \""+command.stringForm()+"\". No such a command.")
		response = self.spinal_cord.set_locomotion_command(
									SpecialPostureTable[command.stringForm()],
									timeout = self.__timeout
									)
		return True if response is not None else False

	def __integrateDistance(self):
		x_i = self.__startCommandPos[0]
		y_i = self.__startCommandPos[1]
		theta_i = self.__startCommandPos[2]
		if isinstance(self.__prevCommand, LocomotionCommand):
			vel = self.convertRegisterToVelocity(self.__prevCommand.velX,
												 self.__prevCommand.velY,
												 self.__prevCommand.omgZ)
			timeInterval = time.time() * self.__startCommandTime
			vx = vel.velX
			vy = vel.velY
			mZ = vel.omgZ
			if mZ != 0:
				x = (vx*math.sin(mZ*timeInterval)+vy*math.cos(mZ*timeInterval))/mZ
				x += x_i
				y = (vy*math.sin(mZ*timeInterval)-vx*math.cos(mZ*timeInterval))/mZ
				y += y_i
			else:
				x = vx*timeInterval + x_i
				y = vy*timeInterval + y_i
			theta = mZ * timeInterval
			theta += theta_i
			theta = theta%(2*math.pi)

		else:
			x = x_i
			y = y_i
			theta = theta_i%(2*math.pi)

		self.__currentPosition = (x,y,theta)

	@staticmethod
	def is_backFallDown( robotStatus ):
		return TiltStatus_FallDown_FaceUp == (robotStatus&TiltStatusMask)

	@staticmethod
	def is_frontFallDown( robotStatus ):
		return TiltStatus_FallDown_FaceDown == (robotStatus&TiltStatusMask)

	def is_robotStanding(self):
		# print "ee"
		self.spinal_cord.read_AllLowLevelData(timeout = self.__timeout)
		# print "dd"
		robotStatus = self.spinal_cord.get_locomotionStatus()
		# print robotStatus
		if robotStatus is None:
			return True

		if self.is_backFallDown( robotStatus ):
			return False

		elif self.is_frontFallDown( robotStatus ):
			return False

		return True

	def getHanumanStatus(self):
		self.spinal_cord.read_AllLowLevelData()
		statusPackage = self.spinal_cord.get_statusPackage()
		if statusPackage is None:
			return
		hanumanStatus = HanumanStatus(statusPackage)
		return hanumanStatus