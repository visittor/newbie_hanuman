#!/usr/bin/env python
from brain.brainState import FSMBrainState
from brain.HanumanRosInterface import HanumanRosInterface

import time
import rospy

class MoveForward(FSMBrainState):
	def __init__(self):
		super(MoveForward, self).__init__("MoveForward")
		# print self.name
		self.rosInterface = None

	def firstStep(self):
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False)
		self.rosInterface.LocoCommand(	velX = 0.5,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False)

	def step(self):
		pass

class MoveBackward(FSMBrainState):
	def __init__(self):
		super(MoveBackward, self).__init__("MoveBackward")
		print self.name
		self.rosInterface = None

	def firstStep(self):
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False)
		self.rosInterface.LocoCommand(	velX = -0.5,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False)

	def step(self):
		pass

class TurnLeft(FSMBrainState):
	def __init__(self):
		super(TurnLeft, self).__init__("TurnLeft")
		self.rosInterface = None

	def firstStep(self):
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False)
		self.rosInterface.LocoCommand(	velX = -0.0,
										velY = 0.0,
										omgZ = 0.5,
										commandType = 0,
										ignorable = False)

	def step(self):
		pass

class TurnRight(FSMBrainState):
	def __init__(self):
		super(TurnRight, self).__init__("TurnRight")
		self.rosInterface = None

	def firstStep(self):
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False)
		self.rosInterface.LocoCommand(	velX = -0.0,
										velY = 0.0,
										omgZ = -0.5,
										commandType = 0,
										ignorable = False)

	def step(self):
		pass

class MoveForwardTimer(FSMBrainState):
	def __init__(self, nextState, time):
		super(MoveForwardTimer, self).__init__("MoveForwardTimer")
		self.nextState = str(nextState)
		self.time = time
		self.__startTime = None

	def firstStep(self):
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False)
		self.__startTime = time.time()
		self.rosInterface.LocoCommand(	velX = 0.5,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False)

	def step(self):
		if time.time() - self.__startTime > self.time:
			self.SignalChangeSubBrain(self.nextState)

class MoveBackwardTimer(FSMBrainState):
	def __init__(self, nextState, time):
		super(MoveBackwardTimer, self).__init__("MoveBackwardTimer")
		self.nextState = nextState
		self.time = time
		self.__startTime = None

	def firstStep(self):
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False)
		self.__startTime = time.time()
		self.rosInterface.LocoCommand(	velX = -0.5,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False) 

	def step(self):
		if time.time() - self.__startTime > self.time:
			self.SignalChangeSubBrain(self.nextState)

class TurnLeftTimer(FSMBrainState):
	def __init__(self, nextState, time):
		super(TurnLeftTimer, self).__init__("TurnLeftTimer")
		self.nextState = nextState
		self.time = time
		self.__startTime = None

	def firstStep(self):
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False)
		self.__startTime = time.time()
		self.rosInterface.LocoCommand(	velX = -0.0,
										velY = 0.0,
										omgZ = 0.5,
										commandType = 0,
										ignorable = False) 

	def step(self):
		if time.time() - self.__startTime > self.time:
			self.SignalChangeSubBrain(self.nextState)

class TurnRightTimer(FSMBrainState):
	def __init__(self, nextState, time):
		super(TurnRightTimer, self).__init__("TurnRightTimer")
		self.nextState = nextState
		self.time = time
		self.__startTime = None

	def firstStep(self):
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False)
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = -0.5,
										commandType = 0,
										ignorable = False) 
		self.__startTime = time.time()
		# print "Fucking Turn Right."

	def step(self):
		if time.time() - self.__startTime > self.time:
			self.SignalChangeSubBrain(self.nextState)

class Stop(FSMBrainState):
	def __init__(self, nextState = None):
		super(Stop, self).__init__("Stop")
		self.rosInterface = None
		self.nextState = nextState

	def firstStep(self):
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False)

	def step(self):
		if self.nextState is not None:
			self.SignalChangeSubBrain(self.nextState)

class KickRight(FSMBrainState):
	def __init__(self, nextState = None):
		super(KickRight, self).__init__("KickRight")
		self.rosInterface = None
		self.nextState = nextState
		self.__timeDelay = 2.0
		self.__initTime = None

	def firstStep(self):
		self.rosInterface.LocoCommand(	command = "RightKick",
										commandType = 1,
										ignorable = False)
		self.__initTime = time.time()

	def step(self):
		if self.nextState is not None and time.time() - self.__initTime>self.__timeDelay:
			self.SignalChangeSubBrain(self.nextState)

class KickLeft(FSMBrainState):
	def __init__(self, nextState = None):
		super(KickLeft, self).__init__("KickLeft")
		self.rosInterface = None
		self.nextState = nextState
		self.__timeDelay = 2.0
		self.__initTime = None

	def firstStep(self):
		self.rosInterface.LocoCommand(	command = "LeftKick",
										commandType = 1,
										ignorable = False)
		self.__initTime = time.time()

	def step(self):
		if self.nextState is not None and time.time() - self.__initTime>self.__timeDelay:
			self.SignalChangeSubBrain(self.nextState)

class Example1(FSMBrainState):
	def __init__(self, nextState = None):
		super(Example1, self).__init__("Example1")
		self.__nextState = nextState
		moveForward = MoveForward()
		moveBackward = MoveBackward()
		stop = Stop()
		leftkick = KickLeft()
		rightkick = KickRight()

		self.addSubBrain(moveForward)
		self.addSubBrain(moveBackward)
		self.addSubBrain(stop)
		self.addSubBrain(leftkick)
		self.addSubBrain(rightkick)

		self.setFirstSubBrain("Stop")

		self.initTime = None

		self.count = 0

		self.rosInterface = None

	def firstStep(self):
		self.initTime = time.time()

	def step(self):

		if time.time() - self.initTime > 15.0:
			if self.__nextState is None:
				self.initTime = time.time()
				return
			rospy.logdebug("[Example1]Change to Example2")
			self.SignalChangeSubBrain(self.__nextState)

		elif time.time() - self.initTime >10.0:
			rospy.logdebug("[Example1]Change to MoveBackward")
			self.ChangeSubBrain("MoveBackward")

		elif time.time() - self.initTime > 6:
			rospy.logdebug("[Example1]Change to KickLeft")
			self.ChangeSubBrain("KickLeft")

		elif time.time() - self.initTime > 1:
			rospy.logdebug("[Example1]Change to MoveForward")
			self.ChangeSubBrain("MoveForward")

		if self.rosInterface.robotStatus is None:
			return
		# print self.rosInterface.robotStatus.linearX, self.rosInterface.robotStatus.linearY

class Example2(FSMBrainState):
	def __init__(self, nextState = None):
		super(Example2, self).__init__("Example2")

		self.__nextState = nextState
		turnLeft = TurnLeftTimer("", 5)
		turnRight = TurnRightTimer("", 5)
		rightkick = KickRight(nextState = "Stop")
		stop = Stop(nextState="TurnLeftTimer")

		turnLeft.nextState = "Stop"
		rightkick.nexState = "Stop"
		turnRight.nextState = "Stop"

		self.addSubBrain(turnLeft)
		self.addSubBrain(turnRight)
		self.addSubBrain(rightkick)
		self.addSubBrain(stop)

		self.setFirstSubBrain("Stop")

		self.initTime = None

		self.rosInterface = None

	def firstStep(self):
		self.initTime = time.time()

	def step(self):
		if self.currSubBrainName == "Stop":
			if self.prevSubBrainName == "TurnLeftTimer":
				self.subBrains["Stop"].nextState = "KickRight"
				rospy.logdebug("[Example2] Change to KickRight")

			elif self.prevSubBrainName == "KickRight":
				self.subBrains["Stop"].nextState = "TurnRightTimer"
				rospy.logdebug("[Example2] Change to TurnRightTimer")

			elif self.prevSubBrainName == "TurnRightTimer":
				self.subBrains["Stop"].nextState = "TurnLeftTimer"
				self.resetState()
				if self.__nextState is None:
					return
				rospy.logdebug("[Example2]"+str(self.currSubBrainName))
				rospy.logdebug("[Example2] Change to Example1")
				self.SignalChangeSubBrain(self.__nextState)
		# print self.subBrains["Stop"].nextState
		# if self.currSubBrainName == "TurnRightTimer":
		# 	print "WHATTTTTTTTTTT"
		# else:
		# 	print  self.currSubBrainName, self.prevSubBrainName

class MainBrain(FSMBrainState):
	def firstStep(self):
		self.rosInterface.Pantilt(	pattern="basic_pattern",
									command=1)

	def end(self):
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False)
		time.sleep(1)

example1 = Example1(nextState = "Example2")
example2 = Example2(nextState = "Example1")

main_brain = MainBrain("main_brain")
main_brain.addSubBrain(example1)
main_brain.addSubBrain(example2)
main_brain.setFirstSubBrain("Example1")