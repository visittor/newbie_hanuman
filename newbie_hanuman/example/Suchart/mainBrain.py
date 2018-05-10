#!/usr/bin/env python
import numpy as np

from brain.brainState import FSMBrainState
from brain.SuchartInterface import SuchartRosInterface

from Brain.scanWorkSpace import ScanWorkSpace
from Brain.placeBoard import PlaceBoards
from Brain.idle import Idle

from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState

from math import radians as rad

import time
import rospy

class Ready(FSMBrainState):
	def __init__(self, nextstate=None):
		super(Ready, self).__init__("Ready")

		self.nextstate = nextstate
		jointName = ["joint"+str(i+1) for i in range(6)]
		self.__readyPos = [-np.pi/2, 100.0, 5*np.pi/6, 0.0, 0.0, 0.0]

		self.__readyCommand = {	"goalPosition":JointState(name=jointName,
													position=self.__readyPos),
								"command":"reach_position"}

		self.__isFinish = False

	def firstStep(self):
		self.__isFinish = False
		rospy.loginfo("Suchart brain is ready.")
		self.rosInterface.pathPlaning.waitForServer()
		self.rosInterface.pathPlaning.setGoal(**self.__readyCommand)

	def step(self):
		if not self.rosInterface.pathPlaning.isFinish:
			return

		if self.rosInterface.motorCortex.gameStatusInitialState:
			self.__isFinish = False
			return

		elif self.rosInterface.motorCortex.gameStatusPlayState:
			self.__isFinish = True
			if self.nextstate is None:
				return
			self.SignalChangeSubBrain(self.nextstate)
			return

	@property
	def isfinish(self):
		return self.__isFinish

class Playgame(FSMBrainState):
	def __init__(self, nextstate = None):
		super(Playgame, self).__init__("Playgame")

		self.__nextstate = nextstate

		scanWorkSpace = ScanWorkSpace(nextstate="Idle")
		idle = Idle()
		placeboards = PlaceBoards(nextstate="Idle")

		self.addSubBrain(scanWorkSpace)

		self.addSubBrain(placeboards)

		self.addSubBrain(idle)

		self.setFirstSubBrain("Idle")

	def firstStep(self):
		## comment code below out in real situation
		jointName = ["joint"+str(i+1) for i in range(6)]
		self.__readyPos = [-np.pi, 100.0, 3*np.pi/4, 0.0, 0.0, 0.0]
		self.__readyCommand = {	"goalPosition":JointState(name=jointName,
													position=self.__readyPos),
								"command":"reach_position",
								"ignoreObstacles":True}
		self.rosInterface.pathPlaning.waitForServer()
		self.rosInterface.pathPlaning.setGoal(**self.__readyCommand)
		rospy.loginfo("Suchart start playing a game.")

	def step(self):
		# pass
		# if not self.rosInterface.pathPlaning.isFinish:
		# 	return

		# if self.currSubBrainName == "Idle" and self.prevSubBrainName=="PlaceBoards":
		# 	return

		# if self.currSubBrainName == "Idle" and self.subBrains["ScanWorkSpace"].isFinish:
		# 	coorboard = self.subBrains["ScanWorkSpace"].finishingCoorBoard()
		# 	rospy.loginfo(str(coorboard))
		# 	self.subBrains["PlaceBoards"].set_boardCoor(coorboard)
		# 	self.ChangeSubBrain("PlaceBoards")


		# For debug
		# if not self.rosInterface.pathPlaning.isFinish:
		# 	# print "not finish"
		# 	return

		# if self.currSubBrainName == "Idle" and self.prevSubBrainName!="PlaceBoards":
		# 	coorboard = [	(Point32(x=0.2734,y=0.475,z=0.636),0),
		# 					(Point32(x=0.485,y=-0.475,z=0.771),12),
		# 					(Point32(x=0.6423,y=0.373,z=0.025),19),]
		# 	self.subBrains["PlaceBoards"].set_boardCoor(coorboard)
		# 	self.ChangeSubBrain("PlaceBoards")

		# For debug
		if not self.rosInterface.pathPlaning.isFinish:
			# print "not finish"
			return

		if self.currSubBrainName == "Idle" and self.prevSubBrainName!="PlaceBoards":
			coorboard = [	(Point32(x=0.27,y=0.475,z=0.621),0),
							(Point32(x=0.52,y=-0.475,z=0.627),29),
							(Point32(x=0.57,y=-0.109,z=0.025),24),]
			self.subBrains["PlaceBoards"].set_boardCoor(coorboard)
			self.ChangeSubBrain("PlaceBoards")

class MainBrain(FSMBrainState):
	def __init__(self):
		super(MainBrain, self).__init__("MainBrain")

		readyBrain = Ready(nextstate="Playgame")
		playgame = Playgame(nextstate="Ready")

		self.addSubBrain(readyBrain)
		self.addSubBrain(playgame)

		self.setFirstSubBrain("Ready")

	def firstStep(self):
		pass

	def step(self):
		pass


main_brain = Playgame()