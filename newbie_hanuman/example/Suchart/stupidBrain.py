#!/usr/bin/env python
import numpy as np

from brain.brainState import FSMBrainState
from brain.SuchartInterface import SuchartRosInterface
from utility.performanceX import Timer

from Brain.scanWorkSpace import ScanWorkSpace
from Brain.placeBoard import PlaceBoards
from Brain.idle import Idle
# from Brain import robotkinematics import SCARAKinematics

import rospy

from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState

from math import radians as rad
import math

PI = math.pi
class StupidBrain(FSMBrainState):
	def __init__(self, nextstate=None):
		super(StupidBrain, self).__init__("StupidBrain")
		self.nextState = nextstate

		self.__jointName = ["joint"+str(i+1) for i in range(6)]
		print self.__jointName
		initial = [-PI, 0.0, PI/2, 0.0, 0.0, -PI/4]

		position1 = [-PI, 200, rad(112), 0.0, 0.0, -PI/4]
		position2 = [-rad(112), 200, rad(112), 0.0, 0.0, -PI/4]
		position3 = [-rad(112), -100, rad(112), 0.0, 0.0, -PI/4]
		position3_2 = [-rad(112), 150, rad(112), 0.0, 0.0, -PI/4]
		position4 = [0.0, 460, rad(90), 0.0, 0.0, -PI/4]
		position5 = [0.0, 460, rad(90), -rad(90), 0.0, -PI/4]
		position6 = [0.0, 460, rad(90), -rad(90), rad(90), -PI/4]
		position6 = [0.0, 460, rad(90), -rad(56), rad(90), -PI/4]
		position7 = [rad(5), 460, rad(45), -rad(56), rad(90), -PI/4]

		self.__timer = Timer()
		self.__pattern = [
						{"command":"release_gripper"},

						{"goalPosition":JointState(	name=self.__jointName,
													position=position1),
						"command":"reach_position",
						"ignoreObstacles":True},

						{"goalPosition":JointState(	name=self.__jointName,
													position=position2),
						 "command":"reach_position",
						 "ignoreObstacles":True},

						 # {"command":"release_gripper"}
						{"goalPosition":JointState(	name=self.__jointName,
													position=position3),
						"command":"reach_position",
						"ignoreObstacles":True},

						{"command":"active_gripper"},

						{"goalPosition":JointState(	name=self.__jointName,
													position=position3_2),
						"command":"reach_position",
						"ignoreObstacles":True},

						{"goalPosition":JointState(	name=self.__jointName,
													position=position4),
						"command":"reach_position",
						"ignoreObstacles":True},

						{"goalPosition":JointState(	name=self.__jointName,
													position=position5),
						"command":"reach_position",
						"ignoreObstacles":True},

						{"goalPosition":JointState(	name=self.__jointName,
													position=position6),
						"command":"reach_position",
						"ignoreObstacles":True},

						{"goalPosition":JointState(	name=self.__jointName,
													position=position7),
						"command":"reach_position",
						"ignoreObstacles":True},

						{"command":"release_gripper"},
						]
		self.__i = 0

	def firstStep(self):
		print "WTF"
		self.rosInterface.pathPlaning.waitForServer()
		self.rosInterface.pathPlaning.setGoal(command="release_gripper")

	def step(self):
		if not self.rosInterface.pathPlaning.isFinish:
			# print "not finish"
			return
		elif not self.__timer.is_begin():
			rospy.loginfo("start Timer.")
			self.__timer.start(2)
			return

		if not self.__timer.is_timeUp():
			return

		rospy.loginfo("Stop Timer.")
		self.__timer.reset()
		command = self.__pattern[self.__i]
		print "do command", command
		self.rosInterface.pathPlaning.setGoal(**command)
		self.__i = self.__i+1

		if self.__i >= len(self.__pattern):
			self.__i = 0
			self.SignalChangeSubBrain(self.nextState)

print "WTFF"

main_brain = StupidBrain()