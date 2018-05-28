#!/usr/bin/env python
import numpy as np
from brain.brainState import FSMBrainState
from brain.SuchartInterface import SuchartRosInterface

from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState

from robotkinematics.scarakinematics import SCARAKinematics
from idle import Idle

import rospy

import math

JOINT_NAME = ["joint"+str(i+1) for i in range(6)]
PI = math.pi

class UTurn(FSMBrainState):

	def __init__(self, nextstate=None):
		super(UTurn, self).__init__("UTurn")

		self.nextstate = nextstate

		self.__posL = [-PI, 350.0, 3*PI/4, 0.0, 0.0, 0.0]
		self.__posR = [-PI, 350.0,-3*PI/4, 0.0, 0.0, 0.0]

		self.__begin = "left"
		self.__is_uturn = False

		self.__state = 0
		self.__isFinish = False

	def setUTurn(self, is_uturn, begin):
		assert begin=="left" or begin=="right"
		self.__is_uturn = is_uturn
		self.__begin = begin
		self.__state = 0
		self.__isFinish = not is_uturn

	def jointstateToList(self, js):
		q = []
		for i in range(6):
			indx = js.name.index("joint"+str(i+1))
			q.append(js.position[indx])
		return q

	def firstStep(self):
		rospy.loginfo("UTurn "+str(self.__is_uturn))
		if not self.__is_uturn:
			self.SignalChangeSubBrain(self.nextstate)

	def step(self):
		if not self.rosInterface.pathPlaning.isFinish:
			return

		if self.__isFinish:
			self.__state = 0
			# self.__isFinish = False
			self.__is_uturn = False
			self.SignalChangeSubBrain(self.nextstate)

		if self.__state == 0:
			print "fuck"
			q_pre=self.jointstateToList(self.rosInterface.motorCortex.currPosition)

			q_pre[2] = 3*PI/4 if self.__begin=="left" else -3*PI/4
			q_pre[3] = 0
			q_pre[4] = 0
			q_pre[5] = 0

			command = 	{"goalPosition":JointState(	name=JOINT_NAME,
													position=q_pre),
						"command":"reach_position",
						"ignoreObstacles":True}
			self.rosInterface.pathPlaning.setGoal(**command)
			self.__state = 1

		elif self.__state == 1:
			print "fuck1"
			q_pre=self.jointstateToList(self.rosInterface.motorCortex.currPosition)

			q_pre[0] = -PI
			q_pre[3] = 0
			q_pre[4] = 0
			q_pre[5] = 0

			command = 	{"goalPosition":JointState(	name=JOINT_NAME,
													position=q_pre),
						"command":"reach_position",
						"ignoreObstacles":True}
			self.rosInterface.pathPlaning.setGoal(**command)
			self.__state = 2

		elif self.__state == 2:
			print "fuck2"
			q_pre=self.jointstateToList(self.rosInterface.motorCortex.currPosition)

			q_pre[2] = -3*PI/4 if self.__begin=="left" else 3*PI/4
			q_pre[3] = 0
			q_pre[4] = 0
			q_pre[5] = 0

			command = 	{"goalPosition":JointState(	name=JOINT_NAME,
													position=q_pre),
						"command":"reach_position",
						"ignoreObstacles":True}
			self.rosInterface.pathPlaning.setGoal(**command)
			self.__state = 0
			self.__isFinish = True

	def isFinish(self):
		return self.__isFinish