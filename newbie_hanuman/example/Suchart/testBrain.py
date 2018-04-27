#!/usr/bin/env python
from brain.brainState import FSMBrainState
from brain.SuchartInterface import SuchartRosInterface

from sensor_msgs.msg import JointState

import time
import math
import rospy

PI = math.pi
class Test1(FSMBrainState):
	def __init__(self, nextState=None):
		super(Test1, self).__init__("Test1")
		self.__jointName = ["joint"+str(i+1) for i in range(6)]
		print self.__jointName
		position1 = [-PI,0,5*PI/6,0,0,0]
		position2 = [0,0,0,0,0,0]
		self.__pattern = [
						{"goalPosition":JointState(	name=self.__jointName,
													position=position1),
						"command":"reach_position"},
						# {"command":"active_gripper"},
						{"goalPosition":JointState(	name=self.__jointName,
													position=position2),
						 "command":"reach_position"},
						 # {"command":"release_gripper"}
						 ]
		self.__i = 0

	def firstStep(self):
		self.rosInterface.pathPlaning.waitForServer()
		self.rosInterface.pathPlaning.setGoal(command="release_gripper")

	def step(self):
		if not self.rosInterface.pathPlaning.isFinish:
			return

		command = self.__pattern[self.__i]
		print "do command", command["goalPosition"].position
		self.rosInterface.pathPlaning.setGoal(**command)
		self.__i = (self.__i+1)%len(self.__pattern)

main_brain = Test1()