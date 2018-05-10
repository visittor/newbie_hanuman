#!/usr/bin/env python
from brain.brainState import FSMBrainState
from brain.SuchartInterface import SuchartRosInterface
from utility.performanceX import Timer

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
		position1 = [-PI, 100, 0.0, 0.0, 0.0, 0.0]
		# position1 = [-PI,0,5*PI/6,0,0,0]
		position2 = [0.0, 200.0, 0.0, 0.0, 0.0, 0.0]
		position3 = [PI,300, 0.0, 0.0, 0.0, 0.0]
		# position4 = [PI,0,5*PI/6,0,0,0]
		self.__timer = Timer()
		self.__pattern = [
						{"goalPosition":JointState(	name=self.__jointName,
													position=position1),
						"command":"reach_position",
						"ignoreObstacles":True},
						# {"command":"active_gripper"},
						{"goalPosition":JointState(	name=self.__jointName,
													position=position2),
						 "command":"reach_position",
						 "ignoreObstacles":True},
						 # {"command":"release_gripper"}
						{"goalPosition":JointState(	name=self.__jointName,
													position=position3),
						"command":"reach_position",
						"ignoreObstacles":True},

						{"goalPosition":JointState(	name=self.__jointName,
													position=position2),
						"command":"reach_position",
						"ignoreObstacles":True},
						]
		self.__i = 0

	def firstStep(self):
		print "WTF"
		self.rosInterface.pathPlaning.waitForServer()
		self.rosInterface.pathPlaning.setGoal(command="release_gripper")

	def step(self):
		if not self.rosInterface.pathPlaning.isFinish:
			# print "not finish"
			self.__timer.start(5)
			return
		if not self.__timer.is_timeUp():
			self.__timer.reset()
			return

		command = self.__pattern[self.__i]
		print "do command", command
		self.rosInterface.pathPlaning.setGoal(**command)
		self.__i = (self.__i+1)%len(self.__pattern)
print "WTFF"
main_brain = Test1()