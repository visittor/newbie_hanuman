#!/usr/bin/env python
from brain.brainState import FSMBrainState
from brain.SuchartInterface import SuchartRosInterface

from Brain.scanWorkSpace import ScanWorkSpace
from Brain.idle import Idle

from math import radians as rad

import time
import rospy

class Idle(FSMBrainState):
	def __init__(self):
		super(Idle, self).__init__("Idle")



class MainBrain(FSMBrainState):
	def __init__(self, nextState = None):
		super(MainBrain, self).__init__("MainBrain")

		self.__nextState = nextState

		scanWorkSpace = ScanWorkSpace(nextstate="Idle")
		idle = Idle()

		self.addSubBrain(scanWorkSpace)

		self.addSubBrain(idle)

		self.setFirstSubBrain("ScanWorkSpace")

	def firstStep(self):
		pass

	def step(self):
		# pass
		if self.currSubBrainName == "Idle" and self.subBrains["ScanWorkSpace"].isFinish:
			rospy.loginfo(self.subBrains["ScanWorkSpace"].finishingCoorBoard())

main_brain = MainBrain()