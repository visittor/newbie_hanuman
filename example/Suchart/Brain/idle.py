#!/usr/bin/env python
from brain.brainState import FSMBrainState

import rospy

class Idle(FSMBrainState):
	def __init__(self):
		super(Idle, self).__init__("Idle")

	def firstStep(self):
		rospy.loginfo("Idle")