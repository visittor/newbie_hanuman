#!/usr/bin/env python

from RosInterface import RosInterface

from newbie_hanuman.msg import SuchartFeedback
from newbie_hanuman.msg import SuchartAction 
from newbie_hanuman.msg import SuchartResult
from newbie_hanuman.msg import SuchartGoal
from newbie_hanuman.msg import SuchartPathPlaningCommandFeedback as Feedback 
from newbie_hanuman.msg import SuchartPathPlaningCommandAction as Action
from newbie_hanuman.msg import SuchartPathPlaningCommandResult as Result
from newbie_hanuman.msg import SuchartPathPlaningCommandGoal as Goal

from newbie_hanuman.msg import suchartPostDictMsg

class SuchartRosInterface(RosInterface):

	def __init__(self):
		super(SuchartRosInterface, self).__init__()

		self.interfaceWithAction("/spinalcord/suchart_path_planing", 
								Action, 
								Goal, 
								"pathPlaning")

		self.interfaceWithPublisher("/vision_manager/kinematic_topic",
									suchartPostDictMsg,
									"visionManager")