#!/usr/bin/env python

from newbie_hanuman.srv import MotorCortexCommand,MotorCortexCommandResponse
from newbie_hanuman.srv import PanTiltPlannerCommand

from newbie_hanuman.msg import HanumanStatusMsg

from RosInterface import RosInterface

class HanumanRosInterface(RosInterface):

	def __init__(self):
		super(HanumanRosInterface, self).__init__()
		self.interfaceWithService(	"motor_cortex_command", 
									MotorCortexCommand, 
									"LocoCommand")
		self.interfaceWithService( "pantiltplannercommand",
									PanTiltPlannerCommand,
									"Pantilt")
		self.interfaceWithPublisher("/spinalcord/hanuman_status", 
									HanumanStatusMsg, 
									"robotStatus")

	def setKinematicInterface(self, msgType):
		self.interfaceWithPublisher("/vision_manager/kinematic_topic",
									msgType,
									"visionManager")