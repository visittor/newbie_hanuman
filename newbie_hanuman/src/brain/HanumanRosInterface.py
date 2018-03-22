#!/usr/bin/env python

from newbie_hanuman.srv import MotorCortexCommand,MotorCortexCommandResponse
from newbie_hanuman.srv import PanTiltPlannerCommand

from newbie_hanuman.msg import HanumanStatusMsg

from RosInterface import RosInterface
from visionManager.visionModule import KinematicModule

import rospy

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

def testInterface(kinematic_msgType):
	rospy.init_node("test_hanuman_rosinterface")
	rate = rospy.Rate(1)
	interface = HanumanRosInterface()
	interface.setKinematicInterface(kinematic_msgType)
	rospy.loginfo("Start node.")
	while not rospy.is_shutdown():
		rate.sleep()
	rospy.loginfo("Close node.")

def main():
	fileName = rospy.get_param("/vision_manager/vision_module_path", '')
	if fileName == '':
		module = KinematicModule()
	else:
		print "get module"
		lobe_module = load_module(fileName)
		assert hasattr( lobe_module, 'vision_module' )
		module = lobe_module.vision_module
	try:
		testInterface(module.posDictMsgType)
	except rospy.ROSInterruptException as e:
		pass
	finally:
		pass