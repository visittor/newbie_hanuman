#!/usr/bin/env python

import rospy

from message_filters import Subscriber,ApproximateTimeSynchronizer
from sensor_msgs.msg import JointState

from cell.nodeCell import NodeBase
from brain.visionModule import VisionModule
from utility.utility import load_module

import inspect
import os
import imp
import sys
from Queue import Queue
import copy

class KinematicBrain(NodeBase):

	def __init__(self):
		super(KinematicBrain, self).__init__("kinematic_brain")
		self.__occipitalMessageType = None
		self.__posDictMessageType = None
		self.__messageFilter = None

		self.__kinematicFunction = lambda x,y:x
		self.__loop = lambda :None
		self.__module = None

	def initial(self, module):
		self.__module = module
		self.setOccipitalMessageType(self.__module.objectsMsgType)
		self.setPosDictMessageType(self.__module.posDictMsgType)
		self.setKinematicFunction(self.__module.kinematicCalculation)
		self.setKinematicLoop(self.__module.loop)
		self.__rosInit()
		self.setFrequency(60)

	def setOccipitalMessageType(self, msgType):
		self.__occipitalMessageType = msgType

	def setPosDictMessageType(self, msgType):
		self.__posDictMessageType = msgType

	def setKinematicFunction(self, func):
		self.__kinematicFunction = func

	def setKinematicLoop(self, func):
		self.__loop = func
		print "loop",self.__loop

	def __rosInit(self):
		self.rosInitNode()
		self.__initPublisher()
		self.__initMessageFilter()

	def __initPublisher(self):
		self.rosInitPublisher(	"/vision_manager/kinematic_topic", 
								self.__posDictMessageType)

	def __initMessageFilter(self):
		# rospy.Subscriber("/vision_manager/occipital_lobe_topic",
		# 				self.__occipitalMessageType,
		# 				self.ddd, 
		# 				queue_size=1)

		# rospy.Subscriber("/spinalcord/sternocleidomastoid_position",
		# 				JointState,
		# 				self.eee, 
		# 				queue_size=1)

		objSub = Subscriber("/vision_manager/occipital_lobe_topic",
							self.__occipitalMessageType, buff_size = 2**24)
		panTiltSub = Subscriber("/spinalcord/sternocleidomastoid_position",
								JointState)
		self.__messageFilter=ApproximateTimeSynchronizer([objSub,panTiltSub],
													10,
													0.5,
													allow_headerless=True)
		self.__messageFilter.registerCallback(self.__callback)

	def ddd(self,msg):
		diff = rospy.Time.now().to_sec() - msg.header.stamp.to_sec()
		rospy.logdebug("Image   "+str(msg.header.stamp.to_sec()))

	def eee(self, msg):
		diff = rospy.Time.now().to_sec() - msg.header.stamp.to_sec()
		rospy.logdebug("Pantilt "+str(msg.header.stamp.to_sec()))

	def debugCallback(self, objMsg, panTiltMsg):
		# pass
		rospy.logdebug("Time Diff between objMsg and panTiltMsg is "+str(objMsg.header.stamp.to_sec() - panTiltMsg.header.stamp.to_sec()))
		diff = rospy.Time.now().to_sec() - min(objMsg,panTiltMsg,key = lambda x:x.header.stamp.to_sec()).header.stamp.to_sec()
		rospy.logdebug("Time diff from current time is "+str(diff))

	def __callback(self, objMsg, panTiltMsg):
		self.debugCallback(objMsg, panTiltMsg)
		msg = self.__kinematicFunction(objMsg, panTiltMsg)
		self.publish(msg)

	def run(self):
		rospy.loginfo("Start kinematic_brain node.")
		while not rospy.is_shutdown():
			self.__loop()
			self.sleep()
		rospy.loginfo("Close kinematic_brain node.")

	def end(self):
		pass

def main():
	fileName = rospy.get_param("/vision_manager/vision_module_path", '')
	if fileName == '':
		module = VisionModule()
	else:
		print "get module"
		lobe_module = load_module(fileName)
		assert hasattr( lobe_module, 'kinematic_module' )
		module = lobe_module.kinematic_module

	try:
		node = KinematicBrain()
		node.initial(module)
		node.run()
	# except rospy.ROSInterruptException as e:
	except Exception as e:
		print "\n\n\n"
		print e
		print "\n\n\n"
	finally:
		node.end()
