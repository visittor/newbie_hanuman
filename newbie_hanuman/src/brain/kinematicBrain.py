#!/usr/bin/env python

import rospy

from message_filters import Subscriber,ApproximateTimeSynchronizer
from sensor_msgs.msg import JointState

from cell.nodeCell import NodeBase
from brain.visionModule import VisionModule

import inspect
import os
import imp
import sys

class KinematicBrain(NodeBase):

	def __init__(self):
		super(KinematicBrain, self).__init__("kinematic_brain")
		self.__occipitalMessageType = None
		self.__posDictMessageType = None
		self.__messageFilter = None

		self.__kinematicFunction = lambda x:x
		
	def initial(self, module):
		self.setOccipitalMessageType(module.objectsMsgType)
		self.setPosDictMessageType(module.posDictMsgType)
		self.__rosInit()

	def setOccipitalMessageType(self, msgType):
		self.__occipitalMessageType = msgType

	def setPosDictMessageType(self, msgType):
		self.__posDictMessageType = msgType

	def setKinematicFunction(self, func):
		self.__kinematicFunction = func

	def __rosInit(self):
		self.rosInitNode()
		self.__initMessageFilter()

	def __initMessageFilter(self):
		objSub = Subscriber("/vision_manager/occipital_lobe_topic",
							self.__occipitalMessageType)
		panTiltSub = Subscriber("/spinalcord/sternocleidomastoid_position",
								JointState)
		self.__messageFilter=ApproximateTimeSynchronizer([objSub,panTiltSub],
													1028,
													20,
													 allow_headerless=False)
		self.__messageFilter.registerCallback(self.callback)

	# 	rospy.Subscriber("/vision_manager/occipital_lobe_topic", 
	# 					 self.__occipitalMessageType, 
	# 					 self.debug1, 
	# 					 queue_size=1)
	# 	rospy.Subscriber("/spinalcord/sternocleidomastoid_position",
	# 					JointState,
	# 					self.debug2,
	# 					queue_size = 1)

	# def debug1(self, msg):
	# 	rospy.logdebug("occi"+str(msg.header.stamp.secs))

	# def debug2(self, msg):
	# 	rospy.logdebug("pant"+str(msg.header.stamp.secs))

	def callback(self, objMsg, panTiltMsg):
		rospy.logdebug("Time Diff between objMsg and panTiltMsg is "+str(objMsg.header.stamp.secs - panTiltMsg.header.stamp.secs))
		diff = rospy.Time.now().secs - min(objMsg,panTiltMsg,key = lambda x:x.header.stamp.secs).header.stamp.secs
		rospy.logdebug("Time diff from current time is"+str(diff))

	def run(self):
		rospy.spin()

	def end(self):
		pass

def load_occipital_lobe(fileName):
	# open file
	fileObj = file( fileName )

	# create new module
	moduleName = '.'.join( os.path.abspath( fileName ).split( '.' )[:-1] )
	newModule = imp.new_module( moduleName )
	
	# execute fileData in this environment
	oldSysPath = sys.path
	sys.path = [ os.path.dirname( os.path.abspath(fileName) ) ] + sys.path
	exec fileObj in newModule.__dict__
	sys.path = oldSysPath
	
	#	return new module
	return newModule

def main():
	fileName = rospy.get_param("/vision_manager/vision_module_path", '')
	if fileName == '':
		module = VisionModule()
	else:
		print "get module"
		lobe_module = load_occipital_lobe(fileName)
		assert hasattr( lobe_module, 'vision_module' )
		module = lobe_module.vision_module

	try:
		node = KinematicBrain()
		node.initial(module)
		node.run()
	except rospy.ROSInterruptException:
		pass
	finally:
		node.end()
