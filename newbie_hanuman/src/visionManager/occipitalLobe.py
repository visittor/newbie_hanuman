#!/usr/bin/env python
import numpy as np 
import cv2

import rospy
import roslib
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

from cell.nodeCell import NodeBase
from brain.visionModule import VisionModule

import inspect
import os
import imp
import sys

class OccipitalLobeException( Exception ):
	pass

# class OccipitalLobe(NodeBase):

# 	def __init__(self):
# 		super(OccipitalLobe, self).__init__("occipital_lobe")
# 		self.__subscribeCallback = lambda x:x
# 		self.__serviceCallback = None
# 		self.__publisherMessageType = None

# 	def __subscribeCallbackWrapper(self, func):
# 		if len(inspect.getargspec(func)[0]) != 3:
# 			raise OccipitalLobeException( "Subscibe call back must have only one argument." )
# 		def subscribeCallback(msg):
# 			npArray = np.fromstring(msg.data, dtype=np.uint8)
# 			recieveImg = cv2.imdecode(npArray, 1)
# 			advertiseMsg = func(recieveImg, msg.header)
# 			self.publish(advertiseMsg)
# 		return subscribeCallback

# 	def set_subscribeCallback(self, func):
# 		self.__subscribeCallback = self.__subscribeCallbackWrapper(func)

# 	def set_publisherMessageType(self, msgType):
# 		self.__publisherMessageType = msgType

# 	def initial(self, occipital_lobe):
# 		self.set_subscribeCallback(occipital_lobe.subscribeCallback)
# 		self.set_publisherMessageType(occipital_lobe.publishMsgType)
# 		self.rosInitNode()
# 		self.rosInitPublisher( 	"/vision_manager/occipital_lobe_topic",
# 								self.__publisherMessageType)
# 		self.rosInitSubscriber(	"/vision_manager/cranial_nerve_ii_topic",
# 								CompressedImage,
# 								self.__subscribeCallback)

# 	def run(self):
# 		self.spin()

class OccipitalLobe(NodeBase):

	def __init__(self):
		super(OccipitalLobe, self).__init__("occipital_lobe")
		self.__subscribeCallback = lambda x:x
		self.__serviceCallback = None
		self.__publisherMessageType = None
		self.__visualizeFunc = lambda x,y:x
		cameraID = self.getParam('/vision_manager/cameraID', 0)
		self.__cap = cv2.VideoCapture(cameraID)

	def set_subscribeCallback(self, func):
		self.__subscribeCallback = func

	def set_publisherMessageType(self, msgType):
		self.__publisherMessageType = msgType

	def set_visualizeFunction(self, func):
		if len(inspect.getargspec(func)[0]) != 3:
			raise OccipitalLobeException( "Subscibe call back must have three argument." )
		self.__visualizeFunc = func

	def initial(self, module):
		self.set_subscribeCallback(module.ImageProcessingFunction)
		self.set_publisherMessageType(module.objectsMsgType)
		self.set_visualizeFunction(module.visualizeFunction)
		self.rosInitNode()
		self.rosInitPublisher( 	"/vision_manager/occipital_lobe_topic",
								self.__publisherMessageType)
		self.__rospub = rospy.Publisher("/vision_manager/cranial_nerve_ii_topic",
								CompressedImage,
								queue_size = 1)
		self.setFrequencyFromParam("/vision_manager/cranial_nerve_ii_frquency")

	def __createCompressedImage(self, img, stamp):
		msg = CompressedImage()
		msg.header.stamp = stamp
		msg.format = "jpeg"
		msg.data = np.array(cv2.imencode(".jpg", img)[1]).tostring()
		return msg

	def publish(self, imgMsg, objectMsg):
		super(OccipitalLobe, self).publish(objectMsg)
		self.__rospub.publish(imgMsg)

	def run(self):
		rospy.loginfo("Start OccipitalLobe node.")
		while not rospy.is_shutdown():
			ret, img = self.__cap.read()
			stamp = rospy.Time.now()
			header = Header(stamp = stamp)
			objectMsg = self.__subscribeCallback(img, header)
			self.__visualizeFunc(img, objectMsg)
			imgMsg = self.__createCompressedImage(img, stamp)
			self.publish(imgMsg, objectMsg)
			self.sleep()

	def end(self):
		self.__cap.release()
		rospy.loginfo("Close OccipitalLobe node.")

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

	node = OccipitalLobe()
	try:
		node.initial(module)
		node.run()
	except rospy.ROSInterruptException as e:
		rospy.logwarn(str(e))
	finally:
		node.end()