#!/usr/bin/env python
import numpy as np 
import cv2

import rospy
import roslib
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Header

from cell.nodeCell import NodeBase
from visionModule import VisionModule
from utility.utility import load_module

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
		self.__img = None
		self.__pantiltMsg = None
		self.__subPantilt = False

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
		self.__subPantilt = module.subscribePantiltTopic
		self.rosInitNode()
		self.rosInitPublisher( 	"/vision_manager/occipital_lobe_topic",
								self.__publisherMessageType,
								queue_size = 1)
		self.__rospub = rospy.Publisher("/vision_manager/occipital_debug_topic",
								CompressedImage,
								queue_size = 1)
		self.rosInitSubscriber("/vision_manager/cranial_nerve_ii_topic",
								 CompressedImage, 
								 self.__receiveImage,
								 queue_size = 1)
		if self.__subPantilt:
			rospy.Subscriber("/spinalcord/sternocleidomastoid_position", 
							 JointState, 
							 callback, 
							 queue_size=queue_size)
		self.setFrequencyFromParam("/vision_manager/cranial_nerve_ii_frquency")

	def __receiveImage(self, msg):
		npArray = np.fromstring(msg.data, dtype=np.uint8)
		self.__img = cv2.imdecode(npArray, 1)

	def __pantiltCallback(self, msg):
		self.__pantiltMsg = msg

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
			# ret, img = self.__cap.read()
			if self.__img is None:
				self.sleep()
				continue
			img = self.__img.copy()
			stamp = rospy.Time.now()
			header = Header(stamp = stamp)

			if self.__subPantilt:
				objectMsg = self.__subscribeCallback(img, 
													 header,
													 self.__pantiltMsg)
			else:
				objectMsg = self.__subscribeCallback(img, 
													 header)

			self.__visualizeFunc(img, objectMsg)
			imgMsg = self.__createCompressedImage(img, stamp)
			self.publish(imgMsg, objectMsg)
			self.sleep()

	def end(self):
		# self.__cap.release()
		rospy.loginfo("Close OccipitalLobe node.")

def main():
	fileName = rospy.get_param("/vision_manager/vision_module_path", '')
	if fileName == '':
		module = VisionModule()
	else:
		print "get module"
		lobe_module = load_module(fileName)
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