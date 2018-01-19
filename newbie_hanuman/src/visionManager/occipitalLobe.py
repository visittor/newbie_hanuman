#!/usr/bin/env python
import numpy as np 
import cv2

import rospy
import roslib
from sensor_msgs.msg import CompressedImage

from cell.nodeCell import NodeBase

import inspect
import os
import imp
import sys

class OccipitalLobeException( Exception ):
	pass

class OccipitalLobe(NodeBase):

	def __init__(self):
		super(OccipitalLobe, self).__init__("occipital_lobe")
		self.__subscribeCallback = lambda x:x
		self.__serviceCallback = None
		self.__publisherMessageType = None

	def __subscribeCallbackWrapper(self, func):
		if len(inspect.getargspec(func)[0]) != 3:
			raise OccipitalLobeException( "Subscibe call back must have only one argument." )
		def subscribeCallback(msg):
			npArray = np.fromstring(msg.data, dtype=np.uint8)
			recieveImg = cv2.imdecode(npArray, 1)
			advertiseMsg = func(recieveImg, msg.header)
			self.publish(advertiseMsg)
		return subscribeCallback

	def set_subscribeCallback(self, func):
		self.__subscribeCallback = self.__subscribeCallbackWrapper(func)

	def set_publisherMessageType(self, msgType):
		self.__publisherMessageType = msgType

	def initial(self, occipital_lobe):
		self.set_subscribeCallback(occipital_lobe.subscribeCallback)
		self.set_publisherMessageType(occipital_lobe.publishMsgType)
		self.rosInitNode()
		self.rosInitPublisher( 	"/vision_manager/occipital_lobe_topic",
								self.__publisherMessageType)
		self.rosInitSubscriber(	"/vision_manager/cranial_nerve_ii_topic",
								CompressedImage,
								self.__subscribeCallback)

	def run(self):
		self.spin()

class OccipitalLobeSetup(object):

	def __init__(self):
		self.publishMsgType = CompressedImage

	def subscribeCallback(self,img, header):
		'''Override this function.'''
		msg = CompressedImage()
		msg.header.stamp = header.stamp
		msg.format = "jpeg"
		msg.data = np.array(cv2.imencode(".jpg", img)[1]).tostring()
		return msg

	def visualizeFunction(self, img, msg):
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
		lobe = OccipitalLobeSetup()
	else:
		print "get module"
		lobe_module = load_occipital_lobe(fileName)
		assert hasattr( lobe_module, 'occipital_lobe' )
		lobe = lobe_module.occipital_lobe

	node = OccipitalLobe()
	try:
		node.initial(lobe)
		node.run()
	except rospy.ROSInterruptException as e:
		rospy.logwarn(str(e))
		pass