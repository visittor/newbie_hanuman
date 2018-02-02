#! /usr/bin/env python
import numpy as np 
import cv2

from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Empty

class VisionModule(object):

	def __init__(self):
		self.objectsMsgType = CompressedImage

	def ImageProcessingFunction(self,img, header):
		'''Override this function.'''
		msg = CompressedImage()
		msg.header.stamp = header.stamp
		msg.format = "jpeg"
		msg.data = np.array(cv2.imencode(".jpg", img)[1]).tostring()
		return msg

	def visualizeFunction(self, img, msg):
		pass

class KinematicModule(object):

	def __init__(self):
		self.objectsMsgType = CompressedImage
		self.posDictMsgType = Empty
		self.cameraFOV = rospy

	def kniematicCalculation(self, jointState, objMsg):
		return Empty()