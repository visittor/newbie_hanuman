#!/usr/bin/env python
##
import cv2
import numpy as np 

import rospy
import roslib

from sensor_msgs.msg import CompressedImage

from cell.nodeCell import NodeBase

class CranialNerveII(NodeBase):
	"""docstring for CranialNerveII"""
	def __init__(self):
		# self.ros_pub = rospy.Publisher("/vision_manager/cranial_nerve_ii_topic", CompressedImage, queue_size = 1)
		# rospy.init_node("cranial_nerve_ii", anonymous = True)
		# cameraID = rospy.get_param('/vision_manager/cameraID', 0)
		# frequency = rospy.get_param('/master_frequency', 60)
		# frequency = rospy.get_param('/vision_manager/cranial_nerve_ii_frquency', frequency)
		# self.__rate = rospy.Rate(frequency)
		super(CranialNerveII, self).__init__("cranial_nerve_ii")
		self.rosInitNode()
		self.rosInitPublisher("/vision_manager/cranial_nerve_ii_topic", CompressedImage, queue_size = 1)
		self.setFrequencyFromParam("/vision_manager/cranial_nerve_ii_frquency")
		cameraID = self.getParam('/vision_manager/cameraID', 0)
		self.__cap = cv2.VideoCapture(cameraID)

	def publish(self, img):
		msg = CompressedImage()
		msg.header.stamp = rospy.Time.now()
		msg.format = "jpeg"
		msg.data = np.array(cv2.imencode(".jpg", img)[1]).tostring()
		super(CranialNerveII, self).publish(msg)
		# print "published"

	def run(self):
		rospy.loginfo("Start cranial_nerve_ii node.")
		while not rospy.is_shutdown():
			ret, img = self.__cap.read()
			self.publish(img)
			self.sleep()
		rospy.loginfo("Close cranial_nerve_ii node.")

	def end(self):
		self.__cap.release()

def main():
	try:
		canialNerve2 = CranialNerveII()
		canialNerve2.run()
	except rospy.ROSInterruptException:
		canialNerve2.end()
	finally:
		canialNerve2.end()
