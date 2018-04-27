import cv2
import numpy as np 

import rospy
import roslib
from message_filters import ApproximateTimeSynchronizer, Subscriber,TimeSynchronizer
from sensor_msgs.msg import CompressedImage

from cell.nodeCell import NodeBase
# from occipitalLobe import 	

import os
import sys
import imp

class cranialNerveII_monitor(NodeBase):
	def __init__(self):
		super(cranialNerveII_monitor, self).__init__("cranial_nerve_ii_monitor")
		self.__recieveImg = np.zeros((640,480,3), dtype=np.uint8)
		self.__visualizeFunction = lambda x,y:None
		self.__cranialAlive = False
		self.__occipitalAlive = False

	def initial(self,):
		self.__rosInitNode()
		self.rosInitSubscriber("/vision_manager/occipital_debug_topic", CompressedImage, self.__callback, buff_size=8*3*480*640)

	def __rosInitNode(self):
		self.rosInitNode()
		self.setFrequencyFromParam('/vision_manager/cranial_nerve_ii_frquency')

	def __callback(self, imgMsg):
		## Read buffer from data and convert it to image.
		print rospy.Time.now().secs - imgMsg.header.stamp.secs
		npArray = np.fromstring(imgMsg.data, dtype=np.uint8)
		self.__recieveImg = cv2.imdecode(npArray, 1)
		## get time stamp.
		timeStamp = imgMsg.header.stamp
		fontFace =  cv2.FONT_HERSHEY_PLAIN
		# cv2.putText(self.__recieveImg, str(timeStamp), (10,10), fontFace, 0.9, (0,0,0), 1)
		# cv2.putText(self.__recieveImg, str(timeStamp), (10,20), fontFace, 0.9, (255,255,255), 1)

	def run(self):
		while not rospy.is_shutdown():
			cv2.imshow("img", self.__recieveImg)
			cv2.waitKey(1)
			self.sleep()
		self.end()

	def end(self):
		cv2.destroyAllWindows()

def main():
	try:
		monitor = cranialNerveII_monitor()
		monitor.initial()
		monitor.run()
	except rospy.ROSInterruptException:
		pass
	finally:
		monitor.end()