import cv2
import numpy as np 

import rospy
import roslib

from sensor_msgs.msg import CompressedImage

from cell.nodeCell import NodeBase

class cranialNerveII_monitor(NodeBase):
	def __init__(self):
		super(cranialNerveII_monitor, self).__init__("cranial_nerve_ii_monitor")
		self.rosInitSubscriber("/vision_manager/cranial_nerve_ii_topic", CompressedImage, self.__callback)
		self.rosInitNode()
		self.__recieveImg = np.zeros((640,480,3), dtype=np.uint8)
		self.setFrequencyFromParam('/vision_manager/cranial_nerve_ii_frquency')

	def __rosInitNode(self):
		
		self.ros_sub = rospy.Subscriber("/vision_manager/cranial_nerve_ii_topic", 
										CompressedImage, self.__callback, 
										queue_size = 10)
		rospy.init_node("cranial_nerve_ii_monitor", 
						anonymous = True)

		self.__recieveImg = np.zeros((640,480,3), dtype=np.uint8)
		frquency = rospy.get_param('/vision_manager/cranial_nerve_ii_frquency', 60)
		self.__rate = rospy.Rate(frquency)

	def __callback(self, data):
		## Read buffer from data and convert it to image.
		npArray = np.fromstring(data.data, dtype=np.uint8)
		self.__recieveImg = cv2.imdecode(npArray, 1)

		## get time stamp.
		timeStamp = data.header.stamp
		fontFace =  cv2.FONT_HERSHEY_PLAIN
		cv2.putText(self.__recieveImg, str(timeStamp), (10,10), fontFace, 0.9, (0,0,0), 1)
		cv2.putText(self.__recieveImg, str(timeStamp), (10,20), fontFace, 0.9, (255,255,255), 1)

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
		monitor.run()
	except rospy.ROSInterruptException:
		pass
	finally:
		monitor.end()