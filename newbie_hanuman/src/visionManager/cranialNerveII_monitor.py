import cv2
import numpy as np 

import rospy
import roslib
from message_filters import ApproximateTimeSynchronizer, Subscriber,TimeSynchronizer
from sensor_msgs.msg import CompressedImage

from cell.nodeCell import NodeBase
from occipitalLobe import OccipitalLobeSetup

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

	def initial(self, occipital_lobe):
		self.__rosInitNode()
		self.rosInitSubscriber("/vision_manager/cranial_nerve_ii_topic", CompressedImage, self.__debug1)
		while not self.__cranialAlive:
			pass
		self.rosInitSubscriber("/vision_manager/occipital_lobe_topic",occipital_lobe.publishMsgType, self.__debug2)
		while not self.__occipitalAlive:
			pass

		self.__rosInitFilter(occipital_lobe)

	def __rosInitNode(self):
		self.rosInitNode()
		self.setFrequencyFromParam('/vision_manager/cranial_nerve_ii_frquency')

	def __rosInitFilter(self, occipital_lobe):
		imgSub = Subscriber("/vision_manager/cranial_nerve_ii_topic", CompressedImage)
		objectSub = Subscriber("/vision_manager/occipital_lobe_topic", occipital_lobe.publishMsgType)
		# fs = ApproximateTimeSynchronizer([imgSub,objectSub],10,0.5,False)
		fs = TimeSynchronizer([imgSub,objectSub], 2048)
		self.setVisualizeFunction(occipital_lobe.visualizeFunction)
		fs.registerCallback(self.__callback)

	def __debug1(self, msg):
		self.__cranialAlive = True
		rospy.logdebug("cranial time"+str(msg.header.stamp.secs))

	def __debug2(self, msg):
		self.__occipitalAlive = True
		rospy.logdebug("occipit time"+str(msg.header.stamp.secs))

	def setVisualizeFunction(self, func):
		self.__visualizeFunction = func

	def __callback(self, imgMsg, objectMsg):
		## Read buffer from data and convert it to image.
		print rospy.Time.now().secs - imgMsg.header.stamp.secs
		npArray = np.fromstring(imgMsg.data, dtype=np.uint8)
		self.__recieveImg = cv2.imdecode(npArray, 1)

		## get time stamp.
		timeStamp = imgMsg.header.stamp
		fontFace =  cv2.FONT_HERSHEY_PLAIN
		cv2.putText(self.__recieveImg, str(timeStamp), (10,10), fontFace, 0.9, (0,0,0), 1)
		cv2.putText(self.__recieveImg, str(timeStamp), (10,20), fontFace, 0.9, (255,255,255), 1)
		self.__visualizeFunction(self.__recieveImg, objectMsg)

	def run(self):
		while not rospy.is_shutdown():
			cv2.imshow("img", self.__recieveImg)
			cv2.waitKey(1)
			self.sleep()
		self.end()

	def end(self):
		cv2.destroyAllWindows()

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
		lobe_module = load_occipital_lobe(fileName)
		assert hasattr( lobe_module, 'occipital_lobe' )
		lobe = lobe_module.occipital_lobe

	try:
		monitor = cranialNerveII_monitor()
		monitor.initial(lobe)
		monitor.run()
	except rospy.ROSInterruptException:
		pass
	finally:
		monitor.end()