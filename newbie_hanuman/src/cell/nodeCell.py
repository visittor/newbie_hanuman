#!/usr/bin/env python

import rospy
import roslib

VERBOSITY_LEVEL = {	"debug"	:	rospy.DEBUG,
					"info"	:	rospy.INFO,
					"warn"	:	rospy.WARN,
					"error"	:	rospy.ERROR,
					"fatal"	:	rospy.FATAL,}

class NodeBase(object):

	def __init__(self, nodeName, anonymous = True):
		self.__nameSpace = rospy.get_namespace()
		self.__nodeName = nodeName
		self.__verbosity = rospy.get_param(self.__nameSpace+"global_verbosity","debug").lower()
		self.__frequency = rospy.get_param(self.__nameSpace+"global_frequency",60)
		self.__queue_size = rospy.get_param(self.__nameSpace+"global_queue_size", 1)
		self.__anonymous = anonymous

	def setFrequency(self, hz):
		self.__frequency = hz
		self.rosInitRate()

	def setFrequencyFromParam(self, param):
		self.__frequency = rospy.get_param(param, self.__frequency)
		self.rosInitRate()

	def getParam(self, param, defualt = None):
		if defualt is None :
			return rospy.get_param(param)
		else:
			return rospy.get_param(param, defualt)

	def rosInitPublisher(self, name, msg, queue_size = None):
		queue_size = self.__queue_size if queue_size is None else queue_size
		self.__ros_pub = rospy.Publisher(name, msg, queue_size = queue_size)

	def rosInitSubscriber(self, name, msg, callback,queue_size = None, buff_size = 65536):
		queue_size = self.__queue_size if queue_size is None else queue_size
		rospy.Subscriber(name, msg, callback, queue_size=10, buff_size = buff_size)

	def rosInitService(self, name, service, handle):
		rospy.Service(name, service, handle)

	def rosInitRate(self):
		self.__rate = rospy.Rate(self.__frequency)

	def rosInitNode(self):
		rospy.init_node(self.__nodeName, anonymous = self.__anonymous, log_level = VERBOSITY_LEVEL[self.__verbosity])

	def publish(self, msg):
		self.__ros_pub.publish(msg)

	def sleep(self):
		self.__rate.sleep()

	def spin(self):
		rospy.spin()

	@property
	def nameSpace(self):
		return self.__nameSpace