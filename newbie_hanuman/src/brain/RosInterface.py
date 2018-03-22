#!/usr/bin/env python

import rospy

from Queue import PriorityQueue, Queue

class RosInterface(object):

	class __Publisher(object):

		def __init__(self, publisher, msgType):
			self.__publisher = publisher
			self.__msgType = msgType
			self.__msg = None

		def setCommand(self, *arg, **kwarg):
			self.__msg = self.__msgType(*arg, **kwarg)

		def doCurrentCommand(self):
			self.__publisher.publish(self.__msg)

		def doThisCommand(self, msg, *arg, **kwarg):
			self.setMsg(*arg, **kwarg)
			self.sendCurrentMsg()

	def __init__(self):
		self.__publisherInterface = []
		self.__serviceInterface = []
		self.__subscriberInterface = []

	def __subscribeCallBackGenerator(self, varName, method):
		def callBack(self, msg):
			print varName
			setattr(self, varName, msg)

		def callBackQueue(self, msg):
			getattr(self, varName).put(msg)

		assert method=='recent' or method=='queue' or method=='priorityQueue'
		if method == 'recent':
			return callBack
		elif method == 'queue'or method == 'priorityQueue':
			return callBackQueue

	def interfaceWithPublisher(self, topic, msg, varName, store_type='recent', initialVal = None,queue_size = 1):
		assert store_type=='recent' or store_type=='queue' or store_type=="priorityQueue"
		assert varName not in self.__subscriberInterface
		if store_type == 'recent':
			setattr(self, varName, initialVal)
			func = lambda x: self.__subscribeCallBackGenerator(varName, store_type)(self, x)
			setattr(self, 
					varName+"Callback", 
					func)

		elif store_type == 'queue':
			setattr(self, varName, Queue())
			func = lambda x: self.__subscribeCallBackGenerator(varName, store_type)(self, x)
			setattr(self, 
					varName+"Callback",
					func)

		elif store_type == 'priorityQueue':
			setattr(self, varName, PriorityQueue)
			func = lambda x: self.__subscribeCallBackGenerator(varName, store_type)(self, x)
			setattr(self, 
					varName+"Callback", 
					func)
		print msg
		rospy.Subscriber(topic, msg,
						 getattr(self, varName+"Callback") ,
						 queue_size=queue_size)

		self.__subscriberInterface.append(varName)

	def interfaceWithService(self, serviceName,  serviceClass, funcName):
		assert funcName not in self.__serviceInterface
		setattr(self, funcName, rospy.ServiceProxy(serviceName, serviceClass))
		self.__serviceInterface.append(funcName)

	def interfaceWithSubscriber(self, topic, msg, varName,queue_size = 1):
		assert varName not in self.__publisherInterface
		pub = rospy.Publisher(topic, msg, queue_size = queue_size)
		setattr(self, varName, self.__Publisher(pub, msg) )
		self.__publisherInterface.append(varName)

