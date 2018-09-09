#!/usr/bin/env python

import rospy

from Queue import PriorityQueue, Queue

import actionlib
from actionlib_msgs.msg import GoalStatus

class Publisher(object):

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

class Service( rospy.ServiceProxy ):
	'''
	Proxy for ros service
	'''
	def __init__( self, serviceName,  serviceClass, ):
		super(Service, self).__init__(serviceName, serviceClass)

		self.__name = serviceName

	def __call__( self, *args, **kwargs ):
		try:
			super(Service, self).__call__( *args, **kwargs )
		except rospy.ServiceException as exc:
			rospy.logwarn("Service did not process request: " + str(exc))

class ActionClient(object):

	def __init__(self, client, actionType, actionGoal):
		self.__client = client
		self.__actType = actionType
		self.__actGoal = actionGoal

		self.__feedBack = None
		self.__result = None

		self.__actFinish = True
		self.__actSuccess = None
		self.__inProgress = False

	def __feedBackCB(self, feed_back):
		self.__feedBack = feed_back
		if self.__client.get_state() == GoalStatus.LOST:
			self.cancelGoal()
			self.__actFinish = True
			self.__actSuccess = False
			self.__inProgress = False
			self.__result = None

	def __doneCB(self, goalStatus, result):
		if goalStatus == GoalStatus.SUCCEEDED:
			self.__actSuccess = True
		else:
			self.__actSuccess = False

		self.__actFinish = True
		self.__inProgress = False
		self.__result = result

	def setGoal(self, **kwarg):
		goalMsg = self.__actGoal(**kwarg)
		self.__inProgress = True
		self.__actFinish = False
		self.__actSuccess = None
		self.__client.send_goal(goalMsg, feedback_cb = self.__feedBackCB,
										done_cb = self.__doneCB)

	def cancelGoal(self):
		self.__client.cancel_goal()

	def waitForServer(self):
		self.__client.wait_for_server()

	@property
	def feedBack(self):
		return self.__feedBack

	@property
	def result(self):
		return self.__result

	@property
	def isFinish(self):
		return self.__actFinish

	@property
	def isSuccess(self):
		return slf.__actSuccess

	@property
	def isInProgress(self):
		return self.__inProgress

class RosInterface(object):

	def __init__(self):
		self.__publisherInterface = []
		self.__serviceInterface = []
		self.__subscriberInterface = []
		self.__actionInterface = []
		self.__varName = []

	def __subscribeCallBackGenerator(self, varName, method):
		def callBack(self, msg):
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
		assert varName not in self.__varName, varName+" already exist please choose a new name."

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
		self.__varName.append(varName)

	def interfaceWithService(self, serviceName,  serviceClass, funcName):
		assert funcName not in self.__serviceInterface
		assert funcName not in self.__varName, varName+" already exist please choose a new name."

		setattr(self, funcName, Service( serviceName, serviceClass ) )
		self.__serviceInterface.append(funcName)
		self.__varName.append(funcName)

	def interfaceWithSubscriber(self, topic, msg, varName,queue_size = 1):
		assert varName not in self.__publisherInterface
		assert varName not in self.__varName, varName+" already exist please choose a new name."

		pub = rospy.Publisher(topic, msg, queue_size = queue_size)
		setattr(self, varName, Publisher(pub, msg) )
		self.__publisherInterface.append(varName)
		self.__varName.append(varName)

	def interfaceWithAction(self, ns, actionType, goalType, varName, feed_cb=None):
		assert varName not in self.__actionInterface
		assert varName not in self.__varName, varName+" already exist please choose a new name."
		client = actionlib.SimpleActionClient(ns, actionType)
		action = ActionClient(client, actionType, goalType)
		setattr(self, varName, action)
		self.__varName.append(varName)
		self.__actionInterface.append(varName)

	def waitAllInterface( self, timeout ):
		'''
		Wait until all interface is ready
		argument:
			timeout	:	timeout
		'''
		## wait for service
		[  getattr(self,service).wait_for_service( timeout ) for service in self.__serviceInterface ]