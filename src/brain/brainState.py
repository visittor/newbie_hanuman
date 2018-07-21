#!/usr/bin/env python
import rospy

from sensor_msgs.msg import JointState

class ChangeBrainState(Exception):
	def __init__(self, newStateName, *arg, **kwarg):
		self.newStateName = str(newStateName)
		self.arg = arg
		self.kwarg = kwarg

	def __str__(self):
		return str(self.newStateName)

class FSMBrainState(object):

	def __init__(self, name):
		self.name = str(name)
		self.subBrains = {}
		self.__currentSubBrain = None
		self.__currentSubBrainName = None
		self.__previousSubBrain = None
		self.__previousSubBrainName = None
		self.__subBrainFirstStep = True
		self.__firstsubBrain = None
		self.__firstsubBrainName = None

	def initializeBrain(self, rosInterface):
		self.rosInterface = rosInterface
		self.initialSubBrain()

	def initialSubBrain(self):
		for name,sub in self.subBrains.items():
			sub.initializeBrain(self.rosInterface)

	def end(self):
		self.endSubBrain()

	def endSubBrain(self):
		for name,sub in self.subBrains.items():
			sub.end()

	def __setCurrentSubBrain(self, name):
		self.__currentSubBrain = self.subBrains[name]
		self.__currentSubBrainName = name
		self.__subBrainFirstStep = True

	def __setPreviousSubBrain(self, name):
		self.__previousSubBrain = self.subBrains[name]
		self.__previousSubBrainName = name

	def addSubBrain(self, subBrain, name=None):
		if name is None:
			name = subBrain.name
		assert not self.subBrains.has_key(name), "{} already exist.".format(str(name))
		self.subBrains[name] = subBrain
		self.__setCurrentSubBrain(name)
		# self.__currentSubBrain = self.subBrains[name]
		# self.__currentSubBrainName = name

	def setFirstSubBrain(self, name):
		assert self.subBrains.has_key(name), "{} sub brain is not exist".format(name)
		self.__setCurrentSubBrain(name)
		self.__firstsubBrain = self.subBrains[name]
		self.__firstsubBrainName = name
		# self.__currentSubBrain = self.subBrains[name]

	def __visitSubBrain(self):
		if self.__currentSubBrain is None:
			return
		try:
			if self.__subBrainFirstStep:
				self.__currentSubBrain.firstUpdate()
				self.__subBrainFirstStep = False
			else:
				self.__currentSubBrain.update()
		except ChangeBrainState as e:
			newStateName = e.newStateName
			if not self.subBrains.has_key(newStateName):
				rospy.logerr("{} do not have sub brain name {}. What the fuck should I do next. Fuck you who wrote this code!!!!!!!!!!.".format(self.name, newStateName))
				return
			# self.__previousSubBrain = self.__currentSubBrain
			# self.__previousSubBrainName = self.__currentSubBrainName
			# self.__currentSubBrain = self.subBrains[newStateName]
			# self.__currentSubBrainName = newStateName
			self.__setPreviousSubBrain(self.__currentSubBrainName)
			self.__setCurrentSubBrain(newStateName)
			self.__subBrainFirstStep = True

	def SignalChangeSubBrain(self, name):
		raise ChangeBrainState(name)

	# def SignalChangeToPreviousSubBrain(self):
	# 	raise ChangeBrainState(self.__previousSubBrainName)

	def ChangeSubBrain(self, newStateName):
		if not self.subBrains.has_key(newStateName):
			rospy.logerr("{} do not have sub brain name {}. What the fuck should I do next. Fuck you who wrote this code!!!!!!!!!!.".format(self.name, newStateName))
			return
		if self.__currentSubBrainName == newStateName:
			return
		# self.__previousSubBrain = self.__currentSubBrain
		# self.__previousSubBrainName = self.__currentSubBrainName
		# self.__currentSubBrain = self.subBrains[newStateName]
		# self.__currentSubBrainName = newStateName
		self.__setPreviousSubBrain(self.__currentSubBrainName)
		self.__setCurrentSubBrain(newStateName)
		self.__subBrainFirstStep = True

	def resetState(self):
		self.__setCurrentSubBrain(self.__firstsubBrainName)
		self.__previousSubBrain = None
		self.__previousSubBrainName = None

	def firstStep(self):
		pass

	def firstUpdate(self):
		self.firstStep()
		self.__visitSubBrain()

	def step(self):
		pass

	def update(self):
		self.step()
		self.__visitSubBrain()

	@property
	def prevSubBrainName(self):
		return self.__previousSubBrainName

	@property
	def currSubBrainName(self):
		return self.__currentSubBrainName

	## ROS heper function.
	def getJointStateFromList(self, jointName, jointPos, jointVel, effort):
		assert len(jointName) == len(jointPos)
		return JointState(	name=jointName,
							position=jointPos,
							velocity=jointVel,
							effort=effort)