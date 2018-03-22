#!/usr/bin/env python
import rospy

class ChangeBrainState(Exception):
	def __init__(self, newStateName, *arg, **kwarg):
		self.newStateName = str(newStateName)
		self.arg = arg
		self.kwarg = kwarg

	def __str__(self):
		return str(self.newStateName)

class FSMBrainState():

	def __init__(self, name):
		self.name = str(name)
		self.__subBrains = {}
		self.__currentSubBrain = None
		self.__currentSubBrainName = None
		self.__previousSubBrain = None
		self.__previousSubBrainName = None
		self.__subBrainFirstStep = True

	def initializeBrain(self, rosInterface):
		self.rosInterface = rosInterface
		self.initial_subbrain()

	def initialSubBrain(self):
		for name,sub in self.__subBrains.items:
			sub.initialize_brain(self.rosInterface)

	def addSubBrain(self, subBrain, name=None):
		if name is None:
			name = subBrain.name
		assert not self.__subBrains.has_key(name), "{} already exist.".format(str(name))
		self.__subBrains[name] = subBrain

	def setFirstSubBrain(self, name):
		assert self.__subBrains.has_key(name), "{} sub brain is not exist".format(name)
		self.__currentSubBrain = self.__subBrains[name]

	def __visitSubBrain(self):
		try:
			if self.__firstStep:
				self.__currentSubBrain.firstUpdate()
				self.__subBrainFirstStep = False
			else:
				self.__currentSubBrain.update()
		except ChangeBrainState as e:
			newStateName = e.newStateName
			if not self.__subBrains.has_key(newStateName):
				rospy.logerr("{} do not have sub brain name {}. What the fuck should I do next. Fuck you who wrote this code!!!!!!!!!!.".format(self.name, newStateName))
				return
			self.__previousSubBrain = self.__currentSubBrain
			self.__previousSubBrainName = self.__currentSubBrainName
			self.__currentSubBrain = self.__subBrains[newStateName]
			self.__currentSubBrainName = newStateName
			self.__subBrainFirstStep = True

	def ChangeSubBrain(self, name):
		if not self.__subBrains.has_key(name):
			rospy.logfatal("{} do not have sub brain name {}. I will not change sub brain for you. Fuck you who wrote this code!!!!!!!!!!.".format(self.name, newStateName))
			return
		raise ChangeBrainState(name)

	def ChangeToPreviousSubBrain(self):
		raise ChangeBrainState(self.__previousSubBrainName)

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