#!/usr/bin/env python
import rospy

from cell.nodeCell import NodeBase
from SuchartInterface import SuchartRosInterface
from brain.brainState import FSMBrainState
from utility.utility import load_module

class SuchartSkull(NodeBase):

	def __init__(self):
		super(SuchartSkull, self).__init__("frontal_cortex")
		self.__main_brain = None
		self.rosInterface = SuchartRosInterface()
		self.__brainFirstStep = True
		self.rosInitNode()
		self.setFrequencyFromParam("/brain/brain_frequency")

	def setMainBrain(self, main_brain):
		self.__main_brain = main_brain

	def initial(self, main_brain):
		self.setMainBrain(main_brain)
		self.__main_brain.initializeBrain(self.rosInterface)

	def __firstStep(self):
		self.__main_brain.firstUpdate()

	def run(self):
		rospy.loginfo("Start frontal_cortex node")
		while not rospy.is_shutdown():
			if self.__brainFirstStep:
				self.__firstStep()
				self.__brainFirstStep = False
				continue
			self.__main_brain.update()
			self.sleep()

	def end(self):
		self.__main_brain.end()
		rospy.loginfo("Close frontal_cortex node.")

def main():
	fileName = rospy.get_param("/brain/brain_module_path", '')
	if fileName == '':
		module = FSMBrainState("Default")
	else:
		print "get module"
		lobe_module = load_module(fileName)
		assert hasattr( lobe_module, 'main_brain')
		module = lobe_module.main_brain

	node = SuchartSkull()

	try:
		node.initial(module)
		node.run()
	except rospy.ROSInterruptException as e:
		rospy.logwarn(str(e))
	finally:
		node.end()