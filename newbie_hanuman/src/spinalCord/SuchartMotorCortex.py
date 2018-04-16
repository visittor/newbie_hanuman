#!/usr/bin/env python
import numpy as np

from SuchartBase import SuchartInterface
from newbie_hanuman.msg import SuchartFeedback, SuchartAction, SuchartResult

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from sensor_msgs.msg import JointState

from cell.nodeCell import NodeBase

import time
import serial

class SuchartMotorCortex(NodeBase):

	def __init__(self):
		super(SuchartMotorCortex, self).__init__("motor_cortex")

		joint1_min = self.getParam(	self.nameSpace+"motor_cortex/joint1_min",
									0)
		joint1_max = self.getParam(	self.nameSpace+"motor_cortex/joint1_max",
									65535)

		joint2_min = self.getParam(	self.nameSpace+"motor_cortex/joint2_min",
									0)
		joint2_max = self.getParam(	self.nameSpace+"motor_cortex/joint2_max",
									65535)

		joint3_min = self.getParam(	self.nameSpace+"motor_cortex/joint3_min",
									0)
		joint3_max = self.getParam(	self.nameSpace+"motor_cortex/joint3_max",
									65535)

		joint4_min = self.getParam(	self.nameSpace+"motor_cortex/joint4_min",
									0)
		joint4_max = self.getParam(	self.nameSpace+"motor_cortex/joint4_max",
									65535)

		joint5_min = self.getParam(	self.nameSpace+"motor_cortex/joint5_min",
									0)
		joint5_max = self.getParam(	self.nameSpace+"motor_cortex/joint5_max",
									65535)

		joint6_min = self.getParam(	self.nameSpace+"motor_cortex/joint6_min",
									0)
		joint6_max = self.getParam(	self.nameSpace+"motor_cortex/joint6_max",
									65535)

		self.__timeout = self.getParam(	self.nameSpace+"motor_cortex/timeout",
										0.1)

		self.__comPort = self.getParam(	self.nameSpace+"motor_cortex/comport",
										"/dev/pts/20")
		self.__baudrate = self.getParam(self.nameSpace+"motor_cortex/baudrate",
										9600)
		self.__rts = self.getParam(self.nameSpace+"motor_cortex/rts", 1)
		self.__dtr = self.getParam(self.nameSpace+"motor_cortex/dtr", 1)

		self.__suchartStatus = None
		self.__goalPosition = None
		self.__gripperCommand = None

		self.rosInitNode()

		self.__serial = serial.Serial()
		self.__connectSerialPort()

		self.__suchartInterface = SuchartInterface(	self.__serial, 
													[joint1_min, joint1_max],
													[joint2_min, joint2_max], 
													[joint3_min, joint3_max],
													[joint4_min, joint4_max], 
													[joint5_min, joint5_max], 
													[joint6_min, joint6_max],
													timeout = self.__timeout)

		self.setFrequency(5)
		self.__rate = self.getParam("/spinal_cord/suchart_action_frequency",5)
		self.__rosInitSubPubAction()


	def __connectSerialPort(self):
		if self.__serial.is_open:
			self.__serial.close()
		self.__serial.port = str(self.__comPort)
		self.__serial.baudrate = int(self.__baudrate)
		self.__serial.setDTR(self.__dtr)
		self.__serial.setRTS(self.__rts)

		try:
			self.__serial.open()
		except serial.SerialException as e:
			rospy.logwarn(str(e))
		except IOError:
			self.__serial=serial.Serial(self.__comPort, self.__baudrate, rtscts=True, dsrdtr=True)
		time.sleep(0.1)
		if not self.__serial.is_open:
			rospy.logwarn("Serial port for lowlevel interface ("+str(self.__comPort)+") cannot be opened.")

		else:
			self.__serial.flushInput()
			self.__serial.flushOutput()
			rospy.loginfo("Connected to "+str(self.__comPort)+".")

	def __rosInitSubPubAction(self):
		self.rosInitPublisher( 	"/spinalcord/suchart_status",
								SuchartFeedback)
		self.__action = actionlib.SimpleActionServer(
												"/spinalcord/suchart_action",
												SuchartAction,
											execute_cb = self.__actionCallback,
												auto_start = False)
		if self.__action.is_active():
			self.__action.set_aborted()
		self.__action.start()

	def __actionCallback(self, goal):
		
		r = rospy.Rate(1)
		success = False

		feedback = SuchartFeedback()

		self.__goalPosition = None
		self.__gripperCommand = None

		if goal.command.lower() == "reach_position":
			self.__goalPosition = self.createGoalPositionDict(	goal.name,
																goal.position)
			goalPosition = self.createGoalPositionDict(	goal.name,
														goal.position)
			# self.__setGoalPosition()
			rospy.loginfo("Start go to goal position ("+str(goalPosition)+").")
			while self.__goalPosition is not None:
				pass
			time.sleep(1)
			success = False
			while not rospy.is_shutdown():
				if self.__suchartStatus.comunication_OK:
					if self.__suchartStatus.is_reach_pos:
						success = True
						break
					feedback = self.__createFeedBack()
					self.__action.publish_feedback(feedback)
				else:
					success = False
					break
				r.sleep()

			if success:
				# pass
				rospy.loginfo("Suchart reach to "+str(goalPosition))

		elif goal.command.lower() == "active_gripper":
			# self.__suchartInterface.set_GripperActive()
			self.__gripperCommand = "active"
			rospy.loginfo("Start active gripper.")
			while self.__gripperCommand is not None:
				pass
			time.sleep(1)
			success = False
			while not rospy.is_shutdown():
				if self.__suchartStatus.comunication_OK:
					if self.__suchartStatus.gripper_active:
						success = True
						break
					feedback = self.__createFeedBack()
					self.__action.publish_feedback(feedback)
				else:
					success = False
					break
				r.sleep()
			if success:
				rospy.loginfo("gripper is active.")

		elif goal.command.lower() == "release_gripper":
			# self.__suchartInterface.set_GripperRelease()
			self.__gripperCommand = "release"
			rospy.loginfo("Start release gripper.")
			while self.__gripperCommand is not None:
				pass
			time.sleep(1)
			success = False
			while not rospy.is_shutdown():
				if self.__suchartStatus.comunication_OK:
					if self.__suchartStatus.gripper_release:
						success = True
						break
					feedback = self.__createFeedBack()
					self.__action.publish_feedback(feedback)
				else:
					success = False
					break
				r.sleep()

			if success:
				rospy.loginfo("gripper is release.")

		else:
			rospy.logwarn("Unknown command \""+goal.command+"\"")
			success = False

		result = SuchartResult(is_success = success)
		if success:
			self.__action.set_succeeded(result)
		else:
			self.__action.set_aborted(result)

	def __createFeedBack(self):
		if not self.__suchartStatus.comunication_OK:
			return SuchartFeedback()
		goalPos = self.createJoitstateFromDict(self.__suchartStatus.goalPosition)
		currPos = self.createJoitstateFromDict(self.__suchartStatus.currPosition)
		gripper_active = self.__suchartStatus.gripper_active
		gripper_release = self.__suchartStatus.gripper_release
		is_initial = self.__suchartStatus.gameStatusInitialState
		is_play = self.__suchartStatus.gameStatusPlayState

		feedback = SuchartFeedback(	goalPosition = goalPos,
									currPosition = currPos,
								gripper_active = gripper_active,
								gripper_release = gripper_release,
							gameStatusInitialState = is_initial,
							gameStatusPlayState = is_play)
		return feedback

	@staticmethod
	def createGoalPositionDict(jointName, position):
		goalPosition = {}
		for n,v in zip(jointName, position):
			goalPosition[n] = v
		return goalPosition

	@staticmethod
	def createJoitstateFromDict(posDict):
		positions = []
		names = []
		for n,p in posDict.items():
			names.append(n)
			positions.append(p)
		return JointState(	name = names,
							position = positions)

	def __setGoalPosition(self):
		success = self.__suchartInterface.set_GoalPosition(self.__goalPosition)
		self.__goalPosition = None
		return success

	def run(self):
		rospy.loginfo("Start motor cortex node.")
		while not rospy.is_shutdown():
			## read Suchart status
			ret, self.__suchartStatus = self.__suchartInterface.request_suchartStatus()
			if not ret:
				rospy.logwarn("Cannot communicate with Suchart. Check your serial connection.")
			else:
				if self.__goalPosition is not None:
					if not self.__setGoalPosition():
						rospy.logwarn("Cannot comunicate with Suchart. Check your serial connection.")

				elif self.__gripperCommand is not None:
					if self.__gripperCommand == "active":
						self.__suchartInterface.set_GripperActive()
					elif self.__gripperCommand == "release":
						self.__suchartInterface.set_GripperRelease()
					self.__gripperCommand = None

			msg = self.__createFeedBack()
			self.publish(msg)
			self.sleep()

	def end(self):
		# if self.__action.is_active():
		# 	# self.__action.set_preempted()
		# 	self.__action.set_aborted()

		self.__serial.close()
		time.sleep(0.1)
		if self.__serial.is_open:
			rospy.logwarn("Cannot close "+ str(self.__serial.port))

def main():
	node = SuchartMotorCortex()
	try:
		node.run()
	except rospy.ROSInterruptException:
		pass
	finally:
		node.end()