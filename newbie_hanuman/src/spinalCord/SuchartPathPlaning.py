#!/usr/bin/env python
from cell.nodeCell import NodeBase
from SuchartBase import SuchartStatus, RegisterAddrDict
from SuchartMotorCortex import SuchartMotorCortex as Cortex
from newbie_hanuman.msg import SuchartFeedback
from newbie_hanuman.msg import SuchartAction 
from newbie_hanuman.msg import SuchartResult
from newbie_hanuman.msg import SuchartPathPlaningCommandFeedback as Feedback 
from newbie_hanuman.msg import SuchartPathPlaningCommandAction as Action
from newbie_hanuman.msg import SuchartPathPlaningCommandResult as Result
from newbie_hanuman.msg import SuchartPathPlaningCommandGoal as Goal

import rospy
import actionlib

class SuchartPathPlaning(NodeBase):
	
	def __init__(self):
		super(SuchartPathPlaning, self).__init__("path_planing")

		self.__suchartStatus = None

		self.rosInitNode()

		self.__rosInitAction()

		self.setFrequencyFromParam("spinalcord/path_planing_frequency")

	def __rosInitAction(self):
		self.__actionClient = actionlib.SimpleActionClient(
												"/spinalcord/suchart_action",
												SuchartAction)
		self.__actionClient.wait_for_server()

		self.__actionServer = actionlib.SimpleActionServer(
											"/spinalcord/suchart_path_planing",
											SuchartAction,
										execute_cb = self.__actionSrvCallback,
										auto_start = False)
		self.rosInitSubscriber(	"/spinalcord/suchart_status",
								SuchartFeedback,
								self.__callback)

	def __callback(self, msg):
		self.__suchartStatus = SuchartStatus(None)
		goalPos = {name:pos for name,pos in zip(msg.goalPosition.name, msg.goalPosition.position)}
		currPos = {name:pos for name,pos in zip(msg.currPosition.name, msg.currPosition.position)}
		self.__suchartStatus.goalPosition = goalPos
		self.__suchartStatus.currPosition = currPos

		self.__suchartStatus.is_reach_pos = msg.is_reach_pos
		self.__suchartStatus.gripper_active = msg.gripper_active
		self.__suchartStatus.gripper_release = msg.gripper_release
		self.__suchartStatus.gameStatusInitialState = msg.gameStatusInitialState
		self.__suchartStatus.gameStatusPlayState = msg.gameStatusPlayState

		self.__suchartStatus.comunication_OK = True

	def searchPath(self, curr, goal):
		found = 1
		return found, [curr, goal]

	def __reachPos(self, goal):
		currPos = self.__suchartStatus.currPosition
		goalPos = {name:pos for name,pos in zip(goal.goalPosition.name, goal.goalPosition.position)}

		found, viaPoints = self.searchPath(currPos, goalPos)
		nViaPoint = len(viaPoint)

		currJoint =  Cortex.createJoitstateFromDict(currPos)
		for i, point in enumerate(viaPoints):
			goalJoint = Cortex.createJoitstateFromDict(point)
			feedback = Feedback(goalPosition = goalJoint,
								currPosition = currJoint,
								n_via_point = nViaPoint,
								current_via_point = i)
			self.__actionServer.publish_feedback(feedback)

			goalMsg = SuchartGoal(	name = goalJoint.name,
									position = goalJoint.position,
									command = "reach_position")
			self.__actionClient.send_goal(goalMsg)
			self.__actionClient.wait_for_result()
			result = self.__actionClient.get_result()
			if not result.is_success:
				serverResult = Result(result = (found<<1)|0)
				# self.__actionServer.set_aborted(serverResult)
				return serverResult
			currJoint = goalJoint
		serverResult = Result(result = (found<<1)|1)
		# self.__actionServer.set_succeded(serverResult)
		return serverResult

	def __activeGripper(self):
		commandMsg = SuchartGoal(command = "active_gripper")
		self.__actionClient.send_goal(commandMsg)
		self.__actionClient.wait_for_result()
		return self.__actionClient.get_result()

	def __releaseGripper(self):
		commandMsg = SuchartGoal(command = "release_gripper")
		self.__actionClient.send_goal(commandMsg)
		self.__actionClient.wait_for_result()
		return self.__actionClient.get_result()

	def __actionSrvCallback(self, goal):
		if goal.command.lower() == "reach_position":
			result = self.__reachPos(goal)
			if result.result&1 == 0:
				self.__actionServer.set_aborted(result)
			self.__actionServer.set_succeded(result)

		elif goal.command.lower() == "active_gripper":
			success = 1 if self.__activeGripper() else 0
			self.__actionServer.set_succeded(Result(result=success))

		elif goal.command.lower() == "release_gripper":
			success = 1 if self.__activeGripper() else 0
			self.__actionServer.set_succeded(Result(result=success))

		else:
			self.__actionServer.set_aborted(Result(result=success))