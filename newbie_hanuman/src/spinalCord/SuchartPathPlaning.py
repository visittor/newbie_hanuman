#!/usr/bin/env python
import numpy as np 
import cv2

from cell.nodeCell import NodeBase
from SuchartBase import SuchartStatus, RegisterAddrDict
from SuchartMotorCortex import SuchartMotorCortex as Cortex
from newbie_hanuman.msg import SuchartFeedback
from newbie_hanuman.msg import SuchartAction 
from newbie_hanuman.msg import SuchartResult
from newbie_hanuman.msg import SuchartGoal
from newbie_hanuman.msg import SuchartPathPlaningCommandFeedback as Feedback 
from newbie_hanuman.msg import SuchartPathPlaningCommandAction as Action
from newbie_hanuman.msg import SuchartPathPlaningCommandResult as Result
from newbie_hanuman.msg import SuchartPathPlaningCommandGoal as Goal

import rospy
import actionlib

import time
import sys

import pprint

PATH = sys.path[0]

MAP = np.genfromtxt("/".join(PATH.split("/")[:]+["spinalCord/map_csv/map.csv"]), delimiter=",")
Q1 =  np.genfromtxt("/".join(PATH.split("/")[:]+["spinalCord/map_csv/Q1.csv"]), delimiter=",")
Q3 =  np.genfromtxt("/".join(PATH.split("/")[:]+["spinalCord/map_csv/Q3.csv"]), delimiter=",")

def radToMap(q1, q3):
	dQ1 = np.abs(Q1 - q1)
	dQ3 = np.abs(Q3 - q3)
	dist = dQ1 + dQ3
	idx = np.unravel_index(np.argmin(dist, axis=None), dist.shape)
	mapQ1 = idx[0]
	mapQ3 = idx[1]
	return mapQ1, mapQ3

def mapTorad(q1, q3):
	radQ1 = Q1[q1,q3]
	radQ3 = Q3[q1,q3]
	return radQ1, radQ3

def findAdjacent(map_, p):
	h,w = map_.shape

	adjacent = []

	if p[0] -1 != -1:
		adjacent.append((p[0]-1, p[1]))

	if p[1] - 1 != -1:
		adjacent.append((p[0], p[1]-1))

	if p[0] + 1 < h:
		adjacent.append((p[0]+1, p[1]))

	if p[1] + 1 < w:
		adjacent.append((p[0], p[1]+1))

	return adjacent

def mapFunc(map_,q1, q3):
	return map_[q1,q3]

def computeCostFunction(agent, n):
	return 2*np.abs(agent[0]-n[0])+np.abs(agent[1]-n[1])

def computeHeuristicFunc(goal, n):
	return 2*(np.abs(n[0]-goal[0]) + np.abs(n[1]-goal[1]))

def AStar(init, goal):
	if  mapFunc(MAP,init[0], init[1]) != 0:
		rospy.logwarn("Invalid starting point. Initialize inside obstacle.")
		return False, []

	if  mapFunc(MAP,goal[0], goal[1]) != 0:
		rospy.logwarn("Invalid starting point. Goal is inside obstacle.")
		return False, []

	map_ = np.zeros((MAP.shape[0],MAP.shape[1],3), dtype=np.uint8)
	map_[:,:,0] = MAP
	map_[:,:,1] = MAP
	map_[:,:,2] = MAP

	found = False

	agent = init
	priQ = []
	visited = [init]
	hashTable = {}
	hFuncStart = np.abs(init[0]-goal[0]) + np.abs(init[1]-goal[1])
	minCost = None

	for n in findAdjacent(MAP, agent):
		costFunc = computeCostFunction(agent, n)
		hFunc = computeHeuristicFunc(goal, n)
		N = (costFunc, hFunc, n)
		if N not in visited and N not in priQ and mapFunc(MAP,n[0], n[1])==0:
			priQ.append(N)
			priQ.sort(key=lambda x:x[0]+x[1])
			hashTable[N] = (0, hFunc, agent)

	while len(priQ) != 0:
		current = priQ.pop(0)
		agent = current[2]
		cost = current[0]
		h = current[1]
		visited.append(current)

		if found:
			if cost + h > minCost:
				continue

		if agent == goal:
			found = True
			minCost = cost + h
			keyGoal = current
			continue

		for n in findAdjacent(MAP, agent):
			costFunc = computeCostFunction(agent, n)
			hFunc = computeHeuristicFunc(goal, n)
			N = (costFunc+cost, hFunc, n)
			# print N[0]
			if N not in visited and N not in priQ and mapFunc(MAP,n[0],n[1])!=1:
				priQ.append(N)
				priQ.sort(key=lambda x:x[0]+x[1])
				hashTable[N] = current
		img = map_*255
		img[init[0], init[1], :] = [255,0,0]
		img[goal[0], goal[1], :] = [255,0,0]
		img[agent[0], agent[1], :] = [0,0,255]
		img = cv2.resize(img,(500,500))
		cv2.imshow("map", img)
		cv2.waitKey(10)

	if found:
		rospy.loginfo("Search found.")
		path = [goal]
		key = hashTable[keyGoal]
		path.append(key)
		# print key
		map_ = map_*255
		map_[key[2][0], key[2][1], :] = [0,255,0]
		cv2.imshow("map", map_)
		cv2.waitKey(1)

		while True:
			key = hashTable[key]
			print key
			path.append(key)

			map_[key[2][0], key[2][1], :] = [0,255,0]
			cv2.imshow("map", cv2.resize(map_,(500,500)))
			cv2.waitKey(1)

			if key[2] == init:
				break
		cv2.waitKey(100)
		cv2.destroyAllWindows()
		return found, path
	rospy.loginfo("Search not found.")
	return False, []

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
		self.__actionClient.cancel_all_goals()
		self.__actionServer = actionlib.SimpleActionServer(
											"/spinalcord/suchart_path_planing",
											Action,
										execute_cb = self.__actionSrvCallback,
										auto_start = False)
		if self.__actionServer.is_active():
			self.__actionServer.set_aborted()
		self.__actionServer.start()
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
		found, path = AStar((30,30), (40,20))
		found = 1 if found else 0
		return found, [curr, goal]

	def __reachPos(self, goal):
		currPos = self.__suchartStatus.currPosition
		goalPos = {name:pos for name,pos in zip(goal.goalPosition.name, goal.goalPosition.position)}
		found, viaPoints = self.searchPath(currPos, goalPos)
		nViaPoint = len(viaPoints)

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
			self.__actionClient.send_goal(goalMsg, feedback_cb = self.__callback)
			self.__actionClient.wait_for_result()
			result = self.__actionClient.get_result()
			if not result or not result.is_success:
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
				# rospy.loginfo("Action reach goal aborted.")
				self.__actionServer.set_aborted(result)
			# rospy.loginfo("Action reach goal succeed.")
			self.__actionServer.set_succeeded(result)

		elif goal.command.lower() == "active_gripper":
			success = 1 if self.__activeGripper() else 0
			rospy.loginfo("Action active_gripper succeed.")
			self.__actionServer.set_succeeded(Result(result=success))

		elif goal.command.lower() == "release_gripper":
			success = 1 if self.__releaseGripper() else 0
			rospy.loginfo("Action release_gripper succeed.")
			self.__actionServer.set_succeeded(Result(result=success))

		else:
			rospy.loginfo("Action aborted.")
			self.__actionServer.set_aborted(Result(result=success))

	def run(self):
		rospy.spin()

	def end(self):
		self.__actionClient.cancel_all_goals()
		# self.__actionServer.set_aborted()
		time.sleep(0.1)

def main():
	node = SuchartPathPlaning()
	try:
		node.run()
	except rospy.ROSInterruptException:
		pass
	finally:
		node.end()