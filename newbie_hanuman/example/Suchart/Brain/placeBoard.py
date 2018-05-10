#!/usr/bin/env python
import numpy as np
from brain.brainState import FSMBrainState
from brain.SuchartInterface import SuchartRosInterface

from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState

from robotkinematics.scarakinematics import SCARAKinematics
from idle import Idle

import rospy

import math

JOINT_NAME = ["joint"+str(i+1) for i in range(6)]

class PlaceBoards(FSMBrainState):

	GOAL = [Point32(x=0.7, y= 0.3, z=0.7),
			Point32(x=0.7, y= 0.1, z=0.7),
			Point32(x=0.7, y=-0.1, z=0.7),
			Point32(x=0.7, y=-0.3, z=0.7),
			Point32(x=0.7, y= 0.3, z=0.5),
			Point32(x=0.7, y= 0.1, z=0.5),
			Point32(x=0.7, y=-0.1, z=0.5),
			Point32(x=0.7, y=-0.3, z=0.5),
			Point32(x=0.7, y= 0.1, z=0.3),
			Point32(x=0.7, y=-0.1, z=0.3)]

	LEFT_ORIENT = np.array([np.pi/2,np.pi/2,0], dtype=float)
	RIGHT_ORIENT = np.array([-np.pi/2,np.pi/2,0], dtype=float)
	GROUND_ORIENT = np.array([0,np.pi,0], dtype=float)
	FRONT_ORIENT = np.array([0,np.pi/2,0], dtype=float)

	GRIPPER_OFFSET = 0.05

	def __init__(self, nextstate=None):
		super(PlaceBoards, self).__init__("PlaceBoards")

		self.__kinematic = SCARAKinematics()

		self.nextstate = nextstate

		self.__isfinish = False

		self.__coorBoard = []

		self.__state = 0

		###################################
		idle = Idle()
		reachPos = ReachPosition(nextstate="Idle")

		self.addSubBrain(reachPos)
		self.addSubBrain(idle)

		self.setFirstSubBrain("Idle")

	def set_boardCoor(self, boardCoor):
		self.__coorBoard = boardCoor
		self.__coorBoard = sorted(self.__coorBoard, key=lambda x:x[1])

	def inPlane(self, point):
		d1 = math.fabs(point.y - 0.5)
		d2 = math.fabs(point.y + 0.5)
		d3 = math.fabs(point.z - 0.0)
		min_d = min([d1, d2, d3])
		if min_d == d1:
			return "left"

		if min_d == d2:
			return "right"

		if min_d == d3:
			return "ground"

	@property
	def isFinish(self):
		return self.__isfinish

	def firstStep(self):
		rospy.loginfo("PlaceBoards.")
		self.__isfinish = False

	def step(self):
		if self.currSubBrainName != "Idle":
			return

		if len(self.__coorBoard) == 0 and self.__state == 0 and self.subBrains["ReachPosition"].isFinish:
			self.__isfinish = True
			self.SignalChangeSubBrain(self.nextstate)
			return

		elif (len(self.GOAL)==0 and self.__state==1) and self.subBrains["ReachPosition"].isFinish:
			self.__isfinish = True
			self.SignalChangeSubBrain(self.nextstate)
			return

		if self.__isfinish:
			return

		if not self.rosInterface.pathPlaning.isFinish:
			return

		if self.__state == 0:
			rospy.loginfo(str(len(self.__coorBoard)))
			coor = self.__coorBoard.pop(0)[0]
			coorNP = np.array([coor.x,coor.y,coor.z], dtype=float)
			# TODO : use invert kinematic to convert task space to joint space.
			# TODO : command path planing.
			if self.inPlane(coor)=="left":
				orient = self.LEFT_ORIENT
				offset = np.array([0.0,-self.GRIPPER_OFFSET,0.0], dtype=float)
				coorNP[1] -= 0.03
				coorNP[2] -= 0.05
			elif self.inPlane(coor)=="right":
				orient = self.RIGHT_ORIENT
				offset = np.array([0.0, self.GRIPPER_OFFSET,0.0], dtype=float)
				coorNP[1] += 0.03
				coorNP[2] -= 0.05
			elif self.inPlane(coor)=="ground":
				orient = self.GROUND_ORIENT
				offset = np.array([0.0,0.0, self.GRIPPER_OFFSET], dtype=float)
				offset *= 2.0
				coorNP[2] += 0.0

			self.subBrains["ReachPosition"].setGoal(coorNP,offset,orient,False)
			self.ChangeSubBrain("ReachPosition")
			self.__state = 1

		elif self.__state == 1:
			coor = self.GOAL.pop(0)
			coorNP = np.array([coor.x,coor.y,coor.z], dtype=float)
			# TODO : use invert kinematic to convert task space to joint space.
			# TODO : comand path planing,
			coorNP[0] -= 0.1
			coorNP[2] -= 0.05
			orient = self.FRONT_ORIENT
			offset = np.array([-self.GRIPPER_OFFSET,0,0], dtype=float)

			self.subBrains["ReachPosition"].setGoal(coorNP,offset,orient,True)
			self.ChangeSubBrain("ReachPosition")
			self.__state = 0

class ReachPosition(FSMBrainState):

	## go to offseted position w/ motion planning.--> contact board w/o motion planning--> release or active gripper. --> offset gripper from the wall.

	def __init__(self, nextstate=None):
		super(ReachPosition, self).__init__("ReachPosition")
		self.__nextstate = nextstate

		self.__kinematic = SCARAKinematics()

		self.__goal = None
		self.__offset = None
		self.__orient = None
		self.__isRelease = None
		self.__isfinish = True
		self.__editSig = -np.array([0.005, 0.005, 0.005])

		self.__state = 0

	def setGoal(self, goal, offset, orient, isRelease):
		self.__goal = goal
		self.__offset = offset
		self.__orient = orient
		self.__isRelease = isRelease
		self.__isfinish = False

	def getReachPosCommand(self, q, ignoreObstacles=False):
		return {"goalPosition":JointState(	name=JOINT_NAME,
											position=q),
				"command":"reach_position",
				"ignoreObstacles":ignoreObstacles}

	def jointstateToList(self, js):
		q = [0]
		for i in range(6):
			indx = js.name.index("joint"+str(i+1))
			q.append(js.position[indx])
		return q

	def firstStep(self):
		rospy.loginfo("ReachPosition. Reach to "+str(self.__goal)+"."+str(self.__isfinish))
		self.__state = 0

	def step(self):
		if not self.rosInterface.pathPlaning.isFinish:
			return

		if self.__isfinish:
			self.__goal = None
			self.__offset = None
			self.__orient = None
			self.__isRelease = None
			self.SignalChangeSubBrain(self.__nextstate)
			return

		if self.__state == 0:
			goalOffset = self.__goal + self.__offset
			q_pre=self.jointstateToList(self.rosInterface.motorCortex.currPosition)

			rospy.loginfo("reach to "+str(goalOffset)+".")
			q = self.__kinematic.inverseKinematicsCalculate(goalOffset*1000,
															self.__orient,
															"zyz", 
															q_pre)
			# print q, goalOffset, self.__orient, q_pre
			i = 0
			while q is None:
				goalOffset += 0.01*self.__offset + self.__editSig
				# goalOffset += np.array([0.1,0.1,0.1])
				# print goalOffset
				q = self.__kinematic.inverseKinematicsCalculate(goalOffset*1000,
															self.__orient,
															"zyz", 
															q_pre)
				if i > 100:
					break
				i += 1
			print goalOffset
			if q is None:
				self.__state = 4
				return
			q = q[1:]
			command = self.getReachPosCommand(q, ignoreObstacles=False)
			self.rosInterface.pathPlaning.setGoal(**command)
			self.__state = 1

		elif self.__state == 1:
			goal = self.__goal
			q_pre=self.jointstateToList(self.rosInterface.motorCortex.currPosition)
			rospy.loginfo("reach to "+str(goal)+".")
			q = self.__kinematic.inverseKinematicsCalculate(goal*1000,
															self.__orient,
															"zyz", 
															q_pre)
			i = 0
			while q is None:
				goal += 0.01*self.__offset + self.__editSig
				q = self.__kinematic.inverseKinematicsCalculate(goal*1000,
															self.__orient,
															"zyz", 
															q_pre)
				if i > 100:
					break
				i += 1
			print goal
			if q is None:
				self.__state = 4
				return
			q = q[1:]
			command = self.getReachPosCommand(q, ignoreObstacles=True)
			self.rosInterface.pathPlaning.setGoal(**command)
			self.__state = 2

		elif self.__state == 2:
			if self.__isRelease:
				rospy.loginfo("ReachPosition. Release gripper")
				self.rosInterface.pathPlaning.setGoal(command="release_gripper")

			elif not self.__isRelease:
				rospy.loginfo("ReachPosition. Active gripper")
				self.rosInterface.pathPlaning.setGoal(command="active_gripper")
			self.__state = 3

		elif self.__state == 3:
			goal = self.__goal + self.__offset
			q_pre = self.jointstateToList(self.rosInterface.motorCortex.currPosition)
			rospy.loginfo("reach to "+str(goal)+".")
			q = self.__kinematic.inverseKinematicsCalculate(goal*1000,
															self.__orient, 
															"zyz",
															q_pre)
			i = 0
			while q is None:
				goal += 0.01*self.__offset + self.__editSig
				q = self.__kinematic.inverseKinematicsCalculate(goal*1000,
															self.__orient,
															"zyz", 
															q_pre)
				if i > 100:
					break
				i += 1
			if q is None:
				self.__state = 4
				return
			q = q[1:]
			command = self.getReachPosCommand(q, ignoreObstacles=True)
			self.rosInterface.pathPlaning.setGoal(**command)
			self.__state = 0
			self.__isfinish = True

		elif self.__state == 4:
			rospy.loginfo("ReachPosition. Release gripper. Skip step.")
			self.rosInterface.pathPlaning.setGoal(command="release_gripper")
			self.__isfinish = True

	@property
	def isFinish(self):
		return self.__isfinish