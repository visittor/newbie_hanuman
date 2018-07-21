#!/usr/bin/env python
import numpy as np
from brain.brainState import FSMBrainState
from brain.SuchartInterface import SuchartRosInterface

from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState

from robotkinematics.scarakinematics import SCARAKinematics
from idle import Idle
from UTurn import UTurn

import rospy

import math

JOINT_NAME = ["joint"+str(i+1) for i in range(6)]

class PlaceBoards(FSMBrainState):

	GOAL = [Point32(x=0.79, y= 0.275, z=0.7),
			Point32(x=0.79, y= 0.1, z=0.7),
			Point32(x=0.79, y=-0.1, z=0.7),
			Point32(x=0.79, y=-0.275, z=0.7),
			Point32(x=0.79, y= 0.275, z=0.5),
			Point32(x=0.79, y= 0.1, z=0.5),
			Point32(x=0.79, y=-0.1, z=0.5),
			Point32(x=0.79, y=-0.275, z=0.5),
			Point32(x=0.79, y= 0.1, z=0.3),
			Point32(x=0.79, y=-0.1, z=0.3)]

	LEFT_ORIENT = np.array([np.pi/2,np.pi/2,0], dtype=float)
	RIGHT_ORIENT = np.array([-np.pi/2,np.pi/2,0], dtype=float)
	GROUND_ORIENT = np.array([0,np.pi,0], dtype=float)
	FRONT_ORIENT = np.array([0,np.pi/2,0], dtype=float)

	GRIPPER_OFFSET = 0.15

	def __init__(self, nextstate=None):
		super(PlaceBoards, self).__init__("PlaceBoards")

		self.__kinematic = SCARAKinematics()

		self.nextstate = nextstate

		self.__isfinish = False

		self.__coorBoard = []

		self.__state = 0

		self.__armSide = 'left'

		###################################
		idle = Idle()
		reachPos = ReachPosition(nextstate="Idle")
		uturn = UTurn(nextstate="ReachPosition")

		self.addSubBrain(reachPos)
		self.addSubBrain(idle)
		self.addSubBrain(uturn)

		self.setFirstSubBrain("Idle")

	def set_boardCoor(self, boardCoor):
		self.__coorBoard = boardCoor
		self.__coorBoard = sorted(self.__coorBoard, key=lambda x:x[1]%10)

		index = [ i for i in range(len(self.__coorBoard))]
		index = sorted(index, key= lambda x:self.__getkey(self.__coorBoard[x]))
		index.extend([ i for i in range(len(self.__coorBoard), len(self.GOAL))])

		self.GOAL = sorted(self.GOAL, key = lambda x:index.index(self.GOAL.index(x)))
		self.__coorBoard = sorted(self.__coorBoard, key = lambda x:index.index(self.__coorBoard.index(x)))

	def __getkey(self, board):
		if self.inPlane(board[0])=="left":
			return (1-board[0].z)*1.0

		elif self.inPlane(board[0])=="right":
			return (1-board[0].z)*10000.0

		else:
			return (1-board[0].x)*100.0

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
			coor = self.__coorBoard.pop(0)
			l = coor[1]
			coor = coor[0]
			rospy.loginfo(str(len(self.__coorBoard))+" remain.")
			rospy.loginfo("Pick "+str(l)+".")
			coorNP = np.array([coor.x,coor.y,coor.z], dtype=float)
			# TODO : use invert kinematic to convert task space to joint space.
			# TODO : command path planing.
			if self.inPlane(coor)=="left":
				orient = self.LEFT_ORIENT
				offset = np.array([-0.05,-self.GRIPPER_OFFSET,0.0], dtype=float)
				coorNP[1] -= 0.0015
				coorNP[2] += max(0.125*(1-coorNP[2]) - 0.025, 0)
				self.subBrains["ReachPosition"].kinematic.robot.q_upper[2]= 3*np.pi/4
				self.subBrains["ReachPosition"].kinematic.robot.q_lower[2]=-0.5*np.pi/4
				self.subBrains["UTurn"].setUTurn(not self.__armSide=="left",
												 self.__armSide)
				self.__armSide = 'left'

			elif self.inPlane(coor)=="right":
				orient = self.RIGHT_ORIENT
				offset = np.array([-0.05, self.GRIPPER_OFFSET,0.0], dtype=float)
				coorNP[1] += 0.001
				# coorNP[2] -= 0.05
				self.subBrains["ReachPosition"].kinematic.robot.q_lower[2]=-3*np.pi/4
				self.subBrains["ReachPosition"].kinematic.robot.q_upper[2]=0.5*np.pi/4
				self.subBrains["UTurn"].setUTurn(not self.__armSide=="right",
												 self.__armSide)
				self.__armSide = 'right'


			elif self.inPlane(coor)=="ground":
				orient = self.GROUND_ORIENT
				offset = np.array([0.0,0.0, self.GRIPPER_OFFSET], dtype=float)
				offset *= 0.25
				coorNP[0] -= 0.025
				coorNP[1] += 0.05
				coorNP[2] = 0.26
				if coorNP[0] < 0.2:
					coorNP[0] = 0.17
					coorNP[1] = 0.37 if coorNP[1] > 0 else -0.37
				self.subBrains["ReachPosition"].kinematic.robot.q_upper[2]= 3*np.pi/4
				self.subBrains["ReachPosition"].kinematic.robot.q_lower[2]=-0.5*np.pi/4
				self.subBrains["UTurn"].setUTurn(not self.__armSide=="left",
												 self.__armSide)
				self.__armSide = 'left'


			self.subBrains["ReachPosition"].setGoal(coorNP,offset,orient,False)
			self.ChangeSubBrain("UTurn")
			self.__state = 1

		elif self.__state == 1:
			coor = self.GOAL.pop(0)
			rospy.loginfo("Place to target number "+str(11-len(self.GOAL)))
			coorNP = np.array([coor.x,coor.y,coor.z], dtype=float)
			# TODO : use invert kinematic to convert task space to joint space.
			# TODO : comand path planing,
			coorNP[0] -= 0.05
			# coorNP[2] -= 0.07
			orient = self.FRONT_ORIENT
			if coorNP[1] > 0:
				offset = 1.5*np.array([-self.GRIPPER_OFFSET,-0.055,0], dtype=float)
			else:
				offset = 1.5*np.array([-self.GRIPPER_OFFSET, 0.06,0], dtype=float)

			self.subBrains["ReachPosition"].setGoal(coorNP,offset,orient,True)
			self.ChangeSubBrain("ReachPosition")
			self.__state = 0

class ReachPosition(FSMBrainState):

	## go to offseted position w/ motion planning.--> contact board w/o motion planning--> release or active gripper. --> offset gripper from the wall.

	def __init__(self, nextstate=None):
		super(ReachPosition, self).__init__("ReachPosition")
		self.__nextstate = nextstate

		self.kinematic = SCARAKinematics()

		self.__goal = None
		self.__offset = None
		self.__orient = None
		self.__isRelease = None
		self.__isfinish = True
		self.__editSig = -np.array([0.005, 0.005, 0.000],  dtype = float)
		self.__editOrt = np.array([1*np.pi/180,0.0,0.0], dtype = float)

		self.__state = "offset1"

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
		q = []
		for i in range(6):
			indx = js.name.index("joint"+str(i+1))
			q.append(js.position[indx])
		return q

	def firstStep(self):
		rospy.loginfo("ReachPosition. Reach to "+str(self.__goal)+"."+str(self.__isfinish))
		self.__state = "offset1"

	def __editOrient(self, goal, orient, q_pre):
		q = None
		i = 0
		print "Singularity or something"
		while q is None:
			
			# goalOffset += 0.001*self.__offset + self.__editSig
			orient += self.__editOrt
			# goalOffset += np.array([0.1,0.1,0.1])
			# print goalOffset
			q = self.kinematic.inverseKinematicsCalculate(goal*1000,
														orient,
														"zyz", 
														q_pre,
														option=1)
			if i > 100:
				break
			i += 1

		i = 0
		while q is None:

			# goalOffset += 0.001*self.__offset + self.__editSig
			orient -= self.__editOrt
			# goalOffset += np.array([0.1,0.1,0.1])
			# print goalOffset
			q = self.kinematic.inverseKinematicsCalculate(goal*1000,
														self.__orient,
														"zyz", 
														q_pre,
														option=1)
			if i > 200:
				break
			i += 1
		return q

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

		if self.__state == "offset1":
			goalOffset = self.__goal + self.__offset
			q_pre=self.jointstateToList(self.rosInterface.motorCortex.currPosition)

			rospy.loginfo("go to offset ("+str(goalOffset)+").")
			q = self.kinematic.inverseKinematicsCalculate(goalOffset*1000,
															self.__orient,
															"zyz", 
															q_pre,
															option=1)
			# print q, goalOffset, self.__orient, q_pre
			if q is None:
				q = self.__editOrient(goalOffset, self.__orient.copy(), q_pre)
			print q
			if q is None:
				self.__state = "skip"
				return
			# q = q[1:]
			# q[0] -= 3*np.pi/180.0
			command = self.getReachPosCommand(q, ignoreObstacles=False)
			self.rosInterface.pathPlaning.setGoal(**command)
			self.__state = "offset2"

		elif self.__state == "offset2":
			goal = self.__goal + self.__offset*0.5
			q_pre=self.jointstateToList(self.rosInterface.motorCortex.currPosition)
			rospy.loginfo("go to goal ("+str(goal)+").")
			q = self.kinematic.inverseKinematicsCalculate(goal*1000,
															self.__orient,
															"zyz", 
															q_pre)
			if q is None:
				q = self.__editOrient(goal, self.__orient.copy(), q_pre)
			print q
			if q is None:
				self.__state = "skip"
				return
			# q = q[1:]
			# q[0] -= 3*np.pi/180.0
			if (self.__orient == PlaceBoards.GROUND_ORIENT).all():
				q[1] = 0.0
			command = self.getReachPosCommand(q, ignoreObstacles=True)
			self.rosInterface.pathPlaning.setGoal(**command)
			self.__state = "goal"

		elif self.__state == "goal":
			goal = self.__goal
			q_pre=self.jointstateToList(self.rosInterface.motorCortex.currPosition)
			rospy.loginfo("go to goal ("+str(goal)+").")
			q = self.kinematic.inverseKinematicsCalculate(goal*1000,
															self.__orient,
															"zyz", 
															q_pre)
			if q is None:
				q = self.__editOrient(goal, self.__orient.copy(), q_pre)
			print q
			if q is None:
				self.__state = "skip"
				return
			# q = q[1:]
			# q[0] -= 3*np.pi/180.0
			if (self.__orient == PlaceBoards.GROUND_ORIENT).all():
				q[1] = 0.0
			command = self.getReachPosCommand(q, ignoreObstacles=True)
			self.rosInterface.pathPlaning.setGoal(**command)
			self.__state = "gripper"

		elif self.__state == "gripper":
			if self.__isRelease:
				rospy.loginfo("ReachPosition. Release gripper")
				self.rosInterface.pathPlaning.setGoal(command="release_gripper")

			elif not self.__isRelease:
				rospy.loginfo("ReachPosition. Active gripper")
				self.rosInterface.pathPlaning.setGoal(command="active_gripper")
			self.__state = "offset3"

		elif self.__state == "offset3":
			goal = self.__goal + 0.5*self.__offset
			q_pre = self.jointstateToList(self.rosInterface.motorCortex.currPosition)
			rospy.loginfo("Double offset("+str(goal)+").")
			q = self.kinematic.inverseKinematicsCalculate(goal*1000,
															self.__orient, 
															"zyz",
															q_pre)
			if q is None:
				q = self.__editOrient(goal, self.__orient.copy(), q_pre)
			print q
			if q is None:
				self.__state = "skip"
				return
			# q = q[1:]
			# q[0] -= 3*np.pi/180.0
			command = self.getReachPosCommand(q, ignoreObstacles=True)
			self.rosInterface.pathPlaning.setGoal(**command)
			self.__state = "offset4"

		elif self.__state == "offset4":
			goal = self.__goal + self.__offset
			q_pre = self.jointstateToList(self.rosInterface.motorCortex.currPosition)
			rospy.loginfo("Double offset("+str(goal)+").")
			q = self.kinematic.inverseKinematicsCalculate(goal*1000,
															self.__orient, 
															"zyz",
															q_pre)
			if q is None:
				q = self.__editOrient(goal, self.__orient.copy(), q_pre)
			print q
			if q is None:
				self.__state = "skip"
				return
			# q = q[1:]
			# q[0] -= 3*np.pi/180.0
			command = self.getReachPosCommand(q, ignoreObstacles=True)
			self.rosInterface.pathPlaning.setGoal(**command)
			self.__state = "offset1"
			self.__isfinish = True

		elif self.__state == "skip":
			rospy.loginfo("ReachPosition. Release gripper. Skip step.")
			self.rosInterface.pathPlaning.setGoal(command="release_gripper")
			self.__isfinish = True

	@property
	def isFinish(self):
		return self.__isfinish