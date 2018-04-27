#!/usr/bin/env python
import numpy as np

from brain.brainState import FSMBrainState
from brain.SuchartInterface import SuchartRosInterface

from utility.performanceX import Timer
from utility.performanceX import StopWatch

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point32

from math import radians as rad
import math

import time
import rospy

JOINTNAME = ["joint"+str(i+1) for i in range(6)]
PANTILTNAME = ["pan", "tilt"]
PANTILTVELO = [math.radians(45), math.radians(45)]

PI = math.pi

class ScanWorkSpace(FSMBrainState):
	tilt,pan = np.indices((7,13))
	tilt = (tilt*90/6).reshape(7,13,1)
	pan = ((pan-6)*90/6).reshape(7,13,1)
	SCAN_PATTERN = np.concatenate((pan,tilt), axis=2).reshape(-1,2)
	for i,p in enumerate(SCAN_PATTERN):
		if (p[1]/15)%2 == 1:
			SCAN_PATTERN[i,0] = -SCAN_PATTERN[i,0]
	SCAN_PATTERN[:,1] *= -1
	SCAN_PATTERN = np.deg2rad(SCAN_PATTERN)

	def __init__(self, nextstate = None):
		super(ScanWorkSpace, self).__init__("ScanWorkSpace")
		self.nextstate = nextstate

		self.__watch = StopWatch()

		self.__isFinish = False
		# TODO : change this config to correct config.
		turnbackPos = [-PI,0,5*PI/6,0,0,0]
		##############################
		self.__turnBackCommand = {	"goalPosition":JointState(name=JOINTNAME,
														position=turnbackPos),
									"command":"reach_position"}

		# TODO : change this config to appropriate config.
		# self.__scanPattern = [ [rad(j), rad(i)] for i in range(0,-91,-15) for j in range(-90,91,15)]
		##############################
		self.__maxI = len(self.SCAN_PATTERN)
		self.__i = 0

		self.__timer = Timer()

		self.__state = 0

		self.__coorBoard = []

	def dist(self, p1, p2):
		return math.sqrt( (p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2 )

	def __addNewCooboard(self, center, label):
		minDist = 0.05 #0.5 decimetre or 0.05 metre or 5 centimetre
		found = False
		I = None
		for i,v in  enumerate(self.__coorBoard):
			cen, l, n = v
			dist = self.dist(cen, center)
			if dist < minDist:
				found = True
				minDist = dist
				I = i

		if found:
			cen, l, n = self.__coorBoard[I]
			x = (cen.x*n + center.x)/(n+1)
			y = (cen.y*n + center.y)/(n+1)
			z = (cen.z*n + center.z)/(n+1)

			# print n,cen.y,center.y,y

			l.append(label)
			n += 1
			self.__coorBoard[I] = (Point32(x=x,y=y,z=z),l,n)

		if not found:
			self.__coorBoard.append((center, [label], 1))
		print self.__coorBoard
	def findMode(self, l):
		return max(set(l), key=l.count)

	def finishingCoorBoard(self):
		if len(self.__coorBoard) > 10:
			self.__coorBoard = sorted(	self.__coorBoard, 
										key=lambda x:x[2],
										reverse = True)[:10]
		else:
			rospy.logwarn("Found only "+str(len(self.__coorBoard))+"/10.")

		out = []
		for cen,l,n in self.__coorBoard:
			l_mode = self.findMode(l)
			out.append((cen, l_mode))

		return out

	@property
	def coorBoard(self):
		return self.__coorBoard

	@property
	def isFinish(self):
		return self.__isFinish

	def firstStep(self):
		rospy.loginfo("ScanWorkSpace")
		self.__isFinish = False
		# self.rosInterface.pathPlaning.waitForServer()
		# self.rosInterface.pathPlaning.setGoal(**self.__turnBackCommand)

		self.__timer.reset()
		self.__state = 0
		self.__i = 0
		self.__coorBoard = []
		self.__watch.start()

	def step(self):
		if not self.rosInterface.pathPlaning.isFinish:
			return

		if self.__isFinish:
			self.__watch.stop()
			self.rosInterface.Pantilt(	name=PANTILTNAME,
										position=[0,0],
										velocity=PANTILTVELO,
										command=0)
			rospy.loginfo(str(self.__watch.getElapseTime()))
			# rospy.loginfo(str(self.__coorBoard))
			if self.nextstate is None:
				return
			self.SignalChangeSubBrain(self.nextstate)
			return

		if self.__state == 0:

			pantiltConfig = self.SCAN_PATTERN[self.__i]

			self.rosInterface.Pantilt(	name=PANTILTNAME,
										position=pantiltConfig,
										velocity=PANTILTVELO,
										command=0)

			self.__timer.start(1.0)
			self.__state = 1
			self.__i += 1

		elif self.__state == 1 and self.__timer.is_timeUp():
			self.__timer.reset()
			## record board position here.
			postDict = self.rosInterface.visionManager
			# print postDict.world_coor, postDict.labels
			for cen, label in zip(postDict.world_coor, postDict.labels):
				self.__addNewCooboard(cen, label)

			self.__state = 0

			if self.__i == self.__maxI:
				self.__isFinish = True