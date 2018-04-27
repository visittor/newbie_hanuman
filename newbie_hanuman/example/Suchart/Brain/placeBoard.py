#!/usr/bin/env python
from brain.brainState import FSMBrainState
from brain.SuchartInterface import SuchartRosInterface

from geometry_msgs.msg import Point32

class PlaceBoards(FSMBrainState):

	GOAL = [Point32(x=1.0, y=-0.3, z=0.7),
			Point32(x=1.0, y=-0.1, z=0.7),
			Point32(x=1.0, y= 0.1, z=0.7),
			Point32(x=1.0, y= 0.3, z=0.7),
			Point32(x=1.0, y=-0.3, z=0.5),
			Point32(x=1.0, y=-0.1, z=0.5),
			Point32(x=1.0, y= 0.1, z=0.5),
			Point32(x=1.0, y= 0.3, z=0.5),
			Point32(x=1.0, y=-0.1, z=0.3),
			Point32(x=1.0, y= 0.1, z=0.3)]

	def __init__(self, nextstate=None):
		super(PlaceBoards, self).__init__(PlaceBoards)

		self.nextstate = nextstate

		self.__isfinish = False

		self.__coorBoard = []

		self.__state = 0

	def set_boardCoor(self, boardCoor):
		self.__coorBoard = boardCoor
		self.__coorBoard = sorted(self.__coorBoard, key=lambda x:x[1])

	@property
	def isFinish(self):
		return self.__isfinish

	def firstStep(self):
		self.__isfinish = True

	def step(self):
		if len(self.__coorBoard) == 0 or len(self.__GOAL)==0:
			self.__isfinish = True
			return

		if self.rosInterface.pathPlaning.isFinish:
			return

		if self.__state == 0:
			coor = self.__coorBoard.pop(0)
			# TODO : use invert kinematic to convert task space to joint space.
			# TODO : command path planing.
			self.__state = 1

		elif self.__state == 1:
			coor - self.__GOAL.pop(0)
			# TODO : use invert kinematic to convert task space to joint space.
			# TODO : comand path planing,
			self.__state = 0