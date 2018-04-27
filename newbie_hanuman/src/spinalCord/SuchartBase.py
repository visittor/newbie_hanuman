#!/usr/bin/env python

from RobotisProtocolBase import RobotisProtocol

import rospy

RegisterNames = [
					"GOAL_POS_JOINT1_L",
					"GOAL_POS_JOINT1_H",
					"GOAL_POS_JOINT2_L",
					"GOAL_POS_JOINT2_H",
					"GOAL_POS_JOINT3_L",
					"GOAL_POS_JOINT3_H",
					"GOAL_POS_JOINT4_L",
					"GOAL_POS_JOINT4_H",
					"GOAL_POS_JOINT5_L",
					"GOAL_POS_JOINT5_H",
					"GOAL_POS_JOINT6_L",
					"GOAL_POS_JOINT6_H",

					"CURR_POS_JOINT1_L",
					"CURR_POS_JOINT1_H",
					"CURR_POS_JOINT2_L",
					"CURR_POS_JOINT2_H",
					"CURR_POS_JOINT3_L",
					"CURR_POS_JOINT3_H",
					"CURR_POS_JOINT4_L",
					"CURR_POS_JOINT4_H",
					"CURR_POS_JOINT5_L",
					"CURR_POS_JOINT5_H",
					"CURR_POS_JOINT6_L",
					"CURR_POS_JOINT6_H",

					"IS_REACH_POS",
					"GRIPPER_STATUS",
					"GAME_STATE"]

RegisterAddrDict = {}

for i,reg in enumerate(RegisterNames):
	RegisterAddrDict[reg] = i

Suchart_ID = 1

GripperStatus_active = 1
GripperStatus_release = 0

Reach_pos = 1
Not_reach_pos = 0

GameState_play = 1
GameState_initial = 0

class SuchartStatus(object):
	def __init__(self, statusPackage):
		self.goalPosition = None
		self.currPosition = None

		self.is_reach_pos = False
		
		self.gripper_active = None
		self.gripper_release = None

		self.gameStatusInitialState = None
		self.gameStatusPlayState = None

		self.comunication_OK = False

		if statusPackage is not None:
			self.__initial(statusPackage)

	def __initial(self, statusPackage):
		startCurrPosAddr = RegisterAddrDict["CURR_POS_JOINT1_L"]
		stopCurrPosAddr = RegisterAddrDict["CURR_POS_JOINT6_H"] + 1

		startGoalPosAddr = RegisterAddrDict["GOAL_POS_JOINT1_L"]
		stopGaolPosAddr = RegisterAddrDict["GOAL_POS_JOINT6_H"] + 1

		joint_names, goal_pos = self.createJointState(statusPackage[startGoalPosAddr:stopGaolPosAddr], 6)
		joint_names, curr_pos = self.createJointState(statusPackage[startCurrPosAddr:stopCurrPosAddr], 6)

		self.goalPosition ={name:pos for name,pos in zip(joint_names,goal_pos)}
		self.currPosition ={name:pos for name,pos in zip(joint_names,curr_pos)}
		self.is_reach_pos = statusPackage[RegisterAddrDict["IS_REACH_POS"]] == Reach_pos

		self.gripper_active = statusPackage[RegisterAddrDict["GRIPPER_STATUS"]] == GripperStatus_active
		self.gripper_release = statusPackage[RegisterAddrDict["GRIPPER_STATUS"]] == GripperStatus_release
		self.gameStatusInitialState = statusPackage[RegisterAddrDict["GAME_STATE"]] == GameState_initial
		self.gameStatusPlayState = statusPackage[RegisterAddrDict["GAME_STATE"]] == GameState_play
		self.comunication_OK = True

	@staticmethod
	def createJointState(package, njoint):
		names = []
		positions = []
		for i in range(njoint):
			name = "joint"+str(i+1)
			position = package[2*i] + package[2*i + 1]*256
			names.append(name)
			positions.append(position)
		return names, positions


class SuchartProtocol(RobotisProtocol):

	def __init__(self, serial):
		super(SuchartProtocol, self).__init__(serial)
		self.__robotStatus = None

	def read_AllLowLevelData(self, timeout = None):
		numB = len(RegisterNames)
		robotStatus_package = self.readRegister(Suchart_ID, 
										RegisterAddrDict["GOAL_POS_JOINT1_L"],
										numB,
										timeout = timeout)
		if robotStatus_package is None:
			self.__robotStatus = SuchartStatus(robotStatus_package)
			return False

		if len(robotStatus_package) >= 6+numB:
			robotStatus_package = robotStatus_package[5:-1]
			self.__robotStatus = SuchartStatus(robotStatus_package)
			return True

		else:
			self.__robotStatus = SuchartStatus(None)
			return False

	def get_suchartStatus(self):
		return self.__robotStatus

	def get_reachPosStatus(self, timeout = None):
		package = self.readRegister(Suchart_ID, 
									RegisterAddrDict["IS_REACH_POS"],
									1,
									timeout = timeout)
		if package is None:
			return None
		elif len(package) == 6+1:
			return package[5] == Reach_pos
		else:
			return None

	def get_GripperStatus(self, timeout = None):
		package = self.readRegister(Suchart_ID, 
									RegisterAddrDict["GRIPPER_STATUS"],
									1,
									timeout = timeout)
		if package is None:
			return None
		elif len(package) == 6+1:
			return package[5]
		else:
			return None

	def get_gameState(self, timeout = None):
		package = self.readRegister(Suchart_ID,
									RegisterAddrDict["GameState_initial"],
									1,
									timeout = timeout)
		if package is None:
			return None
		elif len(package) == 6+1:
			return package[5]
		else:
			return None

	def get_CurrentPosition(self, timeout = None):
		package = self.readRegister(Suchart_ID,
									RegisterAddrDict["CURR_POS_JOINT1_L"],
									12,
									timeout = timeout)
		names, joints = SuchartStatus.createJointState(package, 6)
		return {n:j for n,j in zip(names, joints)}

	def get_GoalPosition(self, timeout = None):
		package = self.readRegister(Suchart_ID,
									RegisterAddrDict["GOAL_POS_JOINT1_L"],
									12,
									timeout = timeout)
		names, joints = SuchartStatus.createJointState(package, 6)
		return {n:j for n,j in zip(names, joints)}

	@staticmethod
	def is_reachPosition(status):
		if status is None:
			return None
		return status == Reach_pos

	@staticmethod
	def is_gripperActive(status):
		if status is None:
			return None
		return status == GripperStatus_active

	@staticmethod
	def is_initialState(status):
		if status is None:
			return None
		return status == GameState_initial

	def set_goalPosition(self, position, timeout = None):
		paramN = [	RegisterAddrDict["GOAL_POS_JOINT1_L"]]
		for i in range(6):
			paramN.append(int(position[i])%256)
			paramN.append(int(position[i])/256)
		return self.writeNBytes(Suchart_ID, 
								paramN, timeout = timeout)

	def set_gripper(self, status, timeout = None):
		paramN = [RegisterAddrDict["GRIPPER_STATUS"], status]
		return self.writeNBytes(Suchart_ID, paramN, timeout = timeout)

class SuchartInterface(object):

	def __init__(self, serial, joint1LimitReg, joint2LimitReg, joint3LimitReg, joint4LimitReg, joint5LimitReg, joint6LimitReg, joint1LimitAng, joint2LimitAng, joint3LimitAng, joint4LimitAng, joint5LimitAng, joint6LimitAng, timeout = None):

		self.__jointLimitReg = {}
		self.__jointLimitReg["joint1"] = joint1LimitReg
		self.__jointLimitReg["joint2"] = joint2LimitReg
		self.__jointLimitReg["joint3"] = joint3LimitReg
		self.__jointLimitReg["joint4"] = joint4LimitReg
		self.__jointLimitReg["joint5"] = joint5LimitReg
		self.__jointLimitReg["joint6"] = joint6LimitReg

		self.__jointLimitAng = {}
		self.__jointLimitAng["joint1"] = joint1LimitAng
		self.__jointLimitAng["joint2"] = joint2LimitAng
		self.__jointLimitAng["joint3"] = joint3LimitAng
		self.__jointLimitAng["joint4"] = joint4LimitAng
		self.__jointLimitAng["joint5"] = joint5LimitAng
		self.__jointLimitAng["joint6"] = joint6LimitAng

		self.__angPerReg = {}
		self.__regPerAng = {}
		for n,v in self.__jointLimitReg.items():
			limAng = self.__jointLimitAng[n]
			self.__angPerReg[n] = float(limAng[1]-limAng[0]) / (v[1]-v[0])
			self.__regPerAng[n] = float(v[1]-v[0]) / (limAng[1]-limAng[0])

		self.__timeout = timeout

		self.__spinal_cord = SuchartProtocol(serial)

		self.__robotStatus = None

	def angle2reg(self, value):
		reg = {}
		for n,v in value.items():
			reg[n] = ((v-self.__jointLimitAng[n][0])*self.__regPerAng[n])\
					 +self.__jointLimitReg[n][0]
		return reg

	def reg2ang(self, value):
		ang = {}
		for n,v in value.items():
			ang[n] = ((v-self.__jointLimitReg[n][0])*self.__angPerReg[n])\
					 +self.__jointLimitAng[n][0]
		return ang

	def request_suchartStatus(self):
		ret = self.__spinal_cord.read_AllLowLevelData(timeout = self.__timeout)
		self.__robotStatus = self.__spinal_cord.get_suchartStatus()
		return ret, self.__robotStatus

	def set_GoalPosition(self, pos):
		actualPosition = {}
		pos = self.angle2reg(pos)
		for n, v in pos.items():
			actualPosition[n] = min(max(v, self.__jointLimitReg[n][0]),
									self.__jointLimitReg[n][1])
		posList = []
		for i in range(len(actualPosition)):
			jointName = "joint"+str(i+1)
			posList.append(actualPosition[jointName])

		if self.__spinal_cord.set_goalPosition(posList) is not None:
			return True

		return False

	def set_GripperActive(self):
		self.__spinal_cord.set_gripper(1, self.__timeout)

	def set_GripperRelease(self):
		self.__spinal_cord.set_gripper(0, self.__timeout)

	def get_suchartStatus(self):
		return self.__robotStatus