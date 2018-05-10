#!/usr/bin/env python
import numpy as np

from SuchartBase import RegisterAddrDict, RegisterNames
from RobotisProtocolBase import RobotisProtocol

import rospy
import actionlib

from sensor_msgs.msg import JointState

from cell.nodeCell import NodeBase

import time
import serial
import math

class SuchartSerialSim(NodeBase):

	def __init__(self):
		super(SuchartSerialSim, self).__init__("suchart_serial_sim")

		self.__comPort = self.getParam(self.nameSpace+"motor_cortex/sim_serial_port", "/dev/pts/25")
		self.__baudrate = self.getParam(self.nameSpace+"motor_cortex/baudrate", 9600)

		self.__jointName = ["joint"+str(i+1) for i in range(6)]
		self.__currJointPos = [0]*6
		self.__goalJointPos = [0]*6
		self.__isReachPos = 1
		self.__gripperStatus = 0
		self.__gameState = 1
		self.__registerVal = {}
		self.attrToRegister()

		self.__rts = True
		self.__dtr = True

		self.__serial = serial.Serial(self.__comPort, self.__baudrate, rtscts = True, dsrdtr = True)

		self.rosInitNode()
		self.__rosInitPublisher()
		self.setFrequency(100)

		self.__count = 0

	def attrToRegister(self):
		self.__registerVal["IS_REACH_POS"] = self.__isReachPos
		self.__registerVal["GRIPPER_STATUS"] = self.__gripperStatus
		self.__registerVal["GAME_STATE"] = self.__gameState

		for i in range(6):
			addrL = "GOAL_POS_JOINT"+str(i+1)+"_L"
			addrH = "GOAL_POS_JOINT"+str(i+1)+"_H"
			self.__registerVal[addrL] = self.__goalJointPos[i]%256
			self.__registerVal[addrH] = (self.__goalJointPos[i]/256)%256

		for i in range(6):
			addrL = "CURR_POS_JOINT"+str(i+1)+"_L"
			addrH = "CURR_POS_JOINT"+str(i+1)+"_H"
			self.__registerVal[addrL] = self.__currJointPos[i]%256
			self.__registerVal[addrH] = (self.__currJointPos[i]/256)%256

	def registerToAttr(self):
		for i in range(6):
			currAddrL = "CURR_POS_JOINT"+str(i+1)+"_L"
			currAddrH = "CURR_POS_JOINT"+str(i+1)+"_H"

			goalAddrL = "GOAL_POS_JOINT"+str(i+1)+"_L"
			goalAddrH = "GOAL_POS_JOINT"+str(i+1)+"_H"

			self.__currJointPos[i] = self.__registerVal[currAddrL] + self.__registerVal[currAddrH]*256
			self.__goalJointPos[i] = self.__registerVal[goalAddrL] + self.__registerVal[goalAddrH]*256

		# self.__isReachPos = self.__registerVal["IS_REACH_POS"]
		self.__gripperStatus = self.__registerVal["GRIPPER_STATUS"]
		self.__gameState = self.__registerVal["GAME_STATE"]

	def __rosInitPublisher(self):
		self.rosInitPublisher("joint_states", JointState)

	def __moveJoint(self):
		self.__count = (self.__count+1)%100
		if self.__count > 10:
			step = 10
		else:
			step = 1
		isReach = 1
		jointAng = [(-np.pi, np.pi), 
					(-0.1, 0.82), 
					(-5*np.pi/6, 5*np.pi/6), 
					(-5*np.pi/6, 5*np.pi/6), 
					(-np.pi/9, 1.1*np.pi/2), 
					(-np.pi, np.pi)]
		jointReg = [(0,65535)]*6
		currMsg = [0.0]*6
		for i,n in enumerate(self.__jointName):
			diff = self.__currJointPos[i] - self.__goalJointPos[i]
			currMsg[i] = (self.__currJointPos[i]-jointReg[i][0])*(jointAng[i][1]-jointAng[i][0])/(jointReg[i][1]-jointReg[i][0]) + jointAng[i][0]
			if i==1:
				print currMsg[1]
			if diff == 0:
				pass

			elif math.fabs(diff) < 10:
				self.__currJointPos[i] += 1 if diff < 0 else -1
				isReach = 0

			elif math.fabs(diff) < 100:
				self.__currJointPos[i] += 10 if diff < 0 else -10
				isReach = 0

			elif math.fabs(diff) < 1000:
				self.__currJointPos[i] += 100 if diff < 0 else -100
				isReach = 0

			elif math.fabs(diff) < 10000:
				self.__currJointPos[i] += 100 if diff < 0 else -100
				isReach = 0

			elif diff != 0:
				self.__currJointPos[i] += 100 if diff < 0 else -100
				isReach = 0
		self.__isReachPos = isReach
		self.attrToRegister()
		# return JointState(	name = self.__jointName,
		# 					position = self.__currJointPos)
		return JointState(	name = self.__jointName,
							position = currMsg)

	def considerPackage(self, package):
		if len(package)<4 or len(package)!=package[3]+4:
			rospy.logwarn("Wrong package protocol"+str(package))
			return []
		elif package[0]!=0xff or package[1]!=0xff or package[2]!=0x01:
			rospy.logwarn("Wrong package protocol",str(package))
			return []

		if package[4] == 0x02:
			startAddr = package[5]
			nData = package[6]
			data = []
			for i in range(nData):
				addrName = RegisterNames[i+startAddr]
				data.append(self.__registerVal[addrName])
				# rospy.loginfo(addrName+" = "+str(data[i]))
			# print "\n"
			return data

		elif package[4] == 0x03:
			startAddr = package[5]
			nAddr = package[3] - 3
			data = package[6:-1]
			for i in range(nAddr):
				addrName = RegisterNames[i+startAddr]
				self.__registerVal[addrName] = data[i]
				# rospy.loginfo(addrName+" = "+str(data[i]))
			self.registerToAttr()
			return []

	def run(self):
		rospy.loginfo("Start serial sim.")
		while not rospy.is_shutdown():
			package = self.__serial.read(self.__serial.inWaiting())
			# print considerPackage
			package = map(lambda x:ord(x), list(package))
			if len(package) == 0:
				response = None
			else:
				response = self.considerPackage(package)
			if response is None:
				pass
			elif response == []:
				returnPack = [0xff,0xff,0x01,0x02,0x00,251]
				returnPack.append(RobotisProtocol.computeChecksum(returnPack))
				self.__serial.write(returnPack)
			
			else:
				returnPack = [0xff,0xff,0x01,len(response)+2,0x00]
				returnPack.extend(response)
				returnPack.append(RobotisProtocol.computeChecksum(returnPack))
				self.__serial.write(returnPack)
			for n,v in self.__registerVal.items():
				print n,v
			print "\n"
			msg = self.__moveJoint()
			msg.header.stamp = rospy.Time.now()
			self.publish(msg)
			self.sleep()
		rospy.loginfo("Exit serial sim.")

	def end(self):
		self.__serial.close()
		time.sleep(0.001)
		if self.__serial.is_open:
			rospy.logwarn("Cannot close "+str(self.__serial.port))

def main():
	node = SuchartSerialSim()
	try:
		node.run()
	except rospy.ROSInterruptException:
		pass
	finally:
		node.end()