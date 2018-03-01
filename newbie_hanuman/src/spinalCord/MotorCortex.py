#!/usr/bin/env python
from HanumanInterface import HanumanInterface, LocomotionCommand,LocomotionSpecialCommand

import rospy
import roslib

from newbie_hanuman.srv import MotorCortexCommand,MotorCortexCommandResponse
from newbie_hanuman.srv import reqMotorCortexIntegration, reqMotorCortexIntegrationResponse

from cell.nodeCell import NodeBase

import time
import math
import serial

class MotorCortex(NodeBase):

	def __init__(self):
		super(MotorCortex, self).__init__("motor_cortex")

		self.__XWorldTranMax = self.getParam(self.nameSpace+'motor_cortex/XWorldTranMax', 0.18)
		self.__XWorldTranMin = self.getParam(self.nameSpace+'motor_cortex/XWorldTranMin', -0.18)
		self.__YWorldTranMax = self.getParam(self.nameSpace+'motor_cortex/YWorldTranMax',0.07)
		self.__YWorldTranMin = self.getParam(self.nameSpace+'motor_cortex/YWorldTranMin', -0.07)
		self.__ZWorldAnglMax = self.getParam(self.nameSpace+'motor_cortex/ZWorldAnglMax', math.radians(16.5))
		self.__ZWorldAnglMin = self.getParam(self.nameSpace+'motor_cortex/ZWorldAnglMin', math.radians(-16.5))
		self.__XRegisTranMax = self.getParam(self.nameSpace+'motor_cortex/XRegisTranMax',167)
		self.__XRegisTranMin = self.getParam(self.nameSpace+'motor_cortex/XRegisTranMin',87)
		self.__YRegisTranMax = self.getParam(self.nameSpace+'motor_cortex/YRegisTranMax',167)
		self.__YRegisTranMin = self.getParam(self.nameSpace+'motor_cortex/YRegisTranMin',87)
		self.__ZRegisAnglMax = self.getParam(self.nameSpace+'motor_cortex/ZRegisAnglMax',147)
		self.__ZRegisAnglMin = self.getParam(self.nameSpace+'motor_cortex/ZRegisAnglMin',107)

		self.__timeout = self.getParam(	self.nameSpace+'motor_cortex/timeout',
										0.1)
		self.__timeTolerance = self.getParam(self.nameSpace+'motor_cortex/timeTolerance',1)

		self.__comPort = self.getParam(self.nameSpace+'motor_cortex/comport','/dev/ttyACM0')
		self.__baudrate = self.getParam(self.nameSpace+'motor_cortex/baudrate',
										115200)
		self.__rts = self.getParam(self.nameSpace+'motor_cortex/rts',0)
		self.__dtr = self.getParam(self.nameSpace+'motor_cortex/dtr',0)

		self.rosInitNode()

		self.__serial = serial.Serial()
		self.__connectSerialPort()

		self.__hanumanInterface = HanumanInterface(self.__serial, 
													self.__XWorldTranMax,
													self.__XWorldTranMin,
													self.__YWorldTranMax,
													self.__YWorldTranMin,
													self.__ZWorldAnglMax,
													self.__ZWorldAnglMin,
													self.__XRegisTranMax,
													self.__XRegisTranMin,
													self.__YRegisTranMax,
													self.__YRegisTranMin,
													self.__ZRegisAnglMax,
													self.__ZRegisAnglMin,
													timeout = self.__timeout,
										timeTolerance = self.__timeTolerance)
		self.setFrequencyFromParam(self.nameSpace+'motor_cortex/MotorCortex_frquency')

		self.__lastResetIntegration = rospy.Time.now()
		self.__lastFallDown = rospy.Time.now()

		self.__initLocomotion()
		self.__rosInitSubPubSer()

	def __connectSerialPort(self):
		if self.__serial.is_open:
			self.__serial.close()
		self.__serial.port = str( self.__comPort )
		self.__serial.baudrate = int( self.__baudrate )
		self.__serial.setDTR( self.__dtr )
		self.__serial.setRTS( self.__rts )
		try:
			self.__serial.open()
		except serial.SerialException as e:
			rospy.logwarn(str(e))
		time.sleep(0.02)
		if not self.__serial.is_open:
			rospy.logwarn("Serial port for locomotion ("+str(self.__comPort)+") cannot be opened.")
		else:
			self.__serial.flushInput()
			self.__serial.flushOutput()
			rospy.loginfo("Connected to "+str(self.__comPort)+".")

	def __initLocomotion(self):
		self.__hanumanInterface.forceStop()
		self.__hanumanInterface.doCurrentCommand()
		self.__hanumanInterface.clearIntegration()

	def __rosInitSubPubSer(self):
		self.rosInitService("motor_cortex_command", 
							MotorCortexCommand,
							self.__receiveCommand)

		rospy.Service(	"request_integration",
						reqMotorCortexIntegration,
						self.__reqIntegration)

		self.setFrequency(1.5)
	def __receiveCommand(self, req):
		if req.commandType == req.LOCOMOTIONCOMMAND:
			vel_x = req.velX
			vel_y = req.velY
			omg_z = req.omgZ
			command = LocomotionCommand(vel_x, vel_y, omg_z)
			self.__hanumanInterface.getCommand(	command, 
												timeStamp = time.time(),
												ignorable= req.ignorable)

		elif req.commandType == req.SPECIALCOMMAND:
			commandString = req.command
			command = LocomotionSpecialCommand(commandString)
			self.__hanumanInterface.getCommand(	command, 
												timeStamp=time.time(),
												ignorable = req.ignorable)

		return MotorCortexCommandResponse(True)

	def __reqIntegration(self, req):
		if req.resetIntegration:
			self.__hanumanInterface.clearIntegration()
			self.__lastResetIntegration = rospy.Time.now()
		x,y,theta = self.__hanumanInterface.doIntegration()
		response = reqMotorCortexIntegrationResponse(x = x,
													 y = y,
													 theta = theta,
											lastFallDown = self.__lastFallDown,
							lastResetIntegration = self.__lastResetIntegration)
		return response

	def run(self):
		rospy.loginfo("Start motor cortex node.")
		while not rospy.is_shutdown():
			if not self.__hanumanInterface.is_robotStanding():
				rospy.loginfo("Robot fall down. Reset integration and force standup.")
				# time.sleep(0.1)
				self.__lastFallDown = rospy.Time.now()
				self.__hanumanInterface.forceStandup()
				self.__hanumanInterface.clearIntegration()
				self.__lastResetIntegration = rospy.Time.now()
			else:
				# time.sleep(0.1)
				rospy.loginfo("Do current command.")
				self.__hanumanInterface.doCurrentCommand()
			self.sleep()
		rospy.loginfo("Close motor cortex node.")

	def end(self):
		self.__hanumanInterface.forceStop()
		self.__serial.close()
		time.sleep(0.01)
		if self.__serial.is_open:
			rospy.logwarn("Cannot close "+str(self.__serial.port))

def main():
	node = MotorCortex()
	try:
		node.run()
	except rospy.ROSInterruptException as e:
		node.end()
		rospy.logwarn(str(e))
	except Exception as e:
		node.end()
		rospy.logwarn(str(e))
	finally:
		node.end()