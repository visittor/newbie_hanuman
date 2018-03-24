#!/usr/bin/env python
import pygame

import rospy

from newbie_hanuman.srv import MotorCortexCommand,MotorCortexCommandResponse
from newbie_hanuman.srv import reqMotorCortexIntegration, reqMotorCortexIntegrationResponse
from newbie_hanuman.msg import PanTiltCommand

from cell.nodeCell import NodeBase

import os
import math

BUTTON_MAP = { 	'A' 		:		0,
				'B' 		:		1,
				'X'			:		2,
				'Y'			:		3,
				'LB'		:		4,
				'RB'		:		5,
}

ANALOG_MAP = {	'y'			:		0,
				'x'			:		1,
				'LT'		:		2,
				'z'			:		3,
				'z2'		:		4,
				'RT'		:		5,
}

class Controller(object):
	name = ["pan", "tilt"]
	position = [0,0]
	velocity = [20,20]
	command = [0,0]
	ang = math.radians(10)

	def __init__(self):
		self.__rate = None
		self.__motorCortexCommand = None
		self.__joyStick = None
		
		self.axisData = {}
		self.buttonData = {}
		self.hatData = {}

		self.initRos()
		self.initPygame()

	def initPygame(self):
		pygame.init()
		pygame.joystick.init()
		self.joyStick = pygame.joystick.Joystick(0)
		self.joyStick.init()

	def initRos(self):
		rospy.init_node("hanuman_controller", 
						anonymous = False, 
						log_level = rospy.INFO)
		self.__rate = rospy.Rate(5)
		self.__ros_pub = rospy.Publisher(
									"/spinalcord/sternocleidomastoid_command",
									PanTiltCommand,
									)
		rospy.wait_for_service('motor_cortex_command')
		self.__motorCortexCommand = rospy.ServiceProxy("motor_cortex_command",
													MotorCortexCommand)

	def __clearJoyStickData(self):
		for k,v in self.buttonData.items():
			self.buttonData[k] = False

		for k,v in self.axisData.items():
			self.axisData[k] = 0.0

		for k,v in self.buttonData.items():
			self.buttonData[k] = (0, 0)

	def __handleInput(self):
		## special command
		if self.buttonData[BUTTON_MAP['X']]:
			self.__motorCortexCommand(	velX = 0,
										velY = 0,
										omgZ = 0,
										commandType = 1,
										command = 'LeftKick',
										ignorable = False)
			return
		if self.buttonData[BUTTON_MAP['B']]:
			self.__motorCortexCommand(	velX = 0,
										velY = 0,
										omgZ = 0,
										commandType = 1,
										command = 'RightKick',
										ignorable = False)
			return
		if self.buttonData[BUTTON_MAP['LB']]:
			self.__motorCortexCommand(	velX = 0,
										velY = 0,
										omgZ = 0,
										commandType = 1,
										command = 'GoalieSaveLeft',
										ignorable = False)
			return
		if self.buttonData[BUTTON_MAP['RB']]:
			self.__motorCortexCommand(	velX = 0,
										velY = 0,
										omgZ = 0,
										commandType = 1,
										command = 'GoalieSaveRight',
										ignorable = False)
			return

		## Locomotion command
		position = [-self.hatData[0][0]*self.ang, -self.hatData[0][1]*self.ang]
		c = PanTiltCommand(	name 		= self.name,
							position 	= position,
							velocity	= self.velocity,
							command 	= self.command)
		self.__ros_pub.publish(c)

		# else:
		gainLin = 0.5*(1.0 + self.axisData[ANALOG_MAP['LT']]) + 1
		gainAng = 0.5*(1.0 + self.axisData[ANALOG_MAP['RT']]) + 1
		xVel = -1*gainLin*(0.5*self.axisData[ANALOG_MAP['x']])
		yVel = -1*gainLin*(0.5*self.axisData[ANALOG_MAP['y']])
		omgZ = -1*gainAng*(0.5*self.axisData[ANALOG_MAP['z']])

		xVel = 0.0 if -0.01 <= xVel <= 0.01 else xVel
		yVel = 0.0 if -0.01 <= yVel <= 0.01 else yVel
		omgZ = 0.0 if -0.01 <= omgZ <= 0.01 else omgZ

		self.__motorCortexCommand(	velX = xVel,
									velY = yVel,
									omgZ = omgZ,
									commandType = 0,
									command = '',
									ignorable = True)
		return

	def run(self):
		rospy.loginfo("Start Node Controller.")

		for i in range(self.joyStick.get_numaxes()):
			self.axisData[i] = 0.0

		for i in range(self.joyStick.get_numbuttons()):
			self.buttonData[i] = False

		for i in range(self.joyStick.get_numhats()):
			self.hatData[i] = (0, 0)

		while not rospy.is_shutdown():
			## Event handle.
			for event in pygame.event.get():
				if event.type == pygame.JOYBUTTONDOWN:
					self.buttonData[event.button] = True
				elif event.type == pygame.JOYBUTTONUP:
					self.buttonData[event.button] = False
				elif event.type == pygame.JOYAXISMOTION:
					self.axisData[event.axis] = round(event.value, 1)
				elif event.type == pygame.JOYHATMOTION:
					self.hatData[event.hat] = event.value

			try:
				self.__handleInput()
			except rospy.ServiceException as exc:
				print("Service did not process request: " + str(exc))

			# print "Button data"
			# print self.buttonData
			# print "Axis data"
			# print self.axisData
			# print "Hat data"
			# print self.hatData

			# os.system("clear")

			self.__rate.sleep
		rospy.loginfo("Exit node Controller.")
	def end(self):
		pass

def main():
	node = Controller()
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