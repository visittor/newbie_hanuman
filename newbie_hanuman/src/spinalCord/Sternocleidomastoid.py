#!/usr/bin/env python

from DynamixelServo import AX12A, MX28, RX28

import numpy as np 

import rospy
import roslib

from sensor_msgs.msg import JointState
from testframework.srv import PanTiltService
from cell.nodeCell import NodeBase

import serial
import time
import math
from Queue import Queue

STRING_TO_CLASS_MOTOR_SERIES = {	"ax12a"	:	AX12A,
									"mx28"	:	MX28,
									"rx28"	:	RX28
								}

##############################
##		Helper function		##
##############################
def getJsPosFromName(Js, name):
	return Js.position[Js.name.index(name)]

def getJsVelFromName(Js, name):
	return Js.velocity[Js.name.index(name)]
##############################
##			 END			##
##############################

class Sternocleidomastoid(NodeBase):

	NAME_MOTOR = ["pan", "tilt"]

	def __init__(self):
		super(Sternocleidomastoid, self).__init__("sternocleidomastoid")
		# self.__nameSpace = rospy.get_namespace()

		# self.__panMotorID = rospy.get_param(self.__nameSpace+'/spinalcord/pan_motor_id', 41)
		# self.__tiltMotorID = rospy.get_param(self.__nameSpace+'/spinalcord/tilt_motor_id', 42)
		
		# self.__comPort = rospy.get_param(self.__nameSpace+'/spinalcord/head_comport', "/dev/tty/USB0")
		# self.__baudrate = rospy.get_param(self.__nameSpace+'/spinalcord/head_baurdrate', 115200)
		# self.__rts = rospy.get_param(self.__nameSpace+'/spinalcord/head_rts', 1)
		# self.__dtr = rospy.get_param(self.__nameSpace+'/spinalcord/head_dtr', 1)
		self.__panMotorID = self.getParam(self.nameSpace+'spinalcord/pan_motor_id', 41)
		self.__tiltMotorID = self.getParam(self.nameSpace+'spinalcord/tilt_motor_id', 42)
		
		self.__comPort = self.getParam(self.nameSpace+'spinalcord/head_comport', "/dev/ttyUSB0")
		self.__baudrate = self.getParam(self.nameSpace+'/spinalcord/head_baurdrate', 115200)
		self.__rts = self.getParam(self.nameSpace+'spinalcord/head_rts', 1)
		self.__dtr = self.getParam(self.nameSpace+'spinalcord/head_dtr', 1)
		self.rosInitNode()

		self.__serial = serial.Serial()
		self.__connectSerialPort()

		self.__panMotorSeries = STRING_TO_CLASS_MOTOR_SERIES[self.getParam(self.nameSpace+'spinalcord/pan_motor_serie', 'ax12a').lower()]
		self.__tiltMotorSeries = STRING_TO_CLASS_MOTOR_SERIES[self.getParam(self.nameSpace+'spinalcord/tilt_motor_serie', 'ax12a').lower()]

		self.__panAngleMax = self.getParam(self.nameSpace+'spinalcord/pan_angle_max', math.radians(135.0))
		self.__panAngleMin = self.getParam(self.nameSpace+'spinalcord/pan_angle_min', math.radians(-135.0))
		self.__tiltAngleMax = self.getParam(self.nameSpace+'spinalcord/tilt_angle_max', math.radians(80.0))
		self.__tiltAngleMin = self.getParam(self.nameSpace+'spinalcord/tilt_angle_min', math.radians(-10.0))

		self.__panDirection = self.getParam(self.nameSpace+'spinalcord/pan_direction', 1)
		self.__tiltDirection = self.getParam(self.nameSpace+'spinalcord/tilt_direction', 1)

		self.__timeout = self.getParam(self.nameSpace+'spinalcord/head_serial_timeout',0.1)

		self.__panMotor = self.__panMotorSeries(	self.__serial, 
													self.__panMotorID,
													maxAngleRads=self.__panAngleMax,
													minAngleRads=self.__panAngleMin,
													timeout=self.__timeout,
													Direction=self.__panDirection)
		self.__tiltMotor = self.__tiltMotorSeries(	self.__serial,
													self.__tiltMotorID,
													maxAngleRads=self.__tiltAngleMax,
													minAngleRads=self.__tiltAngleMin,
													timeout=self.__timeout,
													Direction=self.__tiltDirection)
		self.setFrequencyFromParam(self.nameSpace+'spinalcord/Sternocleidomastoid_frquency')

		self.__stateQueue = Queue()
		self.__lastestState = JointState()

		self.__initServos()
		self.__rosInitSubPubSer()

	def __connectSerialPort(self):
		if self.__serial.is_open:
			self.__serial.close()
		self.__serial.port = str( self.__comPort )
		self.__serial.baudrate = int( self.__baudrate )
		self.__serial.setDTR(self.__dtr)
		self.__serial.setRTS(self.__rts)
		try:
			self.__serial.open()
		except serial.SerialException as e:
			print "Error"
			rospy.logwarn(str(e))
		time.sleep(0.0002)
		if not self.__serial.is_open:
			rospy.logwarn("Serial port for head ("+str(self.__comPort)+") not be opened.")
		else:
			rospy.loginfo("Connected to "+str(self.__comPort)+".")

	def set_serialParam(self, comport=None, baudrate=None, rts=None, dtr=None):
		self.__comPort = str(comport) if comport is not None else self.__comPort
		self.__baudrate = int(baudrate) if baudrate is not None else self.__baudrate
		self.__rts = rts if rts is not None else self.__rts
		self.__dtr = dtr if dtr is not None else self.__dtr
		self.__connectSerialPort()

	def __initServos(self):
		self.__pingMotor()
		self.__setCenter()

	def __pingMotor(self):
		if self.__panMotor.ping_this_motor() is None:
			rospy.logwarn("Cannot ping motor id "+str(self.__panMotorID)+". (pan motor)")
		else:
			rospy.loginfo("Pan motor connected.")
		if self.__tiltMotor.ping_this_motor() is None:
			rospy.logwarn("Cannot ping motor id "+str(self.__tiltMotorID)+". (tilt motor)")
		else:
			rospy.loginfo("Tilt motor connected.")

	def __setCenter(self):
		retPan = 0 if self.__panMotor.moveFromRad(0, 0) is None else 1
		retTilt = 0 if self.__tiltMotor.moveFromRad(0, 0) is None else 1
		js = JointState(	name 	=	self.NAME_MOTOR,
							position=	[0,0],
							velocity=	[0,0],
							effort  =	[retPan, retTilt])
		self.__stateQueue.put(js)

	def __rosInitNode(self):
		rospy.init_node("sternocleidomastoid", anonymous = True, log_level=rospy.DEBUG)

	def __rosInitSubPubSer(self):
		self.rosInitSubscriber(	"/spinalcord/sternocleidomastoid_command", 
								JointState,
								self.callback,
								queue_size = 10)
		self.rosInitPublisher(	'/spinalcord/sternocleidomastoid_position',
								JointState,
								queue_size = 10)
		self.rosInitService(	"sternocleidomastoid_service", 
								PanTiltService,
								self.handleService)

	def handleService(self, req):
		js = self.__lastestState
		return PanTiltService(js)

	def callback(self, msg):
		panPosition = getJsPosFromName(msg, "pan")
		rospy.logdebug(str(panPosition))
		panSpeed = getJsVelFromName(msg, "pan")
		tiltPosition = getJsPosFromName(msg, "tilt")
		rospy.logdebug(str(tiltPosition))
		tiltSpeed = getJsVelFromName(msg, "tilt")
		retPan = 0 if self.__panMotor.moveFromRad_REGACTION(panPosition, panSpeed) is None else 1
		retTilt = 0 if self.__tiltMotor.moveFromRad_REGACTION(tiltPosition, tiltSpeed) is None else 1

		self.__panMotor.broadcastingAction()

		self.__putMessageToQueue(panPosition, panSpeed, tiltPosition, tiltSpeed, retPan, retTilt)

	def __putMessageToQueue(self, panPos, panSpd, tiltPs, tiltSp, retPan, retTil):
		position = [self.__panMotor.clampAngleRad(panPos),
					self.__tiltMotor.clampAngleRad(tiltPs)]
		speed = [panSpd, tiltSp]
		effort = [retPan, retTil]
		js = JointState(name 	=	self.NAME_MOTOR,
						position=	position,
						velocity=	speed,
						effort  =	effort)
		self.__stateQueue.put(js)

	def run(self):
		rospy.loginfo("Start sternocleidomastoid node.")
		while not rospy.is_shutdown():
			if not self.__stateQueue.empty():
				self.__lastestState = self.__stateQueue.get()
				self.publish(self.__lastestState)
				self.__stateQueue.task_done()
			self.sleep()
		rospy.loginfo("Close sternocleidomastoid node.")

	def end(self):
		self.__serial.close()
		time.sleep(0.001)
		if self.__serial.is_open:
			rospy.logwarn("Cannot close "+str(self.__serial.port))

def main():
	node = Sternocleidomastoid()
	try:
		node.run()
	except rospy.ROSInterruptException:
		node.end()
	finally:
		node.end() 