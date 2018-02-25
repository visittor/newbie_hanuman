#!/usr/bin/env python

import serial
from Queue import Queue, Full, Empty
import time

import rospy
import roslib

def read_data(serial):
	'''
		Function for recieve data from serial port.
		Parameters
			serial		serial.Serial instance.
		Return
			list		list of int for data recieved. Empty list if not
						recieve any thing.
	'''			
	if serial is None:
		return []
	PACKAGE = []
	try:
		if not serial.is_open:
			pass
			# rospy.logwarn("Serial port is not open.")
		elif serial.is_open:
			# print serial.inWaiting()
			while serial.inWaiting():
				PACKAGE.append(ord(serial.read()))
			if len(PACKAGE) > 0:
				# rospy.logdebug("RECIEVE<< "+str(PACKAGE))
				rospy.logdebug(str(len(PACKAGE)))
	except Exception as e:
		rospy.logerr("cannot recieve data from serial port"+str(e))
	finally:
		return PACKAGE

def write_data(serial, PACKAGE):
	'''
		Function for sending a package though serial port
		Parameters
			serial		serial.Serial instance.
			PACKAGE 	list or bytes object for sending though serial port.
		Return
			bool 		indicate whether sending data succesful or not.
	'''
	PACKAGE = map(int, PACKAGE)
	if serial is None:
		return False
	try: 
		if not serial.is_open:
			# rospy.logwarn("Serial port is not open.")
			rospy.logdebug("Cannot SEND>> "+str(PACKAGE))
			return False
		elif serial.is_open and len(PACKAGE) > 0:
			## TODO
			# Check serial.write return value.
			# Is it indicate whether serial is succesful sending data or not.
			serial.write(PACKAGE)
			rospy.logdebug("SEND>> "+str(PACKAGE))
			return True
	except Exception as e:
		rospy.logerr("From write data function.")
		rospy.logerr(str(e))
		rospy.logerr("Fail to send " + str(PACKAGE))
		return False

class SpinalCordBase(object):
	'''
		Attribute
			rawRecievingBuffer		Raw data from serial port.
			RecievingBuffer 		Package which found in rawRecievingBuffer.
									Use findPackage to detect package in 
									rawRecievingBuffer
		Method
			readData()
			writeData(package)
			requestData(requestPackage[, timeout[, flushData]])
			findPackage()
	'''

	def __init__(self, serial, header = None):
		'''
			initial function.
			Parameters
				serial 		serial.Serial instance.
				Optional
					header 	basically,don't do any thing but it can be useful
							for child class.
		'''
		self.__header = header
		self.__serial = serial

		self.rawRecievingBuffer = []
		self.RecievingBuffer = []

	def readData(self):
		'''
			Read data from serial port and store them in rawRecievingBuffer.
			Return
				None
		'''
		ReadingData = read_data(self.__serial)
		self.rawRecievingBuffer.extend(ReadingData)

	def writeData(self, package):
		'''
			Send data though serial port as given in intial function.
			Parameters
				package 	Package for sending though serial.
			Return
				True		if success
				False		if fail
		'''
		return write_data(self.__serial, package)

	def requestData(self, requestPackage, timeout = None, flushData = True):
		'''
				First send a requestPackage though a serial. Wait for timeout 
			seconds or detect a package in rawRecievingBuffer. if there aren't 
			any package in rawRecievingBuffer, return None otherwise return
			lastest package detect in rawRecievingBuffer.
				For package detection, use function findPackage which can be 
			override for compatibility.
			Parameters
				requestPackage		Package which send first to request 
									a response.
				Optional
					timeout 		if timeout is None, this function wait
									until package arrive or wait forever if no 
									package arrive.
					flushData		if True, clear rawRecievingBuffer and
									RecievingBuffer before send a
									requestPackage. Defualt : True.
			Return
				lastest package arrive if there are any package arrive in time.
				Otherwise return None.
		'''
		if flushData:
			self.rawRecievingBuffer = []
			self.RecievingBuffer = []
		write_data(self.__serial, requestPackage)
		time.sleep(0.02)
		if not self.__waitForResponse(timeout):
			# rospy.logwarn("Cannot request data from "+str(self.__serial.port+"."))
			return None
		rospy.logdebug("Reciev Package<<"+str(self.RecievingBuffer[-1]))
		return self.RecievingBuffer.pop(-1)

	def __waitForResponse(self, timeout):
		## Wiat for package if timeout is none wait until package arrive.
		if timeout == None:
			return self.__waitForEver()
		return self.__waitWithTimeOut(timeout)

	def __waitWithTimeOut(self, timeout):
		## if package arrive in time return True otherwise False.
		start = time.time()
		while time.time() - start <= timeout:
			self.readData()
			if not self.findPackage():
				time.sleep(0.02)
				continue
			else:
				return True
		return False

	def __waitForEver(self):
		## wait until package arrive. always return True.
		while True:
			self.readData()
			if not self.findPackage():
				time.sleep(0.02)
				continue
			else:
				rospy.logdebug("Find Package.")
				break
		return True

	def findPackage(self):
		'''
				Find a package in rawRecievingBuffer. This function implement
			specificly for robotis package. If want to use with another package
			format should override this function.
			Return
				if found package return True otherwise False.
		'''
		stateEnum = {"H1":0,"H2":1,"ID":2,"L1":3,"DA":4,"CS":5}
		state = stateEnum["H1"]
		temp = []
		packLenght = 0
		status = False
		while len(self.rawRecievingBuffer) != 0:
			dataByte = self.rawRecievingBuffer.pop(0)
			if state == stateEnum["H1"]:
				if dataByte == 0xFF:
					temp.append(dataByte)
					state = stateEnum["H2"]
					# rospy.logdebug("h1")
				else:
					temp = []
			elif state == stateEnum["H2"]:
				if dataByte == 0xFF:
					temp.append(dataByte)
					state = stateEnum["ID"]
					# rospy.logdebug("h2")
				else:
					temp = []
					state = stateEnum["H1"]
			elif state == stateEnum["ID"]:
				temp.append(dataByte)
				state = stateEnum["L1"]
				# rospy.logdebug("id")
			elif state == stateEnum["L1"]:
				temp.append(dataByte)
				packLenght = dataByte
				state = stateEnum["DA"]
				# rospy.logdebug("l1")
			elif state == stateEnum["DA"]:
				temp.append(dataByte)
				# rospy.logdebug("da")
				packLenght -= 1
				if packLenght == 0:
					self.RecievingBuffer.append(temp)
					temp = []
					state = stateEnum["H1"]
					status = True
					continue
		return status