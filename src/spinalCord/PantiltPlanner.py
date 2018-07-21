#!/usr/bin/env python
import rospy

from sensor_msgs.msg import JointState

from newbie_hanuman.msg import PanTiltCommand
from newbie_hanuman.srv import PanTiltPlannerCommand, PanTiltPlannerCommandResponse

from cell.nodeCell import NodeBase

import math

PATTERN = {}

PATTERN["basic_pattern"] = [ (math.radians(-30), math.radians(0)),
							 (math.radians(30), math.radians(0)),
							 (math.radians(30), math.radians(30)),
							 (math.radians(-30), math.radians(30))]

def getJsPosFromName(Js, name):
	indexJs = Js.name.index(name)
	return Js.position[indexJs]

def getJsVelFromName(Js, name):
	indexJs = Js.name.index(name)
	return Js.velocity[indexJs]

class PantiltPlanner(NodeBase):

	class DummyScaner(object):
		def initialScan(self):
			pass

		def execute(self):
			pass

		def terminate(self):
			pass

	class Scan(object):
		def __init__(self, velocity, patterns = PATTERN):
			self.__velocity = velocity
			self.__patterns = patterns

			self.__startTime = None
			self.__currentPattern = None
			self.__currentIndex = None
			self.__goalPosition = None
			self.__prevPosition = None

			self.__isTerminate = True

		def initialScan(self, patternName, currentPosition):
			if not self.__patterns.has_key(patternName):
				rospy.logwarn("No pattern name \""+pattern+"\". Terminate this scan.")
				return False

			if self.__currentPattern == patternName and not self.__isTerminate:
				return False

			self.__prevPosition = currentPosition
			self.__currentPattern = self.__patterns[patternName]
			self.__currentIndex = 0
			self.__goalPosition = self.__currentPattern[self.__currentIndex]
			self.__startTime = rospy.get_time()

			self.__isTerminate = False
			return True

		def execute(self):
			if self.__isTerminate:
				return

			s = self.__velocity * (rospy.get_time() - self.__startTime)

			direcPan  = 1 if self.__goalPosition[0]>self.__prevPosition[0] else -1
			direcTilt  = 1 if self.__goalPosition[1]>self.__prevPosition[1] else -1

			desirePan = self.__prevPosition[0] + (direcPan*s)
			desireTilt= self.__prevPosition[1] + (direcTilt*s)
			tiltReach = False
			panReach = False
			if rospy.get_time() - self.__startTime >= math.fabs(self.__goalPosition[0] - self.__prevPosition[0])/self.__velocity:
				panReach = True
				desirePan = self.__goalPosition[0]

			if rospy.get_time() - self.__startTime >= math.fabs(self.__goalPosition[1] - self.__prevPosition[1])/self.__velocity:
				tiltReach = True
				desireTilt = self.__goalPosition[1]

			if panReach and tiltReach:
				self.__getNextGoalPosition()

			c = PanTiltCommand(	name 		= 	["pan", "tilt"],
								position 	=	[desirePan, desireTilt],
								velocity 	=	[0,0],
								command 	=	[1,1])
			return c

		def __getNextGoalPosition(self):
			self.__prevPosition = self.__goalPosition
			self.__currentIndex += 1
			self.__currentIndex %= len(self.__currentPattern)
			self.__goalPosition = self.__currentPattern[self.__currentIndex]
			self.__startTime = rospy.get_time()

		def terminate(self):
			self.__startTime = None
			self.__currentPattern = None
			self.__currentIndex = None
			self.__goalPosition = None
			self.__prevPosition = None

			self.__isTerminate = True

	class SetPosition(object):
		def __init__(self, velocity):
			self.__velocity = velocity

			self.__startTime = None
			self.__goalPosition = None
			self.__prevPosition = None
			self.__actualVel = (self.__velocity, self.__velocity)

			self.__isTerminate = True

		def initialScan(self, goalPosition, currentPosition, velocity=None):

			self.__goalPosition = goalPosition
			self.__prevPosition = currentPosition

			if velocity is not None:
				self.__actualVel = velocity
			else:
				self.__actualVel = (self.__velocity, self.__velocity)

			self.__startTime = rospy.get_time()
			self.__isTerminate = False

		def execute(self):
			if self.__isTerminate:
				return

			s_pan = self.__actualVel[0] * (rospy.get_time() - self.__startTime)
			s_tilt = self.__actualVel[1] * (rospy.get_time()- self.__startTime)

			direcPan  = 1 if self.__goalPosition[0]>self.__prevPosition[0] else -1
			direcTilt  = 1 if self.__goalPosition[1]>self.__prevPosition[1] else -1

			desirePan = self.__prevPosition[0] + (direcPan*s_pan)
			desireTilt = self.__prevPosition[1] + (direcTilt*s_tilt)

			tiltReach = False
			panReach = False

			if rospy.get_time()-self.__startTime>=math.fabs(self.__goalPosition[0] - self.__prevPosition[0])/self.__actualVel[0]:
				panReach = True
				desirePan = self.__goalPosition[0]

			if rospy.get_time()-self.__startTime>=math.fabs(self.__goalPosition[1] - self.__prevPosition[1])/self.__actualVel[1]:
				tiltReach = True
				desireTilt = self.__goalPosition[1]

			c = PanTiltCommand(	name 		= 	["pan", "tilt"],
								position 	=	[desirePan, desireTilt],
								velocity 	=	[0,0],
								command 	=	[1,1])
			return c

		def terminate(self):
			self.__startTime = None
			self.__goalPosition = None
			self.__prevPosition = None
			self.__actualVel = (self.__velocity, self.__velocity)

			self.__isTerminate = True

	class PDControl(object):
		def __init__(self, kp, kd):
			self.__kp = kp
			self.__kd = kd
			self.__prevTime = rospy.get_time()

			self.__prevError = None
			self.__currError = None
			self.__isTerminate = True

		def initialScan(self, error):
			self.__currError = error
			self.__isTerminate = False


		def execute(self):
			if self.__isTerminate:
				return

			errorPan = min(max(self.__currError[0], -1.0), 1.0)
			errorTilt = min(max(self.__currError[1], -1.0), 1.0)

			if -0.2 < errorPan < 0.2:
				errorPan = 0.0

			if -0.2 < errorTilt < 0.2:
				errorTilt = 0.0

			derivTermPan = self.__kd*(errorPan - self.__prevError[0]) if self.__prevError is not None else 0.0
			propTermPan = self.__kp*errorPan
			pdPan = propTermPan - derivTermPan/(rospy.get_time() - self.__prevTime)
			# print propTermPan, derivTermPan, errorPan

			derivTermTilt = self.__kd*(errorTilt - self.__prevError[1]) if self.__prevError is not None else 0.0
			propTermTilt = self.__kp*errorTilt
			pdTilt = propTermTilt - derivTermTilt/(rospy.get_time() - self.__prevTime)

			self.__prevError = self.__currError
			self.__prevTime = rospy.get_time()
			c = PanTiltCommand(	name 		= 	["pan", "tilt"],
								position 	=	[pdPan, pdTilt],
								velocity 	=	[0,0],
								command 	=	[0,0])
			self.__currError = [0,0]
			return c

		def terminate(self):
			self.__isTerminate = True
			# self.__prevError = None
			# self.__currError = None
			# self.__prevTime = rospy.get_time()

	def __init__(self):
		super(PantiltPlanner, self).__init__("pantiltplanner")
		self.rosInitNode()
		pantiltVel = self.getParam(
							self.nameSpace+'spinalcord/pantilt_scan_velocity', 
							math.radians(30))

		patterns = eval(self.getParam(
							self.nameSpace+'spinalcord/pantilt_scanpantilt', 
							str(PATTERN)))

		kp = float(self.getParam(
							self.nameSpace+'spinalcord/pantilt_kp',
							math.radians(10)))
		kd = float(self.getParam(
							self.nameSpace+'spinalcord/pantilt_kd',
							math.radians(0)))

		self.__dummy = PantiltPlanner.DummyScaner()

		self.__patternScaner = PantiltPlanner.Scan(	pantiltVel, 
													patterns = patterns)

		self.__setPosition   = PantiltPlanner.SetPosition(pantiltVel)

		self.__pdScan		 = PantiltPlanner.PDControl(kp, kd)

		self.__scaner = self.__dummy

		self.__executeFunction = None
		self.__nextExecuteFunc = None

		self.__currentPosition = (0.0,0.0)

		self.setFrequencyFromParam(
							self.nameSpace+'spinalcord/pantilt_frequency')
		self.__rosInitSubPubSer()

	def __rosInitSubPubSer(self):
		self.rosInitPublisher(	"/spinalcord/sternocleidomastoid_command",
			 					PanTiltCommand,
			 					queue_size = 1)
		self.rosInitSubscriber(	"/spinalcord/sternocleidomastoid_position", 
								JointState, 
								self.__callback)
		self.rosInitService(	"pantiltplannercommand", 
								PanTiltPlannerCommand, 
								self.__serviceHandle)

	def __serviceHandle(self, req):
		if req.command == 1:
			self.__scaner.terminate()
			self.__patternScaner.initialScan(req.pattern, 
											self.__currentPosition)
			self.__scaner = self.__patternScaner

		elif req.command == 0:
			if len(req.velocity) >= 2:
				panVel = getJsVelFromName(req, 'pan')
				tiltVel = getJsVelFromName(req, 'tilt')
				velocity = (panVel, tiltVel)
			else:
				velocity = None
			panGoal = getJsPosFromName(req, 'pan')
			tiltGoal= getJsPosFromName(req, 'tilt')

			self.__scaner.terminate()
			self.__setPosition.initialScan( (panGoal, tiltGoal),
											self.__currentPosition,
											velocity = velocity)
			self.__scaner = self.__setPosition

		elif req.command == 2:
			errorPan = req.errorPan
			errorTilt = req.errorTilt
			self.__scaner.terminate()
			self.__pdScan.initialScan((errorPan, errorTilt))
			self.__scaner = self.__pdScan

		elif req.command == 3:
			self.__scaner.terminate()
			self.__scaner = self.__dummy

		else:
			return PanTiltPlannerCommandResponse(success = False)

		return PanTiltPlannerCommandResponse(success = True)

	def __callback(self, msg):
		panAng = getJsPosFromName(msg, "pan")
		tiltAng = getJsPosFromName(msg, "tilt")
		self.__currentPosition = (panAng, tiltAng)

	def run(self):
		rospy.loginfo("Start pantilt planer node.")
		while  not rospy.is_shutdown():
			command = self.__scaner.execute()
			# print command
			if command is not None:
				self.publish(command)
			self.sleep()

	def end(self):
		rospy.loginfo("Exit pantilt planer node.")

def main():
	node = PantiltPlanner()
	try:
		node.run()
	except rospy.ROSInterruptException as e:
		print e
		rospy.logwarn(e)
	# except Exception as e:
		# print e
	# 	rospy.logwarn(e)
	finally:
		node.end()
