#!/usr/bin/env python

import cv2
import numpy as np 

import rospy

from newbie_hanuman.msg import PanTiltCommand

import Queue
import math

name = ["pan", "tilt"]
position = [0,0]
velocity = [20,20]
command = [0,0]
ang = math.radians(10)
def createCommand(k):
	global name, position, velocity, command, ang
	if k == ord('p'):
		position = [0,ang]
	elif k == ord(';'):
		position = [0,-ang]
	elif k == ord("l"):
		position = [ang,0]
	elif k == ord("'"):
		position = [-ang,0]
	else:
		position = [0,0]
	c = PanTiltCommand(	name 		= name,
						position 	= position,
						velocity 	= velocity,
						command 	= command)
	return c

def controller():
	pub = rospy.Publisher('/spinalcord/sternocleidomastoid_command', PanTiltCommand, queue_size=10)
	rospy.init_node('pan_tilt',anonymous=True)
	rate = rospy.Rate(5)
	blank = np.zeros((640,640), dtype=np.uint8)
	while 1==1:
		cv2.imshow("blank",blank)
		k = cv2.waitKey(1)
		command = createCommand(k)
		if k == 27:
			break
		pub.publish(command)
		rate.sleep()

def main():
	try:
		controller()
	except rospy.ROSInterruptException:
		pass
	finally:
		cv2.destroyAllWindows()