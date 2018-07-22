from visionManager.visionModule import VisionModule, KinematicModule
from newbie_hanuman.msg import visionMsg, postDictMsg

import rospy

import numpy as np
import cv2

import os
import sys

import time

print sys.path

PATH = sys.path[0]

def getJsPosFromName(Js, name):
	'''
	Get Joint state of pantilt.
	argument:
		Js 		:	Joint state message
		name 	:	(str) Either 'pan' or 'tilt'
	return:
		Position of that joint
	'''
	indexJs = Js.name.index(name)
	return Js.position[indexJs]

class ImageProcessing(VisionModule):
	'''
	Class for doing image processing
	This function will call function ImageProcessingFunction in ocipital node loop.
	That node will get image from CranialNerveII frame do this for you don't worry.
	What you may have to define is ImageProcessingFunction and visualizeFunction.
	!!! YOU MUST DEFINE self.objectsMsgType !!!
	'''
	def __init__(self):
		super(ImageProcessing, self).__init__()

		#######################################
		##	You should always set self.objectsMsgType to message you want if not it will be CompressedImage.
		self.objectsMsgType = visionMsg

		#######################################
		##	Do your image processing stuff
		self.__lowerOrg = np.array([10, 150, 20], np.uint8)
		self.__upperOrg = np.array([20, 255, 255], np.uint8)
		self.__ball = None

	def ImageProcessingFunction(self, img, header):
		'''
		This function do image processing stuff. It took 
		2 messages from frame work; image and header message.
		arguments:
			image 	:	numpy image
			header	:	header message
		return:
			return message type as define on self.objectsMsgType 
		'''
		#######################################
		##	Do your image processing stuff
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		msk = cv2.inRange(hsv, self.__lowerOrg, self.__upperOrg)

		_, contours, _ = cv2.findContours(msk.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		maxA = 0
		ball = None
		for cnt in contours:
			area = cv2.contourArea(cnt)
			if maxA < area:
				maxA = area
				ball = cnt
		center = []
		if ball is not None:

			M = cv2.moments(ball)
			if M["m00"] != 0:
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				center = [cx, cy]

		self.__ball = ball

		#######################################
		##	Create message for return
		##	Framework will publish this message
		##	for you ^_^
		msg = visionMsg()
		msg.ball = center
		msg.imgH = hsv.shape[0]
		msg.imgW = hsv.shape[1]
		msg.header.stamp = rospy.Time.now()
		return msg

	def visualizeFunction(self, img, msg):
		'''
		This function visualize function.
		It should draw some stuff on img.
		This img will be publish as a message of
		CranialNerveIIMonitor.
		arguments:
			img 	:	numpy img
			msg 	:	message you return in ImageProcessingFunction
		return:
			return nothing
		'''

		#######################################
		##	Draw your image here
		if len(msg.ball) > 0 :
			x = msg.ball[0]
			y = msg.ball[1]
			cv2.circle(img, (x,y), 5, (0,0,255), -1)
			if self.__ball is not None:
				# print "Hey"
				cv2.drawContours(img, [self.__ball], 0, (255,0,0), 2)

class Kinematic(KinematicModule):
	'''
	This class use for transforming img coor to 3D coor
	This function will get message from occipitalLobe and call function 'kinematicCalculation' in callback function.
	In loop in kinematicBrain, it will call function 'loop'. It allow you to do something in loop i.e. show an image.
	You may have to define function 'kinematicCalculation' and function 'loop'.
	!!! YOU MUST DEFINE self.objectsMsgType AND self.posDictMsgType !!!
	'''

	## Parameter for forward kinematic
	OFF1 = 7.6
	L1 = 0.2
	L2 = 0.3
	H1 = 0.6
	H2 = 0.5
	
	def __init__(self):
		super(Kinematic, self).__init__()

		#######################################
		##	load intrinsic camera matrix
		path = "/".join(PATH.split("/")[:]+["output_matrix.npz"])

		# intrinMat = np.load(path)["camera_matrix"]
		# intrinMat[0,0] /= 10
		# intrinMat[:2,:2] *= 1
		intrinMat = np.zeros( (3,3) )
		intrinMat[0,0] = 10.0
		intrinMat[1,1] = 10.0
		intrinMat[0,2] = 320.0
		intrinMat[1,2] = 240.0
		self.set_IntrinsicCameraMatrix(intrinMat)

		#######################################
		##	You should define this 2 variables
		##	self.objectsMsgType must be a same message as above class
		##	self.posDictMsgType is your message which contain coor in 3D coor which use in brain
		self.objectsMsgType = visionMsg
		self.posDictMsgType = postDictMsg

		#######################################
		##	Create homogenouse transformation matrix, 
		##	it took translation vector and rotation vector
		##	create_transformationMatrix(self, tranVec, rotVec, mode, order="tran-first"):
		##	tranVec = translation vector
		##	rotVec	= rotation vector
		##	mode 	= 'zyz' or 'rpy' ( zyz or rpw pitch yaw)
		##	order	= 'tran-first' or 'rot-first'
		##	If you don't know homogenouse transformation, ask Aj.Pi.
		##	or search a googling what the fuck is homogenouse transformation.
		tranVec = np.array([0,0,0], float)
		rotVec = np.array([0,0,0], float)
		HTran = self.create_transformationMatrix(tranVec, rotVec, 'zyz')

		#######################################
		##	Add plane, It use homo tran matrix and boundary of that plane
		## 	For more information ask visittor ( chattarin.rodpon@gmail.com )
		self.add_plane("ground", HTran, (0,100), (-100,100), (-10,10))

		## This is for visualization. Forgot about this
		x, y = np.indices((5,5))
		z = np.zeros((25,1), float)
		points = np.hstack((x.reshape(-1,1), y.reshape(-1,1))).astype(float)
		points[:,0] *= 0.25
		points[:,1] = 0.25*points[:,1] - 0.5
		points = np.hstack((points,z))
		self.points = points.copy()

		self.pattern = [	[0, 4],
							[5, 9],
							[10, 14],
							[15, 19],
							[20, 24],
							[0, 20],
							[1, 21],
							[2, 22],
							[3, 23],
							[4, 24]
						]
		self.image = None

	## For visualization stuff it not matter.
	def clipLine(self, point1, point2, shape):
		if point1 is None or point2 is None:
			return False, point1, point2

		if (-65535>=point1).any() or (point1>=65535).any():
			return False, point1, point2

		if (-65535>=point2).any() or (point2>=65535).any():
			return False, point1, point2

		point1 = point1.astype(int)
		point2 = point2.astype(int)
		ret, pt1, pt2 = cv2.clipLine( (0,0,shape[1],shape[0]), tuple(point1), tuple(point2))
		return ret, pt1, pt2

	## It do forward kinematic of something. It is pantilt head I guess.
	def forwardKinematic(self, Js):
		q1 = getJsPosFromName(Js, "pan")
		q2 = getJsPosFromName(Js, "tilt")

		tranOffset = np.array([0,0.15,self.OFF1],float)
		rotOffset = np.array([0,0,0],float)
		HOffset = self.create_transformationMatrix(tranOffset, rotOffset, 'zyz')

		rot =[	[np.sin(q1), 	np.cos(q1)*np.sin(q2),	np.cos(q1)*np.cos(q2)],
				[-np.cos(q1),	np.sin(q1)*np.sin(q2),	np.sin(q1)*np.cos(q2)],
				[0,				-np.cos(q2),			np.sin(q2)]]
		rot = np.array(rot, float)
		tran = [[self.L2*np.cos(q1)*np.cos(q2)+self.L1*np.cos(q1)-self.H2*np.cos(q1)*np.sin(q2)],
				[self.L2*np.sin(q1)*np.cos(q2)-self.H2*np.sin(q1)*np.sin(q1)+self.L1*np.sin(q1)],
				[self.L2*np.sin(q2)+self.H2*np.cos(q2)+self.H1]]
		tran = np.array(tran, float)
		Homo = np.hstack((rot, tran))
		Homo = np.vstack((Homo, np.array([0,0,0,1])))
		return np.matmul(HOffset, Homo)

	def kinematicCalculation(self, objMsg, joint):
		'''
		This function transform image coordinate to 3D coordinate. It took 
		2 messages from frame work; objMsg as define in self.objectsMsgType and Jointstate from pantilt.
		arguments:
			objMsg 	:	message as define in self.objectsMsgType
			joint	:	JointState message from pantilt
		return:
			return message type as define on self.postDictMsgType 
		'''
		## This for visualization
		self.image = np.zeros((objMsg.imgH, objMsg.imgW), np.uint8)

		## Homo tran matrix from forwardKinematic function
		H = self.forwardKinematic(joint)

		## Get ball in image coordinate
		ball_img = np.array(objMsg.ball).reshape(-1,2).astype(np.float)

		## If there are balls, translate it to 3DCoor using self.calculate3DCoor.
		## calculate3DCoor(self, points, HCamera = None)
		## points it is a numpy array of points. It shape should be ( n , 2 ); n is number of point.
		## HCamera is homogenouse transformation of camera. It transform robot coor to cam coor.
		if len(ball_img) == 0:
			ball_cart = None
		else:
			ball_cart = self.calculate3DCoor(ball_img,HCamera=H)
			ball_cart = ball_cart[0][1]
			print "ball_cart",ball_cart

		## If you want polar coor istead of catesian coor, do this
		if ball_cart is not None:
			ball_polar = cv2.cartToPolar(ball_cart[0], ball_cart[1])
			ball_polar = np.array([ball_polar[0][0], ball_polar[1][0]])
		else:
			ball_cart = np.array([])
			ball_polar = np.array([])

		## visualize stuff
		self.point2D = self.calculate2DCoor(self.points, "ground", HCamera=H)

		#######################################
		##	Create message for return
		##	Framework will publish this message
		##	for you ^_^
		msg = postDictMsg()
		msg.ball_cart = ball_cart.reshape(-1)
		msg.ball_polar = ball_polar.reshape(-1)
		msg.ball_img = ball_img.reshape(-1)
		msg.imgW = objMsg.imgW
		msg.imgH = objMsg.imgH
		msg.header.stamp = rospy.Time.now()

		print msg.header.stamp
		return msg

	def loop(self):
		'''
		This function will call in node loop if you want to visualize via cv2.imshow or something like that, do it in this function.
		It doesn't take any argument, and doesn't return any.
		'''
		# if self.image is not None:
		# 	cv2.imshow("kinematic", self.image)
		# 	cv2.waitKey(1)
		pass

	def end(self):
		cv2.destroyAllWindows()

## You MUST!!! create an instance call vision_module and kinematic_module
vision_module = ImageProcessing()
kinematic_module = Kinematic()