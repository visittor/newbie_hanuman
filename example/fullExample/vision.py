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
	indexJs = Js.name.index(name)
	return Js.position[indexJs]

class ImageProcessing(VisionModule):
	def __init__(self):
		super(ImageProcessing, self).__init__()
		self.objectsMsgType = visionMsg
		self.__lowerOrg = np.array([10, 150, 20], np.uint8)
		self.__upperOrg = np.array([20, 255, 255], np.uint8)
		self.__ball = None

	def ImageProcessingFunction(self, img, header):
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

		msg = visionMsg()
		msg.ball = center
		msg.imgH = hsv.shape[0]
		msg.imgW = hsv.shape[1]
		msg.header.stamp = rospy.Time.now()
		return msg

	def visualizeFunction(self, img, msg):
		if len(msg.ball) > 0 :
			x = msg.ball[0]
			y = msg.ball[1]
			cv2.circle(img, (x,y), 5, (0,0,255), -1)
			if self.__ball is not None:
				# print "Hey"
				cv2.drawContours(img, [self.__ball], 0, (255,0,0), 2)

class Kinematic(KinematicModule):
	OFF1 = 7.6
	L1 = 0.2
	L2 = 0.3
	H1 = 0.6
	H2 = 0.5
	
	def __init__(self):
		super(Kinematic, self).__init__()

		path = "/".join(PATH.split("/")[:]+["output_matrix.npz"])

		intrinMat = np.load(path)["camera_matrix"]
		# intrinMat[0,0] /= 10
		intrinMat[:2,:2] *= 1
		self.set_IntrinsicCameraMatrix(intrinMat)

		self.objectsMsgType = visionMsg
		self.posDictMsgType = postDictMsg

		tranVec = np.array([0,0,0], float)
		rotVec = np.array([0,0,0], float)
		HTran = self.create_transformationMatrix(tranVec, rotVec, 'zyz')

		self.add_plane("ground", HTran, (0,100), (-100,100), (-10,10))

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
		self.image = np.zeros((objMsg.imgH, objMsg.imgW), np.uint8)

		H = self.forwardKinematic(joint)

		ball_img = np.array(objMsg.ball).reshape(-1,2).astype(np.float)
		if len(ball_img) == 0:
			ball_cart = None
		else:
			ball_cart = self.calculate3DCoor(ball_img,HCamera=H)
			ball_cart = ball_cart[0][1]
			print "ball_cart",ball_cart
		if ball_cart is not None:
			ball_polar = cv2.cartToPolar(ball_cart[0], ball_cart[1])
			ball_polar = np.array([ball_polar[0][0], ball_polar[1][0]])
		else:
			ball_cart = np.array([])
			ball_polar = np.array([])

		self.point2D = self.calculate2DCoor(self.points, "ground", HCamera=H)

		msg = postDictMsg()
		msg.ball_cart = ball_cart.reshape(-1)
		msg.ball_polar = ball_polar.reshape(-1)
		msg.ball_img = ball_img.reshape(-1)
		msg.imgW = objMsg.imgW
		msg.imgH = objMsg.imgH
		msg.header.stamp = rospy.Time.now()

		# for i in self.pattern:
		# 	point1 = self.point2D[i[0]]
		# 	point2 = self.point2D[i[1]]
		# 	ret, pt1, pt2 = self.clipLine(point1, point2, self.image.shape)
		# 	if ret:
		# 		cv2.line(self.image, pt1, pt2, (255,0,0), 4)
		# print ball_img
		# cv2.circle(self.image, tuple(ball_img[0].astype(int)), 5, (0,0,255), -1)
		print msg.header.stamp
		return msg

	def loop(self):
		# if self.image is not None:
		# 	cv2.imshow("kinematic", self.image)
		# 	cv2.waitKey(1)
		pass

	def end(self):
		cv2.destroyAllWindows()

vision_module = ImageProcessing()
kinematic_module = Kinematic()