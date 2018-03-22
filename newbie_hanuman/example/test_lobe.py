#!/usr/bin/env python
from visionManager.visionModule import VisionModule, KinematicModule
from newbie_hanuman.msg import msgForBlissFace

import rospy
from sensor_msgs.msg import CompressedImage

import numpy as np
import cv2
import dlib

import os
import sys

import time

print sys.path

PATH = sys.path[0]

def getJsPosFromName(Js, name):
	indexJs = Js.name.index(name)
	return Js.position[indexJs]

class A(VisionModule):
	# def __init__(self):
	# 	super(A,self).__init__()
	# 	self.objectsMsgType = msgForBlissFace
	# 	self.__face_cascade = cv2.CascadeClassifier("/home/visittor/haarcascade_frontalface_default.xml")
	# 	self.__detector = dlib.get_frontal_face_detector()

	# def __transform(self, rect):
	# 	x = rect.left()
	# 	y = rect.top()
	# 	w = rect.right() - x
	# 	h = rect.bottom() - y
	# 	return x,y,w,h
	
	# def ImageProcessingFunction(self,img, header):
	# 	e1 = cv2.getTickCount()
	# 	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	# 	maxA = 0
	# 	biggestFace = [0,0,0,0]
	# 	faces = self.__face_cascade.detectMultiScale(gray, 1.1, 5)
	# 	for (x,y,w,h) in faces:
	# 		if w*h > maxA:
	# 			biggestFace = [x,y,w,h]

	# 	msg = msgForBlissFace()
	# 	msg.x = biggestFace[0]
	# 	msg.y = biggestFace[1]
	# 	msg.w = biggestFace[2]
	# 	msg.h = biggestFace[3]
	# 	# msg.header.stamp = rospy.Time.now()
	# 	msg.header = header
	# 	e2 = cv2.getTickCount()
	# 	t = 1/((e2 - e1)/cv2.getTickFrequency())
	# 	print t
	# 	return msg


	# def visualizeFunction(self, img, msg):
	# 	x = msg.x
	# 	y = msg.y
	# 	w = msg.w
	# 	h = msg.h
	# 	cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)

	pass

class B(KinematicModule):
	OFF1 = 7.6
	L1 = 0.2
	L2 = 0.3
	H1 = 0.6
	H2 = 0.5
	def __init__(self):
		super(B, self).__init__()
		intrinMat = np.identity(3)
		intrinMat[0,0] = 300
		intrinMat[1,1] = 300
		intrinMat[0,2] = 320
		intrinMat[1,2] = 240

		
		path = "/".join(PATH.split("/")[:]+["output_matrix.npz"])

		intrinMat = np.load(path)["camera_matrix"]
		# intrinMat[0,0] /= 10
		intrinMat[:2,:2] *= 1
		print intrinMat
		self.set_IntrinsicCameraMatrix(intrinMat)

		tranVec = np.array([0,0,0],float)
		rotVec = np.array([0,0,0],float)
		Hplane1 = self.create_transformationMatrix(tranVec, rotVec, 'zyz')

		self.add_plane(	"plane1", Hplane1, 
						(-np.inf,np.inf), (-np.inf,np.inf), (-np.inf,np.inf))

		tranVec = np.array([0,5,5],float)
		rotVec = np.array([-np.pi/2,np.pi/2,np.pi/2],float)
		Hplane2 = self.create_transformationMatrix(tranVec, rotVec, 'zyz')

		self.add_plane(	"plane2", Hplane2, 
						(-np.inf,np.inf), (-np.inf,np.inf), (-np.inf,np.inf))

		tranVec = np.array([0,-5,5],float)
		rotVec = np.array([-np.pi/2,-np.pi/2,np.pi/2],float)
		Hplane3 = self.create_transformationMatrix(tranVec, rotVec, 'zyz')

		self.add_plane(	"plane3", Hplane3, 
						(-np.inf,np.inf), (-np.inf,np.inf), (-np.inf,np.inf))

		points = [	[0.0, -0.5, 0.0],
					[0.0, -0.25, 0.0],
					[0.0, 0.0, 0.0],
					[0.0, 0.25, 0.0],
					[0.0, 0.5, 0.0],

					[0.25, -0.5, 0.0],
					[0.25, -0.25, 0.0],
					[0.25, 0.0, 0.0],
					[0.25, 0.25, 0.0],
					[0.25, 0.5, 0.0],

					[0.5, -0.5, 0.0],
					[0.5, -0.25, 0.0],
					[0.5, 0.0, 0.0],
					[0.5, 0.25, 0.0],
					[0.5, 0.5, 0.0],

					[0.75, -0.5, 0.0],
					[0.75, -0.25, 0.0],
					[0.75, 0.0, 0.0],
					[0.75, 0.25, 0.0],
					[0.75, 0.5, 0.0],

					[1.0, -0.5, 0.0],
					[1.0, -0.25, 0.0],
					[1.0, 0.0, 0.0],
					[1.0, 0.25, 0.0],
					[1.0, 0.5, 0.0],
					]
		points = np.array(points, float)*10
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

		tranCam = np.array([0.0,0,0],float)
		rotCam = np.array([0, np.pi/2,-np.pi/2],float)
		HCamera = self.create_transformationMatrix(tranCam, rotCam, 'zyz')

		self.point2D1 = self.calculate2DCoor(points, "plane1", HCamera=HCamera)
		self.point2D1 = np.array(self.point2D1)

		self.point2D2 = self.calculate2DCoor(points, "plane2", HCamera=HCamera)
		self.point2D2 = np.array(self.point2D2)

		self.point2D3 = self.calculate2DCoor(points, "plane3", HCamera=HCamera)
		self.point2D3 = np.array(self.point2D3)

		self.HCamera = HCamera

		self.image = None

		fourcc = cv2.VideoWriter_fourcc(*'DIVX')
		filname = "/".join(PATH.split("/")[:]+["output.avi"])
		self.out = cv2.VideoWriter('output.avi',fourcc, 60.0, (640,480))

	def __code(self, x, y, xmin, xmax, ymin, ymax):
		c = 0
		c += 0 if y>=ymin else 1
		c += 0 if y<=ymax else 2
		c += 0 if x<=xmax else 4
		c += 0 if x>=xmin else 8
		return c

	def clipLine(self, point1, point2, shape):
		# code1 = self.__code(point1[0], point1[1], 0, shape[1], 0, shape[0])
		# code2 = self.__code(point2[0], point2[1], 0, shape[1], 0, shape[0])
		# if code1 & code2 != 0:
		# 	return False, point1, point2
		if point1 is None or point2 is None:
			return False, point1, point2

		if (-65535>=point1).any() or (point1>=65535).any():
			return False, point1, point2

		if (-65535>=point2).any() or (point2>=65535).any():
			return False, point1, point2

		# point1 = np.clip(point1.astype(int), -(sys.maxint&0xFFFFFF), sys.maxint&0xFFFFFF)
		# point2 = np.clip(point2.astype(int), -(sys.maxint&0xFFFFFF), sys.maxint&0xFFFFFF)
		point1 = point1.astype(int)
		point2 = point2.astype(int)
		ret, pt1, pt2 = cv2.clipLine( (0,0,shape[1],shape[0]), tuple(point1), tuple(point2))
		return ret, pt1, pt2

	def __createHomoCam(self, Js):
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
		npArray = np.fromstring(objMsg.data, dtype=np.uint8).copy()
		self.image = cv2.imdecode(npArray, 1)

		# panAng = getJsPosFromName(joint, "pan")
		# tiltAng = -getJsPosFromName(joint, "tilt")

		# tranCam = np.array([0,0,8.75],float)
		# rotCam = np.array([panAng,tiltAng,0],float)
		# HPanTilt = self.create_transformationMatrix(tranCam, rotCam, 'zyz')

		# H = np.matmul(HPanTilt, self.HCamera)
		H = self.__createHomoCam(joint)

		self.point2D1 = self.calculate2DCoor(self.points, "plane1", HCamera=H)
		self.point2D1 = np.array(self.point2D1)

		self.point2D2 = self.calculate2DCoor(self.points, "plane2", HCamera=H)
		self.point2D2 = np.array(self.point2D2)

		self.point2D3 = self.calculate2DCoor(self.points, "plane3", HCamera=H)
		self.point2D3 = np.array(self.point2D3)

		# point3D1 = self.calculate3DCoor(self.point2D1, HCamera=self.HCamera)
		point3D1 = self.calculate3DCoor(self.point2D3, HCamera=H)

		# for i,tup in enumerate(point3D1):
		# 	point = tup[1]
		# 	if point is None:
		# 		print None, self.points[i]
		# 		continue 
		# 	p = [round(point[0],2), round(point[1],2), 0]
		# 	p = np.array(p)
		# 	print p, self.points[i]

		# print point3D1
		for i in self.pattern:
			point1 = self.point2D1[i[0]]
			point2 = self.point2D1[i[1]]
			ret, pt1, pt2 = self.clipLine(point1, point2, self.image.shape)
			if ret:
				cv2.line(self.image, pt1, pt2, (255,0,0), 4)

		for i in self.pattern:
			point1 = self.point2D2[i[0]]
			point2 = self.point2D2[i[1]]
			ret, pt1, pt2 = self.clipLine(point1, point2, self.image.shape)
			if ret:
				cv2.line(self.image, pt1, pt2, (0,0,255), 4)

		for i in self.pattern:
			point1 = self.point2D3[i[0]]
			point2 = self.point2D3[i[1]]
			ret, pt1, pt2 = self.clipLine(point1, point2, self.image.shape)
			if ret:
				cv2.line(self.image, pt1, pt2, (0,255,0), 4)

		return self.posDictMsgType()

	def loop(self):
		if self.image is not None:
			cv2.imshow("image",self.image)
			# self.out.write(self.image)
			cv2.waitKey(1)

	def end(self):
		self.out.release()

vision_module = A()
kinematic_module = B()
