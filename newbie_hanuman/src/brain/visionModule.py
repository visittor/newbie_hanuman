#! /usr/bin/env python
import numpy as np 
import cv2

from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Empty

from utility.transformationModule import Project2Dto3D, Project3Dto2D, getInverseHomoMat

class VisionModule(object):

	def __init__(self):
		self.objectsMsgType = CompressedImage

	def ImageProcessingFunction(self,img, header):
		'''Override this function.'''
		msg = CompressedImage()
		msg.header.stamp = header.stamp
		msg.format = "jpeg"
		msg.data = np.array(cv2.imencode(".jpg", img)[1]).tostring()
		return msg

	def visualizeFunction(self, img, msg):
		pass

class KinematicModule(object):

	class Plane(object):

		def __init__(self):
			self.__H = np.identity(4, dtype = float)
			self.__planeEq = np.array([0, 0, 1, 0])
			self.__boundaryX = (-np.inf, np.inf)
			self.__boundaryY = (-np.inf, np.inf )
			self.__boundaryZ = (-np.inf, np.inf)

		def get_transformationMatrix(self):
			return self.__H.copy()

		def set_transformationMatrix(self, H):
			# Check if Homogeneous is in correct shape. 
			assert H.shape == (4,4)
			# Check if rotation matrix is orthogonal
			assert -1e-16 < np.dot(H[:3,0], H[:3,1]) < 1e-16
			assert -1e-16 < np.dot(H[:3,0], H[:3,2]) < 1e-16
			assert -1e-16 < np.dot(H[:3,1], H[:3,2]) < 1e-16
			assert -1e-16 < np.dot(H[0,:3], H[1,:3]) < 1e-16
			assert -1e-16 < np.dot(H[0,:3], H[2,:3]) < 1e-16
			assert -1e-16 < np.dot(H[1,:3], H[2,:3]) < 1e-16

			# assert ([-0e-16,-0e-16,-0e-16,-0e-16] <= H[:4,3] - [0.0,0.0,0.0,1.0]<=[0e-16,0e-16,0e-16,0e-16]).all()

			self.__H = H

		def get_boundary(self):
			return 	(self.__boundaryX ,
					self.__boundaryY ,
					self.__boundaryZ )

		def set_boundary(self, boundX, boundY, boundZ):
			assert len(boundX) == len(boundY) == len(boundZ) == 2

			self.__boundaryX = boundX
			self.__boundaryY = boundY
			self.__boundaryZ = boundZ

		def checkBoundary(self, x, y, z):
			if not (self.__boundaryX[0] <= x <= self.__boundaryX[1]):
				return False

			if not (self.__boundaryY[0] <= y <= self.__boundaryY[1]):
				return False

			if not (self.__boundaryZ[0] <= z <= self.__boundaryZ[1]):
				return False

			return True

	def __init__(self):
		self.objectsMsgType = CompressedImage
		self.posDictMsgType = Empty
		self.__intrinsicMatrix = None
		self.__planes = {}

	@classmethod
	def create_transformationMatrix(self, tranVec, rotVec, mode):
		aux = np.array([0,0,0,1], dtype=float)
		if mode.lower() == "rot":
			rotMat = rotVec
		elif mode.lower() == "zyz":
			rotMat = self.__create_rotFromZYZ(rotVec)
		elif mode.lower() == "rpy":
			rotMat = self.__create_rotFromRPY(rotVec)
		else:
			raise ValueError(mode+" is not your choice.")

		if tranVec.shape != (3,1):
			tranVec = tranVec.reshape(3,1)
		H = np.hstack((rotMat, tranVec))
		H = np.vstack((H, aux))
		return H

	@classmethod
	def __create_rotFromZYZ(self, rotVec):
		Z1 = np.array([ [np.cos(rotVec[0]), -np.sin(rotVec[0]),		0],
						[np.sin(rotVec[0]), np.cos(rotVec[0]),		0],
						[0,					0,						1]],
					dtype = float)
		Y  = np.array([ [np.cos(rotVec[1]), 0,		np.sin(rotVec[1])],
						[0, 				1,		0],
						[-np.sin(rotVec[1]),0,		np.cos(rotVec[1])]],
					dtype = float)
		Z2 = np.array([ [np.cos(rotVec[2]), -np.sin(rotVec[2]),		0],
						[np.sin(rotVec[2]), np.cos(rotVec[2]),		0],
						[0,					0,						1]],
					dtype = float)
		return np.matmul(Z1, np.matmul(Y,Z2))

	@classmethod
	def __create_rotFromRPY(self, rotVec):
		R = np.array([  [1, 	0,					0],
						[0,		np.cos(rotVec[0]),	-np.sin(rotVec[0])],
						[0,		np.sin(rotVec[0]),	np.cos(rotVec[0])]],
						dtype = float)
		P = np.array([  [np.cos(rotVec[1]), 0,		np.sin(rotVec[1])],
						[0, 				1,		0],
						[-np.sin(rotVec[1]),0,		np.cos(rotVec[1])]],
					dtype = float)
		Y = np.array([  [np.cos(rotVec[2]), -np.sin(rotVec[2]),		0],
						[np.sin(rotVec[2]), np.cos(rotVec[2]),		0],
						[0,					0,						1]],
					dtype = float)
		return np.matmul(np.matmul(Y,P), R)

	def set_IntrinsicCameraMatrix(self, intrinMat):
		assert intrinMat.shape == (3,3)
		self.__intrinsicMatrix = intrinMat

	def add_plane(self, name, H, boundaryX, boundaryY, boundaryZ):
		assert not self.__planes.has_key(name)
		plane = KinematicModule.Plane()
		plane.set_transformationMatrix(H)
		plane.set_boundary(boundaryX, boundaryY, boundaryZ)
		self.__planes[name] = plane

	def calculate3DCoor(self, points, HCamera = None):
		if HCamera is None:
			HCamera = np.identity(4)

		# invHCamera = getInverseHomoMat(HCamera)
		HCam_plane = np.identity(4)

		points3D = [(None,None)] * len(points)

		for name,plane in self.__planes.items():
			H = plane.get_transformationMatrix()
			invH = getInverseHomoMat(HCamera)
			np.matmul(invH, H, HCam_plane)
			for i,point in enumerate(points):
				point3D = Project2Dto3D(point, 
										HCam_plane, 
										self.__intrinsicMatrix)
				# if name == "plane1":
				# 	print point3D, point
				if plane.checkBoundary(points3D[0],points3D[1],points3D[2]):
					points3D[i] = (name, point3D)
		return points3D

	def calculate2DCoor(self, points, name, HCamera = None):
		assert self.__planes.has_key(name)

		if HCamera is None:
			HCamera = np.identity(4)

		H = self.__planes[name].get_transformationMatrix()
		invH = getInverseHomoMat(HCamera)
		HCam_plane = np.matmul(invH, H)

		points2D = []

		for point in points:
			point2D = Project3Dto2D(point, HCam_plane, self.__intrinsicMatrix)
			points2D.append(point2D)

		return points2D

	def kinematicCalculation(self, objMsg, jointState):
		return Empty()

	def loop(self):
		pass