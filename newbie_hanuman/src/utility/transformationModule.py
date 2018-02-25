#!/usr/bin/env python
import numpy as np 
import cv2

def Project2Dto3D(point, HomoMat, intrinMat):

	invHomo = getInverseHomoMat(HomoMat)

	Cpoint_3 = np.matmul(invHomo, np.array([0,0,0,1]) )[:3]

	invIntrin = np.linalg.inv(intrinMat)

	point = np.hstack((point,1))
	pPoint = np.matmul(invIntrin, point).reshape(-1)

	pPoint = np.hstack((pPoint, 1))
	pPoint_3 = np.matmul(invHomo, pPoint)[:3]

	unitVec = pPoint_3 - Cpoint_3
	unitVec /= np.linalg.norm(unitVec)

	lambdaVal = -(Cpoint_3[2] / unitVec[2])

	finalPoint = Cpoint_3 + lambdaVal*unitVec

	return finalPoint

def Project3Dto2D(point, HomoMat, intrinMat):
	point = np.hstack((point,1))

	tranPoint = np.matmul(HomoMat, point)[:3]
	tranPoint = np.matmul(intrinMat, tranPoint)
	tranPoint = tranPoint / tranPoint[2]

	return tranPoint[:2]

def getInverseHomoMat(HomoMat):
	rotMat = HomoMat[:3, :3].T
	tvec = HomoMat[:3,3].reshape(-1,1)
	tvec = -1.0*(np.matmul(rotMat, tvec))

	invMat = np.hstack((rotMat, tvec))
	invMat = np.vstack((invMat, np.array([0,0,0,1],float)))

	return invMat
