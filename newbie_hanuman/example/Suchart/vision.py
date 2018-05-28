from visionManager.visionModule import VisionModule
from newbie_hanuman.msg import suchartVisionMsg
from utility.performanceX import StopWatch

from geometry_msgs.msg import Polygon, Point32
from sensor_msgs.msg import CompressedImage

import rospy

import numpy as np
import cv2

from sklearn.svm import SVC
from sklearn.externals import joblib

import os
import sys

import time
import math

from helperModule.findRectangle import findRectangle, calArea
from helperModule.transform import four_point_transform
from helperModule.binarize import binarize_image
from helperModule.segmentDigit import segmentDigit
from helperModule import HOG
from helperModule.PROJECTION_HISTOGRAM import createProjectionHistogram

PATH = sys.path[0]

class ImageProcessing(VisionModule):

	def __init__(self):
		super(ImageProcessing, self).__init__()
		self.objectsMsgType = suchartVisionMsg
		self.__boards = None

		path = "/".join(PATH.split("/")[:]+["output_matrix.npz"])
		self.mtx = np.load(path)["camera_matrix"]
		self.dist = np.load(path)["distortion_ceff"]

		self.allProcess = StopWatch()
		self.predProcess = StopWatch()

		gamma = 0.8
		self.__lookup = np.array( [ 255.0*((float(i)/255.0)**(gamma)) for i in range(256) ], dtype = np.uint8 )

		fileName = "/".join(PATH.split("/")[:]+["final_train_cropIm.pk1"])
		# fileName = "Train1_cropImMini_SVM_model.pk1"
		self.__clf = joblib.load(fileName)

		self.__hog10 = HOG.createHOGDescription(10)
		self.__hog20 = HOG.createHOGDescription(20)

		self.__timePerLoop = 0

	def __createFeature(self, img):
		hog10 = self.__hog10.compute(img).reshape(-1)
		hog20 = self.__hog20.compute(img).reshape(-1)
		ph = createProjectionHistogram(img)
		X = np.hstack((hog10,hog20))
		X = np.hstack((X,ph))
		return X

	def __enchantImage(self, img):
		return img

	def __findBoards(self, img, areaLowerThr, areaUpperThr):
		rects = findRectangle(img, areaThr = 0.001)
		boards = []
		for rect in rects:
			# print areaUpperThr > calArea(rect) > areaLowerThr
			if areaUpperThr > calArea(rect) > areaLowerThr:
				with self.allProcess:
					dst = four_point_transform(img, rect)
					dst = cv2.resize(dst, (100,100))
					binarize_image(dst)
					dst = segmentDigit(dst)
					if dst is None:
						continue
					dst = cv2.resize(dst, (50,50))
					## Predict image here:
					X = self.__createFeature(dst)
					with self.predProcess:
						y = self.__clf.predict([X])

				boards.append((rect, dst, y[0]))
		# print "----"
		return boards

	def __createMsg(self, boards, img):
		rects = []
		labels = []

		for rect, dst, label in boards:
			poly = Polygon()
			for p in rect:
				poly.points.append(Point32(x=p[0], y=p[1], z=0))
			rects.append(poly)
			labels.append(int(label))

		msg = suchartVisionMsg(rects = rects, labels = labels)
		com = CompressedImage()
		com.format = "jpeg"
		com.data = np.array(cv2.imencode(".jpg", img)[1]).tostring()
		msg.img = com
		return msg


	def ImageProcessingFunction(self, img, header):
		e1 = cv2.getTickCount()
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		h,w = gray.shape
		newcameramtx, roi=cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
		gray = cv2.undistort(gray, self.mtx, self.dist, None, newcameramtx)
		gray = self.__enchantImage(gray)

		areaLowerThr = 0.0035*gray.shape[0]*gray.shape[1]
		areaUpperThr = 0.2*gray.shape[0]*gray.shape[1]

		self.__boards = self.__findBoards(gray, areaLowerThr, areaUpperThr)

		msg = self.__createMsg(self.__boards, img)
		msg.header.stamp = rospy.Time.now()
		e2 = cv2.getTickCount()
		self.__timePerLoop = (e2 - e1) / cv2.getTickFrequency()
		return msg

	def visualizeFunction(self, img, msg):
		h,w,_ = img.shape
		# newcameramtx, roi=cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
		# cv2.undistort(img, self.mtx, self.dist,, newcameramtx)

		time1 = math.ceil(self.allProcess.getAvgTime() * 1000.0)
		time2 = math.ceil(self.predProcess.getAvgTime() * 1000.0)

		for label, rect in zip(msg.labels, msg.rects):
			cnt = np.zeros((4,1,2), dtype = int)
			cen_x = 0
			cen_y = 0
			for i in range(4):
				cnt[i,0,0] = rect.points[i].x
				cnt[i,0,1] = rect.points[i].y
				cen_x += 0.25*rect.points[i].x
				cen_y += 0.25*rect.points[i].y
			for i in range(4):
				cv2.line(img, tuple(cnt[i,0]), tuple(cnt[(i+1)%4,0]), 255,3)
			cv2.polylines(img, cnt, True, 255, 2)
			font = cv2. FONT_HERSHEY_SIMPLEX
			cv2.putText(img, str(label), (int(cen_x),int(cen_y)),
						cv2. FONT_HERSHEY_SIMPLEX,
						1, (0,255,0), 7)
			cv2.putText(img, str(label), (int(cen_x),int(cen_y)),
						cv2. FONT_HERSHEY_SIMPLEX,
						1, (0,0,255), 3)

		cv2.putText(img, "process time:", (0,img.shape[0]-10),
					cv2. FONT_HERSHEY_SIMPLEX,
					1, (0,0,0), 7)
		cv2.putText(img, "process time:", (0,img.shape[0]-10),
					cv2. FONT_HERSHEY_SIMPLEX,
					1, (255,255,255), 3)

		cv2.putText(img, str(time1), (250,img.shape[0]-10),
					cv2. FONT_HERSHEY_SIMPLEX,
					1, (0,0,0), 7)
		cv2.putText(img, str(time1), (250,img.shape[0]-10),
					cv2. FONT_HERSHEY_SIMPLEX,
					1, (255,255,255), 3)

		cv2.putText(img, "predict time:", (0,20),
					cv2. FONT_HERSHEY_SIMPLEX,
					1, (0,0,0), 7)
		cv2.putText(img, "predict time:", (0,20),
					cv2. FONT_HERSHEY_SIMPLEX,
					1, (255,255,255), 3)

		cv2.putText(img, str(time2), (250,20),
					cv2. FONT_HERSHEY_SIMPLEX,
					1, (0,0,0), 7)
		cv2.putText(img, str(time2), (250,20),
					cv2. FONT_HERSHEY_SIMPLEX,
					1, (255,255,255), 3)