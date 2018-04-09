#!/usr/bin/env python

import cv2
import numpy as np 
import pandas as pd

from sklearn.svm import SVC
from sklearn.externals import joblib

import HOG
from PROJECTION_HISTOGRAM import createProjectionHistogram

from findRectangle import findRectangle, calArea
from transform import four_point_transform
from binarize import binarize_image

from segmentDigit import segmentDigit

FILE_NAME = "Train1_cropImMini_SVM_model.pk1"
# MASK = np.load("Train1_complete_mask.npy")
# print MASK.shape

HOG.BLOCKSIZE = 10
HOG.BLOCKSTRIBE = 10/2
HOG.CELLSIZE = 10
hogDes10 = HOG.createHOGDescription(10)

HOG.BLOCKSIZE = 20
HOG.BLOCKSTRIBE = 20/2
HOG.CELLSIZE = 20
hogDes20 = HOG.createHOGDescription(20)

def create_feature(img):
	global hogDes10, hogDes20
	hog10 = hogDes10.compute(img).reshape(-1)
	hog20 = hogDes20.compute(img).reshape(-1)
	ph = createProjectionHistogram(img)
	X = np.hstack((hog10,hog20))
	X = np.hstack((X,ph))
	# X = np.hstack((X))
	# X = X[MASK]
	return X

def find_center(point):
	x = 0
	y = 0
	for i in range(4):
		x += point[i][0]*0.25
		y += point[i][1]*0.25
	return x,y

if __name__ == "__main__":
	clf = joblib.load(FILE_NAME)

	cap = cv2.VideoCapture(1)

	while True:
		ret, img = cap.read()
		
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		areaThr = 0.1*gray.shape[0]*gray.shape[1]

		rects = findRectangle(gray)
		boards = []
		for rect in rects:
			A = calArea(rect)
			if A > areaThr:
				areaThr = A
				dst = four_point_transform(gray, rect)
				dst = cv2.resize(dst, (100,100))
				binarize_image(dst)
				dst = segmentDigit(dst)
				if dst is None:
					continue
				dst = cv2.resize(dst, (50,50))

				X = create_feature(dst)

				y = clf.predict([X])
				boards = [(rect, dst, y[0])]

		for board in boards:
			for i in range(4):
				cv2.line(img, tuple(board[0][i]), tuple(board[0][(i+1)%4]), 255,3)
			x,y = find_center(board[0])
			l = board[2]
			cv2.putText(img, str(l), (int(x),int(y)),
						cv2. FONT_HERSHEY_SIMPLEX,
						1, (0,255,0), 7)
			cv2.putText(img, str(l), (int(x),int(y)),
						cv2. FONT_HERSHEY_SIMPLEX,
						1, (0,0,255), 3)
			cv2.imshow("dst", board[1])

		cv2.imshow("img", img)
		k = cv2.waitKey(1)
		if k == 27:
			break

	cv2.destroyAllWindows()