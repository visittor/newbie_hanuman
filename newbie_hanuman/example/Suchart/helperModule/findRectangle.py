#! /usr/bin/env python

import cv2
import numpy as np
from transform import *
import time

def findSolidity(cnt):
	area = cv2.contourArea(cnt)
	hull = cv2.convexHull(cnt)
	hull_area = cv2.contourArea(hull)
	if hull_area == 0:
		return 0.0
	solidity = float(area)/hull_area
	return solidity

def findRectangle(img, cannyLwr = 50, cannyUpr = 150, areaThr = 0.01):
	assert len(img.shape) == 2

	thr = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,9,10)
	thr = 255-thr

	ret, cnts, hier = cv2.findContours(thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	cv2.imshow("canny", img)
	cv2.imshow("thr", thr)
	cv2.waitKey(1)

	rects = []
	if len(cnts)>0:
		for c in cnts:
			solidity = findSolidity(c)
			if cv2.contourArea(c) < areaThr*(img.shape[0]*img.shape[1]) or solidity < 0.9:
				continue
			peri = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.1*peri, True)
			# print len(approx)
			if len(approx) == 4:
				# print findSolidity(c)
				rects.append(approx.reshape(-1,2))
	return rects

def calArea(points):
	rect = order_points(points)
	(tl, tr, br, bl) = rect

	# compute the width of the new image, which will be the
	# maximum distance between bottom-right and bottom-left
	# x-coordiates or the top-right and top-left x-coordinates
	widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
	widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
	maxWidth = max(int(widthA), int(widthB))

	# compute the height of the new image, which will be the
	# maximum distance between the top-right and bottom-right
	# y-coordinates or the top-left and bottom-left y-coordinates
	heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
	heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
	maxHeight = max(int(heightA), int(heightB))
	return maxHeight * maxWidth

if __name__ == "__main__":

	org = cv2.imread("data/sample4.jpg",1)
	img = cv2.cvtColor(org, cv2.COLOR_BGR2GRAY)

	rects = findRectangle(img)
	# print rects
	orderedRects = []
	for rect in rects:
		orderedRects.append(order_points(rect))

	for rect in orderedRects:
		for i in range(4):
			cv2.line(org, tuple(rect[i]), tuple(rect[(i+1)%4]), 255,3)

	cv2.imshow("org", org)

	for rect in rects:
		dst = four_point_transform(img, rect)
		unBinary = dst.copy()
		binarize_image(dst)
		cv2.imshow("img", dst)
		cv2.imshow('unBinary', unBinary)
		# cv2.imwrite(str(time.time())+".png", dst)
		if cv2.waitKey(0) & 0xFF == 27:
			break

	cv2.destroyAllWindows()