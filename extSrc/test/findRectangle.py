#! /usr/bin/env python

import cv2
import numpy as np
from transform import *
import time

def findRectangle(img, cannyLwr = 50, cannyUpr = 150, areaThr = 0.01):
	assert len(img.shape) == 2


	canny = cv2.Canny(img, cannyLwr, cannyUpr)

	ret, cnts, hier = cv2.findContours(canny.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	# blank = np.zeros(img.shape, dtype=np.uint8)
	# cv2.drawContours(blank, cnts, -1, 255, 1)
	cv2.imshow("canny", canny)
	# cv2.waitKey(0)

	rects = []
	if len(cnts)>0:
		for c in cnts:
			if cv2.contourArea(c) < areaThr*(img.shape[0]*img.shape[1]):
				continue
			peri = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.1*peri, True)
			# print len(approx)
			if len(approx) == 4:
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