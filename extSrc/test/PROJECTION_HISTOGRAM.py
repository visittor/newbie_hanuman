#! /usr/bin/env python
import cv2
import numpy as np

IMAGE_SIZE = 50

DATA_SET_NAME = "PHFull1Font"
FILENAME = "{}-imagesize-{}"
FILENAME = FILENAME.format(	DATA_SET_NAME,
							IMAGE_SIZE)

def createProjectionHistogram(image):
	v = np.sum(image, axis=0) / (image.shape[1])
	h = np.sum(image, axis=1) / (image.shape[0])
	features = np.hstack((v,h)).astype(np.float64)
	# features = features / np.linalg.norm(features)
	features = features / 255
	return features