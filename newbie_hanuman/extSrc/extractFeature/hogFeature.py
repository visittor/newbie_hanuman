#! /usr/bin/env python
import cv2
import numpy as np
import argparse
import glob
import os
from random import shuffle

WIN_SIZE = 20
BLOCKSIZE = 10
BLOCKSTRIBE = BLOCKSIZE/2
CELLSIZE = 10
NBINS = 9
DERIVAPERTURE = 1
WINSIGMA = -1.
HISTOGRAMNORMTYPE = 0
L2HYSTHRESHOLD = 0.2
GAMMACORRECTION = 1
NLEVELS = 64
SINEDGRADIENTS = True

DATA_SET_NAME = "HOGFeature"
FILENAME = "{}-win-{}-block-{}-cell-{}-bin-{}-sined-{}.npz"
FILENAME = FILENAME.format(	DATA_SET_NAME,
							WIN_SIZE,
							BLOCKSIZE,
							CELLSIZE,
							NBINS,
							SINEDGRADIENTS)



def createHOGDescription():
	winSize = 			(WIN_SIZE,		WIN_SIZE)
	blockSize = 		(BLOCKSIZE, 	BLOCKSIZE)
	blockStride = 		(BLOCKSTRIBE, 	BLOCKSTRIBE)
	cellSize = 			(CELLSIZE,		CELLSIZE)
	nbins = 			NBINS
	derivAperture = 	DERIVAPERTURE
	winSigma = 			WINSIGMA
	histogramNormType = HISTOGRAMNORMTYPE
	L2HysThreshold = 	L2HYSTHRESHOLD
	gammaCorrection = 	GAMMACORRECTION
	nlevels = 			NLEVELS
	signedGradients = 	SINEDGRADIENTS
	 
	hog = cv2.HOGDescriptor(winSize,blockSize,blockStride,cellSize,nbins,derivAperture,winSigma,histogramNormType,L2HysThreshold,gammaCorrection,nlevels, signedGradients)

	return hog


if __name__ == "__main__":
	ap = argparse.ArgumentParser()
	ap.add_argument("-i", "--imagepath", required=True,
					help="path to folder which contain images.")
	ap.add_argument("-s", "--save", action='store_true',
					help="To save dataset")
	args = vars(ap.parse_args())

	print os.path.abspath(args["imagepath"])
	filesList = glob.glob(os.path.abspath(args["imagepath"])+"/*.*")

	datas = []
	labels = []

	hog = createHOGDescription()

	for name in filesList:
		assert name.split("_")[0].isdigit() == False
		label = int(name.split("/")[-1].split("\\")[-1].split("_")[0])
		image = cv2.imread(name,0)
		assert image is not None
		cv2.resize(image, (WIN_SIZE,WIN_SIZE), image)
		feature = hog.compute(image).reshape(-1)
		feature = np.hstack((feature,np.array([label],feature.dtype)))

		datas.append(feature)

	shuffle(datas)
	datas = np.array(datas)
	print datas
	X = datas[:,:-1]
	y = datas[:,-1]
	print y

	if args["save"]:
		np.savez(FILENAME, X=X, y=y)