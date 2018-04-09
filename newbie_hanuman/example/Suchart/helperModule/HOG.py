import cv2
import numpy as np

IMAGE_SIZE = 50
BLOCKSIZE = 20
BLOCKSTRIBE = BLOCKSIZE/2
CELLSIZE = BLOCKSIZE
NBINS = 9
DERIVAPERTURE = 1
WINSIGMA = -1.
HISTOGRAMNORMTYPE = 0
L2HYSTHRESHOLD = 0.2
GAMMACORRECTION = 1
NLEVELS = 64
SINEDGRADIENTS = True

DATA_SET_NAME = "HOGFull1Font"
FILENAME = "{}-imagesize-{}-block-{}-cell-{}-bin-{}-sined-{}"
FILENAME = FILENAME.format(	DATA_SET_NAME,
							IMAGE_SIZE,
							BLOCKSIZE,
							CELLSIZE,
							NBINS,
							SINEDGRADIENTS)


def createHOGDescription(block):
	winSize = 			(IMAGE_SIZE,	IMAGE_SIZE)
	blockSize = 		(block, 		block)
	blockStride = 		(block/2, 		block/2)
	cellSize = 			(block,			block)
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