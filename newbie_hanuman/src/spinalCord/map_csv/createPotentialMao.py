#!/usr/bin/env python
import numpy as np 
import cv2
import matplotlib.pyplot as plt

def createPotentialMap(map_):
	_,cnts,_ = cv2.findContours(map_,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	print len(cnts)
	potens = []
	for cnt in cnts:
		potenMap = np.zeros(map_.shape, dtype=float)
		for y in range(map_.shape[0]):
			for x in range(map_.shape[1]):
				potenMap[y,x] = cv2.pointPolygonTest(cnt, (x,y), True)
		potens.append(potenMap.copy())

	PotentialMap = np.zeros(map_.shape, dtype=float)
	PotentialMap = potens[0]
	for i in range(1, len(potens)):
		indx = np.where(potens[i]>PotentialMap)
		PotentialMap[indx] = potens[i][indx]

	return PotentialMap

if __name__ == "__main__":

	map_ = np.genfromtxt("map_suchart.csv", delimiter = ",")
	indxLaser = np.where(map_ == 2)
	map_[indxLaser] = 0

	POTEN_MAP = createPotentialMap(map_.astype(np.uint8))

	# POTEN_MAP = createPotentialMap(poten_map.astype(np.uint8))
	# print POTEN_MAP
	max_ = np.amax(np.abs(POTEN_MAP), axis=None)
	POTEN_MAP = POTEN_MAP/max_
	POTEN_MAP[np.where(POTEN_MAP>=0)] = 1

	np.savetxt("potentialMap.csv", POTEN_MAP)

	plt.imshow(POTEN_MAP, cmap="hot")
	plt.colorbar()
	plt.show()