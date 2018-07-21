from visionManager.visionModule import KinematicModule
from newbie_hanuman.msg import suchartPostDictMsg, suchartVisionMsg

from newbie_hanuman.msg import SuchartFeedback

from geometry_msgs.msg import Polygon, Point32

import rospy

import numpy as np
import cv2

import sys

PATH = sys.path[0]

def getJsPosFromName(Js, name):
	indexJs = Js.name.index(name)
	return Js.position[indexJs]

class Kinematic(KinematicModule):
	# OFF1 = 7.6
	OFF1 = 7.6
	L1 = 0.2
	L2 = 0.3
	H1 = 0.6
	H2 = 0.5
	def __init__(self):
		super(Kinematic, self).__init__()

		path = "/".join(PATH.split("/")[:]+["output_matrix.npz"])
		intrinMat = np.load(path)["camera_matrix"]
		self.set_IntrinsicCameraMatrix(intrinMat)

		self.objectsMsgType = suchartVisionMsg
		self.posDictMsgType = suchartPostDictMsg

		self.motorCortexTopicName = "/spinalcord/suchart_status"
		self.motorCortexMsgType = SuchartFeedback

		## Add plane
		## TODO : re-create plane after known a exact position of robot base.

		tranVec = np.array([-2.5,0,0.25],float)
		rotVec = np.array([0,0,0],float)
		Hplane1 = self.create_transformationMatrix(tranVec, rotVec, 'zyz')

		self.add_plane(	"ground", Hplane1, 
						(0,10), (-5,5), (-1,1))

		tranVec = np.array([-2.5,4.75,5],float)
		rotVec = np.array([-np.pi/2,np.pi/2,np.pi/2],float)
		Hplane2 = self.create_transformationMatrix(tranVec, rotVec, 'zyz')

		self.add_plane(	"left", Hplane2, 
						(0,10), (-5,5), (-1,1))

		tranVec = np.array([-2.5,-4.75,5],float)
		rotVec = np.array([-np.pi/2,-np.pi/2,np.pi/2],float)
		Hplane3 = self.create_transformationMatrix(tranVec, rotVec, 'zyz')

		self.add_plane(	"right", Hplane3, 
						(0,10), (-5,5), (-1,1))

		# self.H1_left = self.getInverseHomoMat(Hplane2)
		# self.H1_right = self.getInverseHomoMat(Hplane3)
		self.H1_ground = Hplane1
		self.H1_left = Hplane2
		self.H1_right = Hplane3

		## for visualize
		x, y = np.indices((5,5))
		z = np.zeros((25,1), float)
		points = np.hstack((x.reshape(-1,1), y.reshape(-1,1))).astype(float)
		points[:,0] *= 0.25
		points[:,1] = 0.25*points[:,1] - 0.5
		points = np.hstack((points,z))
		self.points = points.copy()*10

		# print self.points

		self.point2D1 = None
		self.point2D2 = None
		self.point2D3 = None

		self.pattern = [	[0, 4],
							[5, 9],
							[10, 14],
							[15, 19],
							[20, 24],
							[0, 20],
							[1, 21],
							[2, 22],
							[3, 23],
							[4, 24]
						]

		self.worldCoors = []
		self.labels = []
		self.rects = []

	def __processRConfig(self, rconfig):
		if rconfig is None:
			return 0.0
		if not "joint1" in rconfig.currPosition.name:
			return 0.0
		indx = rconfig.currPosition.name.index("joint1")
		j1 = rconfig.currPosition.position[indx]
		return j1

	def __createHomoCam(self, Js, rconfig=None):
		## FK for camera
		q1 = getJsPosFromName(Js, "pan")
		q2 = getJsPosFromName(Js, "tilt")

		j1 = self.__processRConfig(rconfig)
		# j1 = np.pi

		tranOffset = np.array([2.5,-0.0,self.OFF1],float)
		rotOffset = np.array([j1-np.pi,0,0],float)
		HOffset = self.create_transformationMatrix(tranOffset, rotOffset, 'zyz', order = "rot-first")
		# print j1

		rot =[	[np.sin(q1), 	np.cos(q1)*np.sin(q2),	np.cos(q1)*np.cos(q2)],
				[-np.cos(q1),	np.sin(q1)*np.sin(q2),	np.sin(q1)*np.cos(q2)],
				[0,				-np.cos(q2),			np.sin(q2)]]
		rot = np.array(rot, float)
		tran = [[self.L2*np.cos(q1)*np.cos(q2)+self.L1*np.cos(q1)-self.H2*np.cos(q1)*np.sin(q2)],
				[self.L2*np.sin(q1)*np.cos(q2)-self.H2*np.sin(q1)*np.sin(q1)+self.L1*np.sin(q1)],
				[self.L2*np.sin(q2)+self.H2*np.cos(q2)+self.H1]]
		tran = np.array(tran, float)
		Homo = np.hstack((rot, tran))
		Homo = np.vstack((Homo, np.array([0,0,0,1])))
		return np.matmul(HOffset, Homo)

	def __rect2CntAndCenter(self, rect):
		cnt = np.zeros((4,2), dtype = float)
		center = np.zeros((2), dtype = float)
		for i,p in enumerate(rect.points):
			cnt[i,0] = p.x
			cnt[i,1] = p.y
			center[0] += 0.25*p.x
			center[1] += 0.25*p.y
		return cnt, center

	def clipLine(self, point1, point2, shape):
		if point1 is None or point2 is None:
			return False, point1, point2

		if (-1000000>=point1).any() or (point1>=1000000).any():
			return False, point1, point2

		if (-1000000>=point2).any() or (point2>=1000000).any():
			return False, point1, point2

		point1 = point1.astype(int)
		point2 = point2.astype(int)
		ret, pt1, pt2 = cv2.clipLine( (0,0,shape[1],shape[0]), tuple(point1), tuple(point2))
		# print ret
		return ret, pt1, pt2

	def kinematicCalculation(self, objMsg, Js, rconfig=None):
		H = self.__createHomoCam(Js, rconfig)

		self.worldCoors = []
		self.labels = []
		self.rects = []
		for l, rect in zip(objMsg.labels, objMsg.rects):
			cnt, center = self.__rect2CntAndCenter(rect)
			temp = self.calculate3DCoor(cnt, HCamera=H)
			# r = [self.calculate2DCoor(i[1], i[0], HCamera=H) for i in temp if i[0] is not None]
			# print temp
			poly = Polygon()
			for n, p in temp:
				# print p
				if n is None:
					continue
				p2D = self.calculate2DCoor([p], n, HCamera=H)[0]
				if p2D is not None:
					poly.points.append(Point32(x=p2D[0], y=p2D[1]))

			center = self.calculate3DCoor([center], HCamera=H)[0]
			fromBase = None
			if center[0] is None:
				continue
			elif center[0] == "ground":
				# center[1] = np.hstack((center[1],1))
				center_aux = np.hstack((center[1],1)).T
				fromBase = np.matmul(self.H1_ground, center_aux)[:-1]/10.0
			elif center[0] == "right":
				center_aux = np.hstack((center[1],1)).T
				fromBase = np.matmul(self.H1_right, center_aux)[:-1]/10.0
			elif center[0] == "left":
				center_aux = np.hstack((center[1],1)).T
				fromBase = np.matmul(self.H1_left, center_aux)[:-1]/10.0
			self.worldCoors.append(Point32(x=fromBase[0],y=fromBase[1],z=fromBase[2]))
			# print self.H1_right
			# print center_aux
			# print fromBase
			self.labels.append(l)
			self.rects.append(poly)

		self.point2D1 = self.calculate2DCoor(self.points, "ground", HCamera=H)
		self.point2D1 = np.array(self.point2D1)

		self.point2D2 = self.calculate2DCoor(self.points, "left", HCamera=H)
		self.point2D2 = np.array(self.point2D2)

		self.point2D3 = self.calculate2DCoor(self.points, "right", HCamera=H)
		self.point2D3 = np.array(self.point2D3)

		msg = suchartPostDictMsg(rects = objMsg.rects,
								labels = objMsg.labels,
								world_coor = self.worldCoors)
		msg.header.stamp = rospy.Time.now()

		return msg

	def loop(self):
		blank = np.zeros((480,640,3), dtype=np.uint8)
		if self.point2D1 is not None:
			for i in self.pattern:
				point1 = self.point2D1[i[0]]
				point2 = self.point2D1[i[1]]
				# print point1, point2
				ret, pt1, pt2 = self.clipLine(point1, point2, blank.shape)
				if ret:
					# print "Draw"
					cv2.line(blank, pt1, pt2, (255,0,0), 4)

		if self.point2D2 is not None:
			for i in self.pattern:
				point1 = self.point2D2[i[0]]
				point2 = self.point2D2[i[1]]
				ret, pt1, pt2 = self.clipLine(point1, point2, blank.shape)
				if ret:
					cv2.line(blank, pt1, pt2, (0,0,255), 4)

		if self.point2D3 is not None:
			for i in self.pattern:
				point1 = self.point2D3[i[0]]
				point2 = self.point2D3[i[1]]
				ret, pt1, pt2 = self.clipLine(point1, point2, blank.shape)
				# print ret
				if ret:
					cv2.line(blank, pt1, pt2, (0,255,0), 4)

		for rect,label in zip(self.rects, self.labels):
			cen_x = 0
			cen_y = 0
			cnt = np.zeros((4,1,2))
			for i,p in enumerate(rect.points):
				cen_x += 0.25*p.x
				cen_y += 0.25*p.y
				cnt[i,0,0] = p.x
				cnt[i,0,1] = p.y
				# p1 = (int(p.x), int(p.y))
				# p2 = (int(rect.points[(i+1)%4].x), int(rect.points[(i+1)%4].y))
				# cv2.line(blank, p1, p2, (255,255,255),-1)
			# print cnt
			cv2.drawContours(blank, [cnt.astype(int)], 0, (255,255,255), -1)
			cv2.putText(blank, str(label), (int(cen_x),int(cen_y)),
					cv2. FONT_HERSHEY_SIMPLEX,
					1, (0,255,0), 7)
			cv2.putText(blank, str(label), (int(cen_x),int(cen_y)),
					cv2. FONT_HERSHEY_SIMPLEX,
					1, (0,0,255), 3)

		cv2.imshow("img", blank)
		cv2.waitKey(1)

	def end(self):
		cv2.destroyAllWindows()