import numpy as np
from scarakinematics import SCARAKinematics

from pprint import pprint

kinematics = SCARAKinematics()

q_previous = np.array([0,0,0,0,0,0])
pos = np.array([150,300,25])
orientation = np.array([0,np.pi,0])

q = kinematics.inverseKinematicsCalculate(pos,orientation,"zyz",q_previous,option=2)
(H,H_e,R_e,p_e) = kinematics.forwardKinematics(q)

print(p_e)

# Q1 = np.genfromtxt("Q1.csv",delimiter=",")
# Q3 = np.genfromtxt("Q3.csv",delimiter=",")
# poten_map = np.genfromtxt("potentialMap.csv",delimiter=" ")


# # find nearest
# q1_nearest = (Q1-q[0])**2 + (Q3-q[2])**2
# val = q1_nearest.min()
# idx = np.where(q1_nearest == val)

# print(idx)
# print(Q1[idx[0],idx[1]])
# print(Q3[idx[0],idx[1]])

