import numpy as np
# TODO : Use configobj for set configuration of robot later

class RobotConfiguration:
	ground_base = 0 # Constant
	# Unit of lenght/height is mm
	h1 = 99.90
	h2 = 8
	l1 = 46
	h3 = 54.02
	l2 = 380
	h4 = 42.15
	l3 = 320
	h5 = 43.48
	h67 = 136.5900

	q_init = [0,0,0,0,0,0]

	type_of_joint = [1,0,1,1,1,1]
	
	dh_table = np.array([[0,   h2,   l1,  0],
						[0,   h3,   l2,  0],
						[0,   h4,   l3,  0],
						[0,  -h5,    0,  np.pi/2],
						[0,    0,    0,  np.pi/2],
						[0,  h67,    0,  0]])
	base_frame = np.vstack((np.hstack((np.eye(3),np.array([[0],[0],[h1]]))),np.array([0,0,0,1])))
	pi = np.pi
	q_upper = np.array([pi, 750, 3*pi/4, 3*pi/4,   9.1*pi/18,  pi])
	q_lower = np.array([-pi, 0, -0.5*pi/4,-3*pi/4,  -30*pi/180, -pi])