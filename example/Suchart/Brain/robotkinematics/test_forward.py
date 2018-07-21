import numpy as np
from scarakinematics import SCARAKinematics


kinematics = SCARAKinematics()

# q = np.array([0.,1.83131681,187.42,-1.95820958,0.,0.,0.])
q = np.array([0,0,0,0,0,0,0])

H = kinematics.forwardKinematics(q)[3]

print H

# H2 = kinematics.forwardKinematics(q)[0]

# print H2[:3,:3,3]

# H3 = kinematics.forwardKinematics(q)[0]

# print H3[:3,:3,3]

# H4 = kinematics.forwardKinematics(q)[0]

# print H4[:3,:3,3]