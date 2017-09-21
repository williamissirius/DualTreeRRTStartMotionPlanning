import cgkit.all as cg
# import numpy as np
import numpy as np
import userdefined as us

import matplotlib.pyplot as plt
# q0 = cg.quat([1,0,0,0])
# # print q 
# q1 = cg.quat([0,1,0,0])

# # def quat2list(q):
# # 	return 

# for i in range(0,101):
# 	# qn1 = cg.slerp(0.01*i+0.01, q0, q1, shortest=True)
# 	qn = cg.slerp(0.01*i, q0, q1, shortest=True)
	

# 	print i,qn.w
# dic = {(1,2,3):(1,2,3),
# 		(2,4,1):(4,12,2),(5,2,4):(2,3,4)}

# for i in dic:
# 	print i

# print dic[0]
	

# R = array(range(0,9)).reshape([3,3])
# print R
# print us.vee(R)
# print R.T



J = np.array([[0.04,0,0],
            [0,0.04,0],
            [0,0,0.07]])
Jinv = np.array([[ 25.        ,   0.        ,   0.        ],
                [  0.        ,  25.        ,   0.        ],
                [  0.        ,   0.        ,  14.28571429]])
m = 1.85
d = 0.2
ctf = 0.008
g = 9.8
e3 = np.array([0,0,1])


f2M = np.array([[1,1,1,1],[0,-d,0,d],[d,0,-d,0],[-ctf,ctf,-ctf,ctf]])
print f2M

M2f = np.linalg.inv(f2M)
print M2f




M2f = np.array([[  0.25,0.,     2.5,  -31.25], [  0.25,  -2.5,    0.,    31.25], [  0.25,   0.,    -2.5,  -31.25], [  0.25,   2.5,    0.,    31.25]])
f2M = np.array([[ 1.,     1.,     1.,     1.   ], [ 0.,    -0.2,    0.,     0.2  ], [ 0.2,    0.,    -0.2,    0.,   ], [-0.008,  0.008, -0.008,  0.008]])
print M2f
print f2M
print np.matmul(M2f,f2M)
# MV = np.matmul(,np.array([u[0],u[1],u[2],u[3]]))


print 3*M2f


X = np.linspace(-np.pi, np.pi, 256, endpoint=True)
print X
C, S = np.cos(X), np.sin(X)

plt.plot(X, C)
plt.plot(X, S)

plt.show()
print range(0,1)