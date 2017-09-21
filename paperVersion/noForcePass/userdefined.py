import time
import openravepy
import sys
import numpy as np
from numpy import sin,cos 
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
# import random
import transformationFunction as tf 
import kdtree
import scipy.spatial as spatial
import cgkit.all as cg

# weight of quaternion in distance 
QWEIGHT = 0.1

M2f = np.array([[  0.25,0.,     2.5,  -31.25], [  0.25,  -2.5,    0.,    31.25], [  0.25,   0.,    -2.5,  -31.25], [  0.25,   2.5,    0.,    31.25]])
f2M = np.array([[ 1.,     1.,     1.,     1.   ], [ 0.,    -0.2,    0.,     0.2  ], [ 0.2,    0.,    -0.2,    0.,   ], [-0.008,  0.008, -0.008,  0.008]])


J = np.array([[0.04,0,0],
				[0,0.04,0],
				[0,0,0.07]])
Jinv = np.array([[ 25.        ,   0.        ,   0.        ],
					[  0.        ,  25.        ,   0.        ],
					[  0.        ,   0.        ,  14.28571429]])









def S2CQ(s):
	# return the configuration part of a state point
	# s = [x,v,Q,W]
	return s[0:3]+s[6:10]


def nnNodeIndexPathSQ(s,path,startIndex):
	# return the nearest node index in path of state point s, index smaller than startIndex won't count
	minL = 1000
	for i in range(startIndex,len(path)):
		dist = distCSQ(s,path[i])
		if minL > dist:
			minIndex = i
			minL = dist
	return minIndex

def sampleFCSAP(s,c):
	# sample force for altitude control and position control
	# from state s to goal c
	# the goal state is with zero velocity

	# attitude control parameter
	kR = 1;
	kO = 3;
	c2 = 1;
	epR = 1;	


	# postion control parameter
	stdev = 0.5
	kz = 2
	kx = 5
	ky = kx
	# ky = 0.5
	kv = 10




	R = Q2R(s[6:10])
	X = s[0:3]
	V = np.array(s[3:6])
	O = np.array(s[10:13])


	dR = Q2R(c[3:7])
	dX = np.array(c[0:3])
	dV = np.array([0,0,0])
	dO = dV

	zw = np.matmul(R,[0,0,1])
	zproj = zw[2]
	average = 4.5325/zproj


	eX = dX - X
	eXb = np.matmul(R.T,eX) #error on X in body Frame
	eVb = np.matmul(R.T,dV - V)

	eXx = eXb[0]
	eXy = eXb[1]
	eXz = eXb[2]


	average = average + eXz*kz

	# f1 = np.random.normal(average + 0.5*kx*eXx, stdev)
	# f2 = np.random.normal(average + 0.5*ky*eXy, stdev)
	# f3 = np.random.normal(average - 0.5*kx*eXx, stdev)
	# f4 = np.random.normal(average - 0.5*ky*eXy, stdev)


	f1 = np.random.normal(0.5*kx*eXx, stdev)
	f2 = np.random.normal(0.5*ky*eXy, stdev)
	f3 = np.random.normal( - 0.5*kx*eXx, stdev)
	f4 = np.random.normal( - 0.5*ky*eXy, stdev)



	eR = 0.5*(np.matmul(dR.T,R) - np.matmul(R.T,dR))


	eR =  vee(0.5*(np.matmul(dR.T,R)-np.matmul(R.T,dR)))
	eO = O - np.matmul(np.matmul(R.T,dR),dO)

	# an approximation of the attitude control
	M = np.array([average*4,0,0,0])
	M[1:4] = - kR*eR - kO*eO + np.cross(O,np.matmul(J,O)) 



	f = np.matmul(M2f,M) + np.array([f1,f2,f3,f4])


	f = list(f)

	return f




def sampleFSC(s,c):
	# sample force vector at state s while setting c as the goal
	# error is the position error with the goal

	# stdev = 1

	# q = s[6:10]
	# R = Q2R(q)
	# zw = np.matmul(R,[0,0,1])
	# zproj = zw[2]
	# average = 4.5325/zproj

	# # for the control part
	# R = Q2R(s[6:10])
	# X = Q2R(s[0:3])
	# V = np.array(s[3:6])
	# O = np.array(s[10:13])

	# dR = Q2R(c[3:7])
	# dX = np.array(c[0:3])
	# dV = np.array([0,0,0])
	# dO = dV

	# eR =  vee(0.5*(np.matmul(dR.T,R)-np.matmul(R.T,dR)))
	# eO = O - np.matmul(np.matmul(R.T,dR),dO)

	stdev = 0.6

	R = Q2R(s[6:10])
	X = s[0:3]
	V = np.array(s[3:6])
	O = np.array(s[10:13])

	dR = Q2R(c[3:7])
	dX = np.array(c[0:3])
	dV = np.array([0,0,0])
	dO = dV

	zw = np.matmul(R,[0,0,1])
	zproj = zw[2]
	average = 4.5325/zproj


	eX = dX - X
	eXb = np.matmul(R.T,eX) #error on X in body Frame
	eVb = np.matmul(R.T,dV - V)


	kz = 1
	kx = 0.5
	ky = 0.5
	kv = 0.3

	eXb = eXb + eVb*kv

	eXx = eXb[0]
	eXy = eXb[1]
	eXz = eXb[2]


	average = average + eXz*kz

	f1 = np.random.normal(average + 0.5*kx*eXx, stdev)
	f2 = np.random.normal(average + 0.5*ky*eXy, stdev)
	f3 = np.random.normal(average - 0.5*kx*eXx, stdev)
	f4 = np.random.normal(average - 0.5*ky*eXy, stdev)
	f = [f1,f2,f3,f4]

	return f



def distCSQ(s,c,w = QWEIGHT):
	# return the distance between a state point and a configuration point
	# using quaternion
	# s = [x,v,Q,W]
	return distXQ(S2CQ(s),c,w)    

def sampleFS(s):
	# sample a fore vector
	# based on the current state and try to balance the copter
	# avf = 1.85*9.8/4
	stdev = 0.5

	q = s[6:10]
	R = Q2R(q)
	zw = np.matmul(R,[0,0,1])
	zproj = zw[2]
	average = 4.5325/(0.5*zproj+0.5)

	f1 = abs(np.random.normal(average, stdev))
	f2 = abs(np.random.normal(average, stdev))
	f3 = abs(np.random.normal(average, stdev))
	f4 = abs(np.random.normal(average, stdev))
	f = [f1,f2,f3,f4]
	return f

def sampleF():
	# sample a fore vector
	stdev = 1
	average = 5;
	f1 = abs(np.random.normal(average, stdev))
	f2 = abs(np.random.normal(average, stdev))
	f3 = abs(np.random.normal(average, stdev))
	f4 = abs(np.random.normal(average, stdev))
	f = [f1,f2,f3,f4]
	return f

def distXQ(a,b, w = QWEIGHT):
	# return the distance between two configuration
	ax = np.array(a[0:3])
	aq = np.array(a[3:7])
	bx = np.array(b[0:3])
	bq = np.array(b[3:7])

	return np.linalg.norm(ax - bx) + w* (1 - np.abs(np.dot(aq,bq)))

def nnNodeCQ(tree,node):
	# tree is a dictionary
	# node is a list
	# Configuratoin space
	# Using quaternion for orientation
	min = 10000
	for i in tree:
		iToNode = distXQ(node,i)        
		if iToNode < min:
			min = iToNode
			minIndex = i
	return minIndex

def xyzt(start,end,t):
	# return a interpolation of start to end at t
	return list((np.array(end)-np.array(start))*t + np.array(start))

def stepXQ(start,end,n): 
	# return a configuration sequence in the form of [X,Q]
	# n >=2
	if n == 2:
		return [start,end]
	qs = cg.quat(start[3:7])
	qe = cg.quat(end[3:7])
	xyzs = start[0:3]
	xyze = end[0:3]
	 
	nodes = []
	for i in range(0,n):
		t = float(i)/(n-1)
		# print t
		qt = cg.slerp(t, qs, qe, shortest=True)
		nodes.append(list(xyzt(xyzs,xyze,t)+[qt.w,qt.x,qt.y,qt.z])) 
	return nodes 

def stepNodesQ(start,end,step):
	# return a list of nodes start from the s to e, with a specific step 
	# the node is in the form of [X,Q]
	# the returned path exclude the start
	l = distXQ(start,end)
	if l <= step:
		return [end]
	else:
		n = int(np.ceil(l/step)) + 1
		nodes = stepXQ(start , end , n)
		del nodes[0]
		nodes.pop()
		nodes.append(end)
		return nodes


def discretePath(path, step = 0.1):
	# input a path and a max step, return a discreterized path with maximum step   
	newPath = [path[0]]
	for i in range(0,len(path)-1):
		NodeS = path[i]
		NodeE = path[i+1]
		seg = stepNodesQ(NodeS,NodeE,step) 
		newPath = newPath + seg

	return newPath


def step1NodeQ(start,end,step):
	# return a list of nodes start from the s to e, with a specific step 
	l = distXQ(start,end)
	if l <= step:
		return end
	else:
		t = step/l
		qs = cg.quat(start[3:7])
		qe = cg.quat(end[3:7])
		qt = cg.slerp(t, qs, qe, shortest=True)

		return list(xyzt(start[0:3],end[0:3],t)+[qt.w,qt.x,qt.y,qt.z])


def getpath(tree,goal):
	# get the path from a RRT tree
	# tree is in dictionary
	# path , goal is list
	path = [goal]

	while 1:
		if tree[tuple(path[0])] == tuple(path[0]):
			break
		path = [list(tree[tuple(path[0])])]+path

	return path





def nodesDist(x,y):
	return np.linalg.norm(np.asarray(x)-np.asarray(y))

def stepNodes(start,end,step):
	# return a list of nodes start from the s to e, with a specific step 
	l = nodesDist(start,end)
	if l <= step:
		return [end]
	else:
		n = int(np.ceil(l/step))
		delta = (np.asarray(end)-np.asarray(start))/l*step

		nodes = []
		for i in range(0,n-1):
			nodes.append(list(np.asarray(start)+delta*(i+1)))
		nodes.append(end)
		return nodes

def step1Node(start,end,step):
	# return a node steer to end from start
	l = nodesDist(start,end)
	if l <= step:
		return end
	else:
		return list(np.asarray(start)+(np.asarray(end)-np.asarray(start))/l*step)
		



def plotHist(x):
	# the histogram of the data
	n, bins, patches = plt.hist(x, 50, normed=1, facecolor='green', alpha=0.75)
	plt.xlabel('Smarts')
	plt.ylabel('Probability')
	plt.show()


def limitTo(a,lower,upper):
	if a <= lower:
		return lower
	if a >= upper:
		return upper
	return a

	# sample a anlge from

# [-4.5,3.5,-2.2,2.2,0.21,1.54]
def sampleCE(workspaceBound = [-4.5,4.5,-2.2,2.2,0.21,1.54]):
	x = np.random.uniform(workspaceBound[0],workspaceBound[1])
	y = np.random.uniform(workspaceBound[2],workspaceBound[3])
	z = np.random.uniform(workspaceBound[4],workspaceBound[5])

	q1 = np.random.uniform(0,2*np.pi)
	q3 = np.random.uniform(0,2*np.pi)

	while 1:
		q2 = np.abs(np.random.normal(0,np.pi/4))
		if q2 <= np.pi/2:
			break
	return [x,y,z,q1,q2,q3]

def sampleCQ(workspaceBound = [-4.5,4.5,-2.2,2.2,0.21,1.54]):
	x = np.random.uniform(workspaceBound[0],workspaceBound[1])
	y = np.random.uniform(workspaceBound[2],workspaceBound[3])
	z = np.random.uniform(workspaceBound[4],workspaceBound[5])


	q1 = np.random.uniform(0,2*np.pi)
	q3 = np.random.uniform(0,2*np.pi)
	# q3 = 0 #np.random.uniform(0,2*np.pi)

	# while 1:
	#     q2 = np.abs(np.random.normal(0,np.pi/2))
	#     if q2 <= np.pi/2:
	#         break
	q2 = np.random.uniform(0,0.5*np.pi)



	return [x,y,z] + list(tf.quaternion_from_euler(q1,q2,q3,'rzxz'))




def E2Q(x):
	return x[0:3] + list(tf.quaternion_from_euler(x[3],x[4],x[5],'rzxz'))

def Q2R(Q):
	# convert a quaternion to a rotation matrix
	# input must be a unit quaternion
	qw = Q[0]
	qx = Q[1]
	qy = Q[2]
	qz = Q[3]
	R = np.array([[1 - 2*qy**2 - 2*qz**2,   2*qx*qy - 2*qz*qw,   2*qx*qz + 2*qy*qw],
				[2*qx*qy + 2*qz*qw,   1 - 2*qx**2 - 2*qz**2,   2*qy*qz - 2*qx*qw],
				[2*qx*qz - 2*qy*qw,   2*qy*qz + 2*qx*qw   ,1 - 2*qx**2 - 2*qy**2]])

	return R


def genCQ(x,y,z,q1,q2,q3):
	# generate a quaternion by parameters
	sq32 = sin(q3/2)
	sq1 = sin(q1)
	# print sq32
	# print sq1
	return [x,y,z,cos(q3/2),sq32*sq1*cos(q2),sq32*sq1*sin(q2),sq32*cos(q1)]

def hat(v):
	# hat map of a vector
	# input an numpy array or list, output an numpy array
	return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])


def vee(A):
	# inverse of hat map
	return np.array([A[2,1],A[0,2],A[1,0]])

def cross(a, b):
	c = np.array([[a[1]*b[2] - a[2]*b[1]],
		 [a[2]*b[0] - a[0]*b[2]],
		 [a[0]*b[1] - a[1]*b[0]]])

	return c
def updateState(s1,u,ts):
	# update state x1 to x2 with control input u and time step ts
	# s uses position vector and quaternion to represent
	# s = [x,v,Q,W] Q is the position,velocity, attitude quaternion ad angular velocity  
	# the quaternion are translated to a rotation matrix for computation
	# then the rotatoin matrix is converted to quaternion before return
	# input and output are both lists
	# u     rotation speed of each motor
	# a     accelatation in inertial frame
	# x     position in inertial frame
	# v     velocity in inertial frame
	# Q     rotation quaternion of the body in the inertial frame
	# W     angular velocity in the body frame
	# M     moment vector in the body fixed frame
	# m     total mass of the drone
	# Rd    the derivetive of rotation matrix
	# J     inertia matrix
	# ctf   constant to convert force to torque: f*ctf = t
	# MV    moment vector f,mx,my,mz

	# J = np.array([[0.04,0,0],
	# 			[0,0.04,0],
	# 			[0,0,0.07]])
	# Jinv = np.array([[ 25.        ,   0.        ,   0.        ],
	# 				[  0.        ,  25.        ,   0.        ],
	# 				[  0.        ,   0.        ,  14.28571429]])
	m = 1.85
	d = 0.2
	ctf = 0.008
	g = 9.8
	e3 = np.array([0,0,1])


	MV = np.matmul(f2M,np.array([u[0],u[1],u[2],u[3]]))
	f = MV[0]
	M = MV[[1,2,3]]



	x1 = np.array(s1[0:3])
	v1 = np.array(s1[3:6])
	Q1 = np.array(s1[6:10])
	W1 = np.array(s1[10:13])
	R1 = Q2R(Q1)

	R1d = np.matmul(R1,hat(W1))

	a  = - g*e3+(f*np.matmul(R1,e3))/m
	

	W1d = np.matmul( Jinv, M - np.cross(W1,np.matmul(J,W1)))

	x2 = x1 + ts*v1
	v2 = v1 + ts*a
	R2 = R1 + ts*R1d
	W2 = W1 + ts*W1d

	R2t = np.identity(4)
	R2t[0:3,0:3] = R2
	Q2 =  tf.quaternion_from_matrix(R2t)

	s2 = list(x2)+list(v2)+list(Q2)+list(W2)
	return s2



# start  = [1,2,3,1,0,0,0]
# end = [3,2,5,0,1,0,0]

# # stepNodesQ
# for i in stepNodesQ(start,end,0.1):
#     print i#,distXQ(i,start),distXQ(i,end)
# a = np.array([1,2,3])
# print np.dot(a,a)

# print "test update state"

# s2 = [0,0,0,0,0,0,1,0,0,0,0,0,0]
# # s1 = [1,1,1,1,0,0,0,0.2,0.2,0.2,0.1,0.1,-0.1]

# u = [0,0,0,0]

# ts = 0.02
# t = range(0,100)



# for tt in t:
#     s2 = updateState(s2,u,ts)

#     x1 = np.array(s2[0:3])
#     v1 = np.array(s2[3:6])
#     Q1 = np.array(s2[6:10])
#     W1 = np.array(s2[10:13])
#     E1 = tf.euler_from_quaternion(Q1)

#     print x1
#     print v1
#     print Q1
#     print W1

# axarr[0, 0].plot(x, y)
# axarr[0, 0].set_title('Axis [0,0]')
# axarr[0, 1].scatter(x, y)
# axarr[0, 1].set_title('Axis [0,1]')
# axarr[1, 0].plot(x, y ** 2)
# axarr[1, 0].set_title('Axis [1,0]')
# axarr[1, 1].scatter(x, y ** 2)
# axarr[1, 1].set_title('Axis [1,1]')
# # Fine-tune figure; hide x ticks for top plots and y ticks for right plots
# plt.setp([a.get_xticklabels() for a in axarr[0, :]], visible=False)
# plt.setp([a.get_yticklabels() for a in axarr[:, 1]], visible=False)



# q = [1,0,0,0]
# q0 = tf.random_quaternion()
# r0 = Q2R(q0)
# print hat([1,2,3])
# print tf.euler_from_matrix(r0)
# print tf.euler_from_quaternion(q0)



# print hat([1,2,3])
# print [1,2,3,4][3]

# v = [1,2,3]
# np.array([0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0])

# print sampleRotation()

# # print np.random.normal(0, 3.14, 1)
# eM = tf.euler_matrix(0,0,1.57)


# print eM
# print np.random.uniform(0,3)
# # print 1
# print tf.random_rotation_matrix()
# print np.dot(tf.random_quaternion(),tf.random_quaternion())
# print np.matmul(tf.random_rotation_matrix(),tf.random_rotation_matrix())


# start = tf.random_quaternion();
# print start
# print tuple(start)
# a = {tuple(start):tuple(start)}
# print a
# print a[tuple(start)]
# x =     [sampleC()];
# KDtree = kdtree.create(x)
# print x
# for i in range(0,200):
#     # x.append(sampleC()[5])
#     newnode =sampleC() 
#     x.append(newnode)
#     KDtree.add(newnode)
# # print x


# kdtree.visualize(KDtree)
# node = sampleC()
# print node
# a = KDtree.search_nn(node)[0].data

# print a
# aa = 1000
# for i in x:
#     # print "this is i"
#     # print np.asarray(i)
#     # print type(np.asarray(i))
#     # print np.linalg.norm(np.asarray(i),np.asarray(i))
#     aa = min(aa,np.linalg.norm(np.asarray(i)-np.asarray(node)))


# print aa

# print np.linalg.norm(np.asarray(a)-np.asarray(node))



# print nodesDist(1,3)
# print nodesDist([1,2,3],[4,5,6])



# print np.power(nodesDist([[2,3,4],[2,3,4]],[[1,2,3],[1,2,3]]),2)
# print np.asarray([[2,3,4],[2,3,4]])


# print np.floor(3.4)

# yy = [];
# yy.append([1,2,3])
# yy.append([1,2,5])
# print yy


# print ""
# print step1Node([30,40],[0,0.1],5)


# a = {(2,3):(1,2),(1,2):(1,2),(3,4):(1,2),(5,6):(3,4),(9,8):(3,4)};
# print a
# print getpath(a,[5,6])





# print ""





# points = np.array([ (3, 4), (1, 2),(4, 5),(6,7),(2,5),(2,4)])
# points = [[1,2],[4,5],[5,2]]
# point_tree = spatial.KDTree(points)
# This finds the index of all points within distance 1 of [1.5,2.5].
# print(point_tree.query_ball_point([1.5, 2.5], 2))
# print point_tree.query([1.5, 2.5])
# print point_tree.data[point_tree.query([1.5, 2.5])[1]]
# [0]

# # This gives the point in the KDTree which is within 1 unit of [1.5, 2.5]
# print(point_tree.data[point_tree.query_ball_point([1.5, 2.5], 1)])
# # [[1 2]]

# # More than one point is within 3 units of [1.5, 1.6].
# print(point_tree.data[point_tree.query_ball_point([1.5, 1.6], 3)])
# # [[1 2]
# #  [3 4]]


# x = []
# for i in range(0,1000):
#     while 1:
#         q1 = np.random.normal(np.pi/4,np.pi/8)
#         if np.abs(q1-np.pi/4) <= np.pi/4:
#             break
#     x.append(q1)
# plotHist(x)

# startconfig = [ 4.0,-1.5 ,0.2 ,1 ,0.0, 0.0, 0.0 ]
# print E2Q(startconfig)