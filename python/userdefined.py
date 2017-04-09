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


# def 







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
    # return a list of nodes start from the s to e, with a specific step 
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


def sampleCE(workspaceBound = [-4.5,3.5,-2.2,2.2,0.21,1.54]):
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

def sampleCQ(workspaceBound = [-4.5,3.5,-2.2,2.2,0.21,1.54]):
    x = np.random.uniform(workspaceBound[0],workspaceBound[1])
    y = np.random.uniform(workspaceBound[2],workspaceBound[3])
    z = np.random.uniform(workspaceBound[4],workspaceBound[5])


    q1 = np.random.uniform(0,2*np.pi)
    q3 = 0 #np.random.uniform(0,2*np.pi)

    while 1:
        q2 = np.abs(np.random.normal(0,np.pi/4))
        if q2 <= np.pi/2:
            break

    return [x,y,z] + list(tf.quaternion_from_euler(q1,q2,q3,'rzxz'))


def E2Q(x):
    return x[0:3] + list(tf.quaternion_from_euler(x[3],x[4],x[5],'rzxz'))



def genCQ(x,y,z,q1,q2,q3):
    # generate a quaternion by parameters
    sq32 = sin(q3/2)
    sq1 = sin(q1)
    print sq32
    print sq1
    return [x,y,z,cos(q3/2),sq32*sq1*cos(q2),sq32*sq1*sin(q2),sq32*cos(q1)]

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