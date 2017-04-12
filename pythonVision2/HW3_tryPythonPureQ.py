#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for EECS 598 Motion Planning
import time
import random
import openravepy
import userdefined as us
import kdtree
import transformationFunction as tf 
import numpy as np
from random import randrange
import math
#### YOUR IMPORTS GO HERE ####
handles = [];
	
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
	from openravepy import *
	from numpy import *





def calculateL2(state1, state2):
	i = 0
	s = 0.0
	while i < 6:
		s = s + (state1[i] - state2[i])**2
		i = i + 1
	
	return math.sqrt(s)

def waitrobot(robot):
	"""busy wait for robot completion"""
	while not robot.GetController().IsDone():
		time.sleep(0.01)



def stringToFloatList(path):
	path = path.split('\n')
	for line in xrange(len(path)):
	  path[line] = path[line].split(',')
	  for i in xrange(len(path[line])):
		  path[line][i]=float(path[line][i])
	return path
	
def drawPath(path,robot,color,size):
	
	if type(path) is str: path = stringToFloatList(path)
	for i in path:
		robot.SetActiveDOFValues(i)
		handles.append(env.plot3(points=robot.GetTransform()[0:3,3],pointsize=size,colors=color,drawstyle=1))


if __name__ == "__main__":

	env = Environment()
	env.SetViewer('qtcoin')
	collisionChecker = RaveCreateCollisionChecker(env,'ode')
	env.SetCollisionChecker(collisionChecker)
	env.Reset()        
	# load a scene from ProjectRoom environment XML file
	env.Load('env/bitreequad.env.xml')
	time.sleep(0.1)

	# 1) get the 1st robot that is inside the loaded scene
	# 2) assign it to the variable named 'robot'
	robot = env.GetRobots()[0]

	robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z|DOFAffine.RotationQuat) 
	# print "DOF"
	# print robot.GetActiveDOFIndices();

	# startconfig = [ 4.0,-1.5 ,0.35 ,1.0, 0.0, 0.0, 0.0 ];
	startconfig = [ -2,-1.8 ,0.35 ,1.0, 0.0, 0.0, 0.0 ];
	robot.SetActiveDOFValues(startconfig);
	# robot.GetController().SetDesired(robot.GetDOFValues());
	# waitrobot(robot);
	handles = [];
	# raw_input("T_T")


	# raw_input("Press enter to continue to RRT...")
	with env:
		goalconfig = [-4.3, 0.8, 1 ,1.0 ,0.0 ,0.0 ,0.0];
		# goalconfig = [4,0.87,2.09,1.5,0,0];
		goalbias = 0.1;
		step = 0.3;

		# workspace boundary x,y,z
		workspaceBound = [-4.5,4.5,-2.2,2.2,0.21,1.54]

		# all data are stored in list except diction CTree
		# build RRT tree in cSpace
		# RRT tree initialize
		cTree = {tuple(startconfig):tuple(startconfig)} #\
		iterTime = 0        # search and build tree
		iterout = 0
		cTreeSize = 1
		newAddedNode = startconfig
		while iterTime < 100000:
			if iterout >= 1000:
				print iterTime
				iterout = iterout - 1000
			# print iterTime
			iterTime = iterTime + 1
			iterout = iterout + 1
			# test bias
			chance = random.uniform(0,1)

			if chance < goalbias: # try to connect the goal
				# print "try goal"
				newnode = goalconfig
				nnNode = us.nnNodeCQ(cTree,newnode)
				# nnNode = newAddedNode
				if us.distXQ(nnNode,newnode) <= step: # goal is within one step length!
					cTree[tuple(goalconfig)] = tuple(nnNode) 
					print "find path, it is just near!"
					break
				else: # goal is far than one step
					nnPath = us.stepNodesQ(nnNode,newnode,step)
					collision = False
					for pathNode in nnPath: # check the path to goal
						robot.SetActiveDOFValues(pathNode)
						if env.CheckCollision(robot) == True:
							collision = True
							break

					if collision == False: # the path is clear!
						cTree[tuple(nnPath[0])] = tuple(nnNode)
						for i in range(1,len(nnPath)):
							cTree[tuple(nnPath[i])] = tuple(nnPath[i-1])
						# cTree[tuple(goalconfig)] = tuple(nnPath[len(nnPath)-1])	
							# print nnPath[i-1]

						print "find path, by setp nodes!"	
						break
			else: # add new node to the RRT tree

				newnode = us.sampleCQ();
				nnNode = us.nnNodeCQ(cTree,newnode)
				steerNode = us.step1NodeQ(nnNode,newnode,step)
				robot.SetActiveDOFValues(steerNode)
				if env.CheckCollision(robot) == False:
					cTree[tuple(steerNode)] = tuple(nnNode)
					newAddedNode = steerNode
					cTreeSize += 1
					print "add new node  cTree size:",cTreeSize	

		robot.SetActiveDOFValues(startconfig) # return the robot the start            





		if tuple(goalconfig) not in cTree: # have not found the path to the goal
			raw_input("T_T")
		else:
			print "find goal in the outcome path!"
			path = us.getpath(cTree,goalconfig)
			# smooth path

			for i in range(200):
				print i
				n = len(path)
				A = randrange(0,n)
				B = randrange(0,n)
				while(abs(A-B)<=1):
					B = randrange(0,n)
				t = A
				A = min(A,B)
				B = max(B,t)
				a = path[A]
				b = path[B]
				smoothPath = us.stepNodesQ(a,b,step)
				flag = False
				# print len(smoothPath)
				for smoothNode in smoothPath:
					# print smoothNode
					robot.SetActiveDOFValues(smoothNode)
					if env.CheckCollision(robot) == True:
						flag = True
						break

				if flag == False:
					for k in range(A+1,B):
						del path[A+1]
						# print 'smoothing'
					k = A+1    
					for smoothNode in smoothPath:
						path.insert(k,smoothNode)
						k = k + 1
			color = array(((0,0,1)))

			path = us.discretePath(path, 0.1)
			for i in path:
				handles.append(env.plot3(points=dot(array([i[0],i[1],i[2]]),1),
										   pointsize=0.03,
										   colors=color,
										   drawstyle=1));
			
	robot.SetActiveDOFValues(startconfig)
	print "Path Length", len(path)
	raw_input("Press enter to move robot...")
	# Fit the path with planning

	#sample
	path_length = len(path)

	StateTree = []

	node = path[0]
	print node
	# node = us.E2Q(node)
	StateTree.append([node[0],node[1],node[2], 0.0, 0.0, 0.0, node[3],node[4],node[5],node[6], 0.0, 0.0, 0.0,])
	i = 1
	backtrackPoint = 0
	backtrackHeuristics = 0
	while (i < path_length):	
		print "Current Fitting ", i
		previousStateNode = StateTree[i-1]
		attemptConfig = path[i]
		reachedWaypoint = False;
		minL2 = 100;
		delta_t = 0.01
		for x in xrange(1,50):
			f = us.sampleF()
			t = delta_t
			newState = previousStateNode
			while t < 1:



				nearestIndex = us.nnNodeIndexPathSQ(newState,path,i)

				# f = us.sampleFS(newState)
				# f = us.sampleFSC(newState,attemptConfig)
				f = us.sampleFSC(newState,path[nearestIndex])
				newState = us.updateState(newState, f, delta_t)
				robot.SetActiveDOFValues(us.S2CQ(newState))
				# Check if within range
				

				# l2Distance = us.distCSQ(newState,attemptConfig)


				# nearestIndex = us.nnNodeIndexPathSQ(newState,path,i)
				print "nearestIndex",nearestIndex,"path_length",path_length
				l2Distance = us.distCSQ(newState,path[nearestIndex],0.05)
				time.sleep(0.001)

				print l2Distance
				if minL2 > l2Distance:
					minL2 = l2Distance
				if l2Distance < 0.3:
					# Check Collision
					robot.SetActiveDOFValues(us.S2CQ(newState))
					print "L2Distance with in 0.1"
					if env.CheckCollision(robot) == False:
						print "Success ! \n\n\n"
						#Set Flag
						reachedWaypoint = True
						#Add New StateNode
						StateTree.append(newState)
						#Check if backtrack is clear |-> clear heuristics
						if i == backtrackPoint:
							backtrackHeuristics = 0;
						#increment i
						# i = i + 1

						i = nearestIndex + 1
						break
				else:
					print "Failed, L2 distance is ", l2Distance, "Current Minimum", minL2, "Fitting Index", i
				t = t + delta_t
			if reachedWaypoint:
				break;

		if not reachedWaypoint:
			# Increment Heuristics
			backtrackHeuristics = backtrackHeuristics + 1;
			# Check waypoint
			if backtrackPoint < i :
				backtrackPoint = i
			# update i & Dequeque Statespace
			backtrackLimit = int(backtrackHeuristics/2) + 1;
			while (backtrackPoint - i < backtrackLimit):
				i = i - 1
				StateTree.pop();

		print "Iteration ", i, " Total Time ", t
		if reachedWaypoint:
			print "Successful"
		else:
			print "Fail"

	# robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z|DOFAffine.Rotation3D)
	traj = RaveCreateTrajectory(env,'');
	config = robot.GetActiveConfigurationSpecification('linear');

	config.AddDeltaTimeGroup();
	traj.Init(config);
	# mypath = [];
	newpath = [];

	offset = 0;
	for i,point in enumerate(path):
		newPoint = us.E2Q(point)

		if i == 0 :
			newpath.append([newPoint[0],newPoint[1],newPoint[2],newPoint[3],newPoint[4],newPoint[5], newPoint[6], i*0.005])
		else:
			lastPoint = us.E2Q(path[i-1])
			if (lastPoint[0] == point[0] and lastPoint[1] == point[1] and lastPoint[2] == point[2]):
				offset = offset + 1
				pass
			else:
				newpath.append([newPoint[0],newPoint[1],newPoint[2],newPoint[3],newPoint[4],newPoint[5], newPoint[6], (i-offset)*0.005]);
			
		
	
	for i, wayPoint in enumerate(newpath):

		traj.Insert(i, wayPoint,config,True);


	while 1:
		robot.GetController().SetPath(traj);

		waitrobot(robot)


	raw_input("Press enter to exit...")

