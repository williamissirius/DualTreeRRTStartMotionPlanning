#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for EECS 598 Motion Planning
import time
import random
import openravepy
import userdefinedTest as us
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
def pathNearNarrowPoint(NarrPt):
    # give a narrow point
    # return a feasible path through it 
    for speedi in linspace(3, 18, num = 10): # sample speed at the narrow point
        statei = NarrPt[0:3]+[-speedi,0,0] + NarrPt[3:7]+[0,0,0]
        for i in range(0,1): # sample trajectory at the state with specific speed 
            iterTotal = 3
            delta_t = 0.01
            statet = statei    
            tbackward = -3
            tforward = 3


            fFind = 0
            bFind = 0
            cf = []
            cb = []
            t1 = 0
            t2 = -delta_t
            #######################
            ### forward ########### 
            #######################
            statet = statei
            while t1<tforward:
                f = us.sampleFSstill(statet,0.1)
                statet = us.updateState(statet,f,delta_t)
                robot.SetActiveDOFValues(statet)

                time.sleep(delta_t)
                
                cf.append(us.S2CQ(statet))
                zAngle = us.Q2E(statet[6:10])[1]
                speedTotal = abs(statet[3] + statet[4] + statet[5])

                if env.CheckCollision(robot) == True:
                    break

                print "zAngle " + str(zAngle) + " speedTotal " + str(speedTotal)
                if abs((zAngle ))<0.2 and speedTotal < 1:
                    fFind = 1
                    break
                    print "find stable transition point"
                t1 = t1 + delta_t

            #######################
            #### backrward ########
            #######################
            fFind = 1
            if fFind == 1:
                statet = statei
                while t2<tforward:
                    f = us.sampleFSstillBack(statet,0.1)
                    statet = us.updateState(statet,f,-delta_t)
                    robot.SetActiveDOFValues(statet)
                    time.sleep(delta_t)

                    cb.append(us.S2CQ(statet))
                    zAngle = us.Q2E(statet[6:10])[1]
                    speedTotal = abs(statet[3] + statet[4] + statet[5])

                    if env.CheckCollision(robot) == True:
                        break 

                    print "zAngle " + str(zAngle) + " speedTotal " + str(speedTotal)
                    if abs((zAngle ))<0.4 and speedTotal < 2:
                        bFind = 1
                        break
                        print "find stable transition point"
                    t2 = t2 - delta_t
            #################################
            #### Animate the trajectory #####
            #################################
            # if bFind == 1 and fFind == 1:
        ctotal = list(reversed(cb))+cf

        for ct in ctotal:
            robot.SetActiveDOFValues(ct)
            time.sleep(delta_t)
            

        if bFind == 1 and fFind == 1:
            break
        raw_input("can not find a path at current speed, press enter to try a higher speed ")







    return ctotal


def RRT(startconfig,goalconfig,goalbias = 0.05, step = 0.5, workspaceBound = [-4.5,4.5,-2.2,2.2,0.21,1.54]):
    # return a unsmoothed path from start to goal
    # take congiguration as input [translation,quaternion]
    # [x,q]
    # all data are stored in list except diction CTree
    # build RRT tree in cSpace
    # RRT tree initialize
    cTree = {tuple(startconfig):tuple(startconfig)} #\
    iterTime = 0        # search and build tree
    iterout = 0
    cTreeSize = 1
    newAddedNode = startconfig
    while iterTime < 100000:
        # time.sleep(0.4)
        if iterTime % 1000 ==0:
            print iterTime
        iterTime = iterTime + 1
        # test bias

        chance = random.uniform(0,1)

        if chance < goalbias: # try to connect the goal
            # print "try goal"
            newnode = goalconfig
            # nnNode = us.nnNodeCQ(cTree,newnode)
            nnNode = newAddedNode
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
            # robot.SetActiveDOFValues(newnode)
            # time.sleep(0.2)
            nnNode = us.nnNodeCQ(cTree,newnode)
            steerNode = us.step1NodeQ(nnNode,newnode,step)

            robot.SetActiveDOFValues(steerNode)
            # time.sleep(0.4)
            if env.CheckCollision(robot) == False:
                cTree[tuple(steerNode)] = tuple(nnNode)
                newAddedNode = steerNode
                cTreeSize += 1
                print "add new node  cTree size:",cTreeSize 

    robot.SetActiveDOFValues(startconfig) # return the robot the start            


    ###################
    # Smooth the path #
    ###################

    if tuple(goalconfig) not in cTree: # have not found the path to the goal
        raw_input("T_T")
    else:
        print "find goal in the outcome path!"
        path = us.getpath(cTree,goalconfig)
    return path

def MaxmarginNarrowpointFromNarrowPath(NarrPath):
    # give a small and continuous narrow path
    # return a Maxmargin narrow point
    NarrPt = NarrPath[int(len(NarrPath)/2)]
    ydis = np.linspace(-0.2, 0.2, num = 20)
    zdis = np.linspace(-0.2, 0.2, num = 20)
    sumValidNarrpts = array([0,0,0,0,0,0,0]) # valid points around narrow point
    numValidNarrpts = 0
    for zi in zdis:
        for yi in ydis:
            incYZ = array([0,yi,zi,0,0,0,0])
            newNode = array(NarrPt) + incYZ
            robot.SetActiveDOFValues(list(newNode))
            if env.CheckCollision(robot) == False:
                numValidNarrpts = 1 + numValidNarrpts
                sumValidNarrpts = incYZ + sumValidNarrpts
    NarrPt = list(sumValidNarrpts/numValidNarrpts + array(NarrPt))


    return NarrPt





def smoothPath(path):
    # smooth the path
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
    return path

def CheckNarrow(waypoint,stepsize):
    a = [stepsize,0,-stepsize];
    counter = 0;
    for dz in a:
        # for dy in a:
        #   for dz in a:
                T = waypoint;
                T[2] = T[2]+dz;
                robot.SetActiveDOFValues(us.E2Q(T))
                if env.CheckCollision(robot):
                    counter = counter +1;
    return counter;
            

def FindNarrowPt(path,stepsize,threshold):
    NarrowPt = []
    NarrowIndex = []
    for i in range(0,len(path)):
        waypoint = path[i]
        lvl = CheckNarrow(waypoint,stepsize);
        if lvl >= threshold:
            NarrowPt.append(waypoint);
            NarrowIndex.append(i)
    return NarrowPt,NarrowIndex





def plotPath(path,handles,Color = array(((0,0,1))),clear = 0):
    if clear == 1:
        handles = []
    for i in path:
        print i
        handles.append(env.plot3(points=dot(array([i[0],i[1],i[2]]),1),
                                   pointsize=0.03,
                                   colors=Color,
                                   drawstyle=1));




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
    env.Load('env/bitreequadmulitirobotsTwoWins.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]
    robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z|DOFAffine.RotationQuat) 
    
    robotAni = [] # a list of robot for animation
    for i in range(1,21):
        robotAni.append(env.GetRobots()[i])
        robotAni[i-1].SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z|DOFAffine.RotationQuat) 
    # print "DOF"
    # print robot.GetActiveDOFIndices();

    # startconfig = [ 4.0,-1.5 ,0.35 ,1.0, 0.0, 0.0, 0.0 ];
    goalconfig = [-4.3, 0.8, 1 ,1.0 ,0.0 ,0.0 ,0.0];
    startconfig = [ 4.0,-1.5 ,0.9 ,1.0, 0.0, 0.0, 0.0 ];
    # startconfig = [ -2,-1.8 ,0.35 ,1.0, 0.0, 0.0, 0.0 ];
    robot.SetActiveDOFValues(startconfig);
    # robot.GetController().SetDesired(robot.GetDOFValues());
    # waitrobot(robot);
    handles = [];
    #####################################################################
    ############################## within env ###########################
    #####################################################################    
    with env:
        # goalconfig = [4,0.87,2.09,1.5,0,0];
        goalbias = 0.1;
        step = 0.3;
        # workspace boundary x,y,z
        workspaceBound = [-4.5,4.5,-2.2,2.2,0.21,1.54]

        ##########################################
        # Use RRT to find a holonomic path first #
        ##########################################
        path = RRT(startconfig,goalconfig,goalbias,step,workspaceBound)
        ##################################
        # smooth and discretize the path #
        ##################################
        # smooth path
        path = smoothPath(path)
        path = us.discretePath(path, 0.2)
        Color = array(((0,0,1)))
        plotPath(path,handles)
        # raw_input("clear the draw")
        # handles = [];
        ################################################################################################
        # adjust the narrow point; replace the narrow point in the path with the new generated point #
        ################################################################################################

        NarrPath,NarrowIndex = FindNarrowPt(path,0.3,1);

        NarrPtIndex = int(len(NarrPath)/2) # Find the middel of the 
        print "narrow points"
        print NarrPath 
        handles = []
        NarrPathPos = []
        NarrPathNeg = []
        for NarrNodei in NarrPath:
            if NarrNodei[0] > 0:
                NarrPathPos.append(NarrNodei)
                Color = array(((1,1,1)))
            if NarrNodei[0] < 0:
                NarrPathNeg.append(NarrNodei)
                Color = array(((0,0,0)))
            handles.append(env.plot3(points=dot(array([NarrNodei[0],NarrNodei[1],NarrNodei[2]]),1),
                               pointsize=0.04,
                               colors=Color,
                               drawstyle=1));
        for ii in NarrPathNeg:
            print ii
        for ii in NarrPathPos:
            print ii     
        NarrPtNeg = MaxmarginNarrowpointFromNarrowPath(NarrPathNeg)
        handles.append(env.plot3(points=dot(array([NarrPtNeg[0],NarrPtNeg[1],NarrPtNeg[2]]),1),
                           pointsize=0.08,
                           colors=Color,
                           drawstyle=1));

        NarrPtPos = MaxmarginNarrowpointFromNarrowPath(NarrPathPos)
        handles.append(env.plot3(points=dot(array([NarrPtPos[0],NarrPtPos[1],NarrPtPos[2]]),1),
                           pointsize=0.08,
                           colors=Color,
                           drawstyle=1));
        print "the maxmargin points"
        print NarrPtNeg
        print NarrPtPos


    NarrPathNeg = pathNearNarrowPoint(NarrPtNeg)
    NarrPathPos = pathNearNarrowPoint(NarrPtPos)




    
    Npath = [NarrPathPos,NarrPathNeg]

    raw_input("enter_to draw path")
    ######################################
    # suppose have n narrow passage      #
    # Npath = [[path1],[path2],,,[pathn]]#
    ######################################
    PathFinal = []
    stepFinal = 0.02
    beginNode = startconfig 
    for i in range(len(Npath)):
        Npahti = Npath[i]
        NpathiBegin = Npahti[0]
        NpahtiEnd = Npahti[-1]

        PathFinal = PathFinal + us.stepNodesQ(beginNode,NpathiBegin,stepFinal) + Npahti
        beginNode = NpahtiEnd
    PathFinal = PathFinal + us.stepNodesQ(beginNode,goalconfig,stepFinal)

    print PathFinal
    print type(PathFinal)
    handles = []
    #plotPath(PathFinal,handles)

    #raw_input("enter_to draw path by quadcopters")





    # numTotal = len(PathFinal)
    # for num in range(0,20):
    #     numNode = PathFinal[int(np.ceil(numTotal*num/20))]
    #     robotAni[num].SetActiveDOFValues(numNode)

    raw_input("enter_to do the animation")
    for num in range(0,20):
        robotAni[num].SetActiveDOFValues(startconfig)


    robot.SetActiveDOFValues(startconfig)
    traj = RaveCreateTrajectory(env,'');
    config = robot.GetActiveConfigurationSpecification('linear');
    config.AddDeltaTimeGroup();
    traj.Init(config);


   
    timer = 0;    
    
    for i, wayPoint in enumerate(PathFinal):
 		# if i == 0:
 		# 	timer = 0;
 		# else:
   #  		distance = (wayPoint[0]**2 + wayPoint[1]**2 + wayPoint[2]**2)**0.5; 
   #  		speed = (speed_array[0]**2 + speed_array[1]**2 + speed_array[2]**2)**0.5;
   #  		timer = timer + distance/speed;
        traj.Insert(i, wayPoint+[0.0001*i],config,True);


    while 1:
        robot.GetController().SetPath(traj);
        waitrobot(robot)








    raw_input("Press enter to exit...")

