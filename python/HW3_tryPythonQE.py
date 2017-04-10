#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for EECS 598 Motion Planning
import time
import openravepy
import userdefined as us
import kdtree
import transformationFunction as tf 
from random import randrange
#### YOUR IMPORTS GO HERE ####
handles = [];
    
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

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
    print "DOF"
    print robot.GetActiveDOFIndices();

    startconfig = [ 4.0,-1.5 ,0.2 ,0.0, 0.0, 0.0 ];
    robot.SetActiveDOFValues(us.E2Q(startconfig));
    # robot.GetController().SetDesired(robot.GetDOFValues());
    # waitrobot(robot);
    handles = [];
    # raw_input("Press enter to move robot...")
    raw_input("Press enter to exit...")
    with env:
        goalconfig = [-4.3, 0.8, 1 ,0.0 ,0.0 ,0.0];
        # goalconfig = [4,0.87,2.09,1.5,0,0];
        ### YOUR CODE HERE ###
        ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig
        goalbias = 0.1;
        step = 0.3;

        # workspace boundary x,y,z
        workspaceBound = [-4.5,4.5,-2.2,2.2,0.21,1.54]

        # all data are stored in list except diction CTree
        # build RRT tree in cSpace
        # RRT tree initialize
        cTree = {tuple(startconfig):tuple(startconfig)} #\
        KDtree = kdtree.create([startconfig])
        iterTime = 0        # search and build tree
        iterout = 0
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
                newnode = goalconfig
                nnNode = KDtree.search_nn(newnode)[0].data
                if us.nodesDist(nnNode,newnode) <= step: # goal is within one step length!
                    cTree[tuple(goalconfig)] = tuple(nnNode) 
                    KDtree.add(goalconfig)
                    break
                else: # goal is far than one step
                    nnPath = us.stepNodes(nnNode,newnode,step)
                    collision = False
                    for pathNode in nnPath: # check the path to goal
                        robot.SetActiveDOFValues(us.E2Q(pathNode))
                        if env.CheckCollision(robot) == True:
                            collision = True
                            break

                    if collision == False: # the path is clear!
                        cTree[tuple(nnPath[0])] = tuple(nnNode)
                        KDtree.add(nnPath[0])
                        for i in range(1,len(nnPath)):
                            KDtree.add(nnPath[i])
                            cTree[tuple(nnPath[i])] = tuple(nnPath[i-1])
                        break
            else: # add new node to the RRT tree

                newnode = us.sampleCE();
                nnNode = KDtree.search_nn(newnode)[0].data
                steerNode = us.step1Node(nnNode,newnode,step)
                robot.SetActiveDOFValues(us.E2Q(steerNode))
                if env.CheckCollision(robot) == False:
                    KDtree.add(steerNode)
                    cTree[tuple(steerNode)] = tuple(nnNode)

        robot.SetActiveDOFValues(us.E2Q(startconfig)) # return the robot the start            





        if tuple(goalconfig) not in cTree: # have not found the path to the goal
            raw_input("T_T")
        else:
            print "find path!"
            path = us.getpath(cTree,goalconfig)
            # smooth path

            for i in range(20000):
                n = len(path)
                A = randrange(0,n)
                B = randrange(0,n)
                while(abs(A-B)<=2):
                    B = randrange(0,n)
                t = A
                A = min(A,B)
                B = max(B,t)
                #print "AB"
                #print (A,B)
                # A = B + 2
                a = path[A]
                b = path[B]
                smoothPath = us.stepNodes(a,b,step)

                flag = False
                #print "smooth path"
                #print smoothPath
                for smoothNode in smoothPath:
                    # print smoothNode
                    robot.SetActiveDOFValues(us.E2Q(smoothNode))
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

            for i in path:
                handles.append(env.plot3(points=dot(array([i[0],i[1],i[2]]),1),
                                           pointsize=0.03,
                                           colors=color,
                                           drawstyle=1));
            # raw_input("Press enter to move robot...")

    print "Path Length", len(path)
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
            
        
    
    for i ,wayPoint in enumerate(newpath):

        traj.Insert(i, wayPoint,config,True);


    while 1:
        robot.GetController().SetPath(traj);

        waitrobot(robot)


    raw_input("Press enter to exit...")

