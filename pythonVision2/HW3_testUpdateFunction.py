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

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);        
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

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
    # print robot.GetActiveDOFValues()
    # raw_input("Press enter to move robot...")
    # qt = tf.quaternion_from_euler(0.5,0.5,0.75,'rzxz')
    # startconfig = [4.0,-1.5 ,0.2] + list(qt)
    # print startconfig

    startconfig = [ 4.0,-1.5 ,0.2 ,0.0, 0.0, 0.0 ];
    robot.SetActiveDOFValues(us.E2Q(startconfig));
    # robot.GetController().SetDesired(robot.GetDOFValues());
    # waitrobot(robot);




    waitrobot(robot)

    print "test update state"


    # s1 = [1,1,1,1,0,0,0,0.2,0.2,0.2,0.1,0.1,-0.1]
    avf = 1.85*9.8/4
    u = [-0.5*avf,2*avf,-0.5*avf,3*avf]
    ts = 0.02
    t = range(0,100)


    while 1:
        s2 = [0,0,0,0,0,0,1,0,0,0,0,0,0]
        for tt in t:
            s2 = us.updateState(s2,u,ts)
            x1 = array(s2[0:3])
            v1 = array(s2[3:6])
            Q1 = array(s2[6:10])
            W1 = array(s2[10:13])
            E1 = tf.euler_from_quaternion(Q1)

            C = list(x1)+list(Q1)

            robot.SetActiveDOFValues(C);
            time.sleep(0.02)







        # traj = RaveCreateTrajectory(env,'');
        # config = robot.GetActiveConfigurationSpecification('linear');

        # config.AddDeltaTimeGroup();
        # traj.Init(config);
        # # myPath = [ [point.x, point.y,point.theta,i*0.01] for i,point in enumerate(path) ];
        
        # num = 0
        # for pathNode in path:
        #     num += 1
        #     traj.Insert(num,pathNode,config,True)



        # # for i ,wayPoint in enumerate(myPath):
        # #     traj.Insert(i,wayPoint,config,True);
        # robot.GetController().SetPath(traj);

        # # robot.GetController().SetPath(traj)

    ### END OF YOUR CODE ###

    raw_input("Press enter to exit...")

