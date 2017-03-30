#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for EECS 598 Motion Planning
import time
import openravepy

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
        handles.append(env.plot3(points=robot.GetLinks()[49].GetTransform()[0:3,3],pointsize=size,colors=color,drawstyle=1))


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()        
    # load a scene from ProjectRoom environment XML file
    # env.Load('quadrotor.env.xml')
    env.Load('hw3.env.xml')
    time.sleep(0.1)
    robot = env.GetRobots()[0]
    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    # robot = env.GetRobots()[0]
    # robot2 = env.ReadRobotXMLFile('quadrotor_hokuyo_utm30lx.xml')
    # args = parse_args()
    # params = parser.Yaml(file_name=args.params)

    # env = rave.Environment()
    # env.SetViewer('qtcoin')
    # env.Load(params.scene)
    # env.UpdatePublishedBodies()
    # robot = env.GetRobots()[0]

        ### YOUR CODE HERE ###
        ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig


    ### END OF YOUR CODE ###


    raw_input("Press enter to exit...")

