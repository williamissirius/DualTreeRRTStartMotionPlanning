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
        handles.append(env.plot3(points=robot.GetTransform()[0:3,3],pointsize=size,colors=color,drawstyle=1))


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()        
    # load a scene from ProjectRoom environment XML file
    env.Load('bitreequad.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    ### INITIALIZE YOUR PLUGIN HERE ###
    RaveInitialize();
    RaveLoadPlugin('build/myRRT')
    myRRT = RaveCreateModule(env,'myRRT')
    ### END INITIALIZING YOUR PLUGIN ###
   

    # tuck in the PR2's arms for driving
    # tuckarms(env,robot);
  
    #set start config
    robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z|DOFAffine.Rotation3D)    
    startconfig = [ 4.0,-1.5 ,0.2 ,0.0, 0.0, 0.0 ];
    robot.SetActiveDOFValues(startconfig);
    # robot.GetController().SetDesired(robot.GetDOFValues());
    # waitrobot(robot);
    handles = [];
    raw_input("next?!!!!!!!");
    with env:
        goalconfig = [-4.3, 0.8 ,1 ,0.0 ,0.0 ,0.0];
        # goalconfig = [4,0.87,2.09,1.5,0,0];
        ### YOUR CODE HERE ###
        ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig
        goalbias = 0.4;
        step = 0.3;

        myRRT.SendCommand('SetStrGoal str %f,%f,%f,%f,%f,%f; goal %f,%f,%f,%f,%f,%f;'%tuple(startconfig+goalconfig) );
        myRRT.SendCommand('SetPara %f,%f'%tuple([goalbias,step]) );
       
        RRTtime = time.time();
        PathString = myRRT.SendCommand("FindPath");
        RRTtime = time.time() - RRTtime;

        # drawPath(PathString,robot,[1,0,0],0.03);
        print 'Time to find path       ',  RRTtime;
        SmoothTime = time.time();
        SmoothedPathString = myRRT.SendCommand("PathSmooth");
        SmoothTime =time.time() - SmoothTime; 

        drawPath(SmoothedPathString,robot,[0,0,0],0.03);
        # print 'Time to smooth path     ',  SmoothTime,;
        # print robot.GetActiveDOFWeights();
        # print 'path length is ', len( stringToFloatList(PathString) );
        # print 'smoothed path length is ', len(stringToFloatList(SmoothedPathString));
        # raw_input("Press enter to draw traj!!!!");

        # # robot.SetActiveDOFValues(startconfig)

        path = stringToFloatList(SmoothedPathString);
        raw_input("next?!!!!!!!")
        # # path.reverse();
        traj = RaveCreateTrajectory(env,'');
        config = robot.GetActiveConfigurationSpecification('linear');

        config.AddDeltaTimeGroup();
        traj.Init(config);
        myPath = [ [pose[0],pose[1],pose[2],pose[3],pose[4],pose[5],i*0.05] for i,pose in enumerate(path) ];
    
        for i ,wayPoint in enumerate(myPath):
            traj.Insert(i,wayPoint,config,True);
      
        

        robot.GetController().SetPath(traj)

    ### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")

