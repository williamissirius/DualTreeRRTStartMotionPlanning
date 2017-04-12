#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for EECS 598 Motion Planning
import time
import openravepy
import userdefined as us
import kdtree
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

    ### INITIALIZE YOUR PLUGIN HERE ###
    # RaveInitialize();
    # RaveLoadPlugin('build/myRRT')
    # myRRT = RaveCreateModule(env,'myRRT')
    ### END INITIALIZING YOUR PLUGIN ###
   

    # tuck in the PR2's arms for driving
    # tuckarms(env,robot);
  
    #set start config
    # robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z|DOFAffine.Rotation3D) 
    robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z|DOFAffine.Rotation3D) 
    print robot.GetActiveDOFValues()
    raw_input("Press enter to move robot...")

    p2 = 1.57
    startconfig = [ 4.0,-1.5 ,0.2 ,p2, p2/2, 0.0 ];
    robot.SetActiveDOFValues(startconfig);
    # robot.GetController().SetDesired(robot.GetDOFValues());
    # waitrobot(robot);
    handles = [];
    raw_input("Press enter to move robot...")
    with env:
        goalconfig = [-4.3, 0.8 ,0.5 ,0.0 ,0.0 ,0.0];
        # goalconfig = [4,0.87,2.09,1.5,0,0];
        ### YOUR CODE HERE ###
        ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig
        goalbias = 0.1;
        step = 0.2;

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
                        robot.SetActiveDOFValues(pathNode)
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
                robot.SetActiveDOFValues(steerNode)
                if env.CheckCollision(robot) == False:
                    KDtree.add(steerNode)
                    cTree[tuple(steerNode)] = tuple(nnNode)

        robot.SetActiveDOFValues(startconfig) # return the robot the start            
        if tuple(goalconfig) not in cTree: # have not found the path to the goal
            raw_input("T_T")
        else:
            print "find path!"
            path = us.getpath(cTree,goalconfig)
            # print path

            color = array(((0,0,1)))
            for i in path:
                handles.append(env.plot3(points=dot(array([i[0],i[1],i[2]]),1),
                                           pointsize=0.03,
                                           colors=color,
                                           drawstyle=1))
            raw_input("Press enter to move robot...")






            # # my path code
            # traj = RaveCreateTrajectory(env,'')
            # traj.Init(robot.GetActiveConfigurationSpecification())
            
            # num = 0
            # total = len(path)
            # for i in path:
            #     traj.Insert(num,array(i))
            #     num = num+1
            # planningutils.RetimeActiveDOFTrajectory(traj,robot,hastimestamps=False,maxvelmult=0.8,maxaccelmult=0.2,plannername='LinearTrajectoryRetimer')
            # # print 'duration',traj.GetDuration()

            # Zhang's code
            # traj = RaveCreateTrajectory(env,'');
            # config = robot.GetActiveConfigurationSpecification('linear');

            # config.AddDeltaTimeGroup();
            # traj.Init(config);
            # # myPath = [ [pose[0],pose[1],pose[2],pose[3],pose[4],pose[5],i*0.05] for i,pose in enumerate(path) ];
    
            # # for i ,wayPoint in enumerate(myPath):
            # #     traj.Insert(i,wayPoint,config,True);
            # num = 0
            # for i in path:
            #     traj.Insert(num*0.5,i,config,True)
            #     num = num+1


            # robot.GetController().SetPath(traj)
            # robot.WaitForController(0)

            # another code
        traj = RaveCreateTrajectory(env,'')
        traj.Init(robot.GetActiveConfigurationSpecification())
        traj.Insert(0,robot.GetActiveDOFValues())
        traj.Insert(1,goalconfig)
        planningutils.RetimeActiveDOFTrajectory(traj,robot,hastimestamps=False,maxvelmult=1,plannername='LinearTrajectoryRetimer')
    



       
        # RRTtime = time.time();
        # RRTtime = time.time() - RRTtime;

        # # drawPath(PathString,robot,[1,0,0],0.03);
        # print 'Time to find path       ',  RRTtime;
        # SmoothTime = time.time();
        # SmoothTime =time.time() - SmoothTime; 

        # drawPath(SmoothedPathString,robot,[0,0,0],0.03);
        # # print 'Time to smooth path     ',  SmoothTime,;
        # # print robot.GetActiveDOFWeights();
        # # print 'path length is ', len( stringToFloatList(PathString) );
        # # print 'smoothed path length is ', len(stringToFloatList(SmoothedPathString));
        # # raw_input("Press enter to draw traj!!!!");

        # # # robot.SetActiveDOFValues(startconfig)

        # path = stringToFloatList(SmoothedPathString);
        # raw_input("next?!!!!!!!")
        # # # path.reverse();
        # traj = RaveCreateTrajectory(env,'');
        # config = robot.GetActiveConfigurationSpecification('linear');

        # config.AddDeltaTimeGroup();
        # traj.Init(config);
        # myPath = [ [pose[0],pose[1],pose[2],pose[3],pose[4],pose[5],i*0.05] for i,pose in enumerate(path) ];
    
        # for i ,wayPoint in enumerate(myPath):
        #     traj.Insert(i,wayPoint,config,True);
      
        

        # robot.GetController().SetPath(traj)

    ### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")

