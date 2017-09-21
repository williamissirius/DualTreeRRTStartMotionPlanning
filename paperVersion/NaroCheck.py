
import time
import openravepy
import userdefined as us
import kdtree
import transformationFunction as tf 
from random import randrange
#### YOUR IMPORTS GO HERE ####

def CheckNarrow(waypoint,stepsize):
    a = [stepsize,0,-stepsize];
    counter = 0;
    for dx in a:
        for dy in a:
            for dz in a:
                T = waypoint;
                T(0) = T(0)+dx;
                T(1) = T(1)+dx;
                T(2) = T(2)+dx;
                robot.SetActiveDOFValues(us.E2Q(T))
                if env.CheckCollision(robot)
                counter = counter +1;
    return counter;
            

def FindNarrowPt(path,stepsize):
    NarrowPt = [];
    for waypoint in path
        lvl = CheckNarrow(waypoint,stepsize);
        if lvl >= 20:
            NarrowPt.append(waypoint);
    return NarrowPt