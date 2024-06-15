# -*- coding: utf-8 -*-
"""
Original code
Created on Sun Jun 10 18:28:23 2018

@author: Richard Bloemenkamp

Refactored code
Created on Sat Jun 15 2024

@author: Frans Zdyb
"""
import pybullet as p
import time
import numpy as np
from dog import Dog
    

p.connect(p.GUI)
p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0,0)

dog = Dog()

dog.set_weight_motor()
dog.set_feet_friction()

#Add earth like gravity
p.setGravity(0,0,-9.81)
p.setRealTimeSimulation(1)
#Point the camera at the robot at the desired angle and distance
p.resetDebugVisualizerCamera( cameraDistance=1.5, cameraYaw=-30, cameraPitch=-30, cameraTargetPosition=[0.0, 0.0, 0.25])


#Scenery e.g. an inclined box
boxHalfLength = 2.5
boxHalfWidth = 2.5
boxHalfHeight = 0.2
sh_colBox = p.createCollisionShape(p.GEOM_BOX,halfExtents=[boxHalfLength,boxHalfWidth,boxHalfHeight])
mass = 1
block=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [-2,0,-0.1],baseOrientation=[0.0,0.1,0.0,1])



t0=time.time()

cyaw=10
cpitch=-15
cdist=1.5

while True:
    dogPos, _ = p.getBasePositionAndOrientation(dog.dogId)
    #Camera paramers to be able to yaw pitch and zoom the camera (Focus remains on the robot) 

    p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=dogPos)

    keys = p.getKeyboardEvents()
    #Keys to change camera
    if keys.get(100):  #D
        cyaw+=1
    if keys.get(97):   #A
        cyaw-=1
    if keys.get(99):   #C
        cpitch+=1
    if keys.get(102):  #F
        cpitch-=1
    if keys.get(122):  #Z
        cdist+=.01
    if keys.get(120):  #X
        cdist-=.01

    #Keys to change the robot walk (fwd, bkw, rot right, rot left)
    if keys.get(65297): #Up
        dog.drp=0
    if keys.get(65298): #Down
        dog.drp=2
    if keys.get(65296): #Right
        dog.drp=1
        dog.xrcO=dog.xrO        #Set the center for the robot rotation to the current robot pos
        dog.set_leg_sequence_for_next_cycle(lseq=[1,0,2,3]) #Change the leg sequence to open up the front arms rather than close them
    if keys.get(65295): #Left
        dog.drp=3
        dog.xrcO=dog.xrO
        dog.set_leg_sequence_for_next_cycle(lseq=[0,1,3,2]) #Change the leg sequence to open up the front arms rather than close them
	
    #Time cycle
    dog.walkLoopSpd = 400
    dog.leg_cycle_length = dog.walkLoopSpd / 2
    tv=int(((time.time()-t0) * dog.walkLoopSpd)  % (dog.walkLoopSpd * 2))
    dog.walk_loop(tv)
    time.sleep(0.01)


p.disconnect()
    






















