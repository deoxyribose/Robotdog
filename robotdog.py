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
from doggie import Doggie
    

p.connect(p.GUI)
p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0,0)

# dog = Dog()

# dog.set_weight_motor()
# dog.set_feet_friction()

doggie = Doggie()

doggie.set_weight_motor()
doggie.set_feet_friction()

doggie.set_init_pos_orient_pose([2, 2, 0.6])

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
    # dogPos, _ = p.getBasePositionAndOrientation(dog.dogId)
    dogPos, _ = p.getBasePositionAndOrientation(doggie.dogId)
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

    # #Keys to change the robot walk (fwd, bkw, rot right, rot left)
    # if keys.get(65297): #Up
    #     dog.drp=0
    # if keys.get(65298): #Down
    #     dog.drp=2
    #     dog.set_leg_sequence_for_next_cycle(lseq=[2,3,1,0])
    # if keys.get(65296): #Right
    #     dog.drp=1
    #     dog.xrcO=dog.xrO        #Set the center for the robot rotation to the current robot pos
    #     dog.set_leg_sequence_for_next_cycle(lseq=[1,0,2,3]) #Change the leg sequence to open up the front arms rather than close them
    # if keys.get(65295): #Left
    #     dog.drp=3
    #     dog.xrcO=dog.xrO
    #     dog.set_leg_sequence_for_next_cycle(lseq=[0,1,3,2]) #Change the leg sequence to open up the front arms rather than close them
	
    # #Time cycle
    # dog.walkLoopSpd = 400
    # dog.leg_cycle_length = dog.walkLoopSpd / 2
    # tv=int(((time.time()-t0) * dog.walkLoopSpd)  % (dog.walkLoopSpd * 2))

    # doggie.shift_body_weight2()
    
    # set color of link 1 to black
    p.changeVisualShape(doggie.dogId, 1, rgbaColor=[0,0,0,1]) # front left hip
    # p.changeVisualShape(doggie.dogId, 2, rgbaColor=[0,0,0,1]) # front left knee


    # doggie.forward_kinematics(0, 0, 0, 0)
    # doggie.forward_kinematics(0, 0.0991796640729571, 0.5628440549212753, -1.1256881098425509)
    doggie.forward_kinematics(0, 0.0991796640729571, 0.4, -1.1256881098425509)

    # dog.walk_loop(tv)

    # yawri=1.3
    # xrOi=np.array([1,1,0.5])
    # dog.Ryawri = dog.RotYawr(yawri)
    # legsRi=np.array([[dog.xhipf,dog.xhipf,dog.xhipb,dog.xhipb],
    #                 [dog.yhipl+0.1,-dog.yhipl-0.1,dog.yhipl+0.1,-dog.yhipl-0.1],
    #                 [-0.5,-0.5,-0.5,-0.5]])
    # print(legsRi)
    # dog.legsO=(np.dot(dog.Ryawri,legsRi).T + xrOi).T
    # print(dog.legsO)
    # legsR = np.dot(dog.Ryawri.T,dog.legsO)
    # print(legsR)
    # # x, y, z, v = [0,0,0],[0,0,0],[0,0,0],[0,0,0]
    # dog.setlegsxyz(legsR[0], legsR[1], legsR[2], [1, 1, 1, 1])
    time.sleep(0.01)


p.disconnect()
    






















