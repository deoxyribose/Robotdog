from typing import Any
import pybullet as p
import numpy as np
from actuator_commands import xyztoang

class Doggie():
    def __init__(self):
        #Dog robot part shapes
        sh_body = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.45, 0.08, 0.02])
        sh_extraweight = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.45, 0.08, 0.025])
        sh_roll = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.02, 0.02, 0.02])
        sh_hip = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.02, 0.02, 0.02])
        sh_knee = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.02, 0.02, 0.02])
        sh_foot = p.createCollisionShape(p.GEOM_SPHERE,radius=0.04)

        #The Dog robot is the body with other shapes linked to it
        body_Mass = 1
        visualShapeId = -1
        link_Masses=[.1, .1, .1, .1,
                    .1, .1, .1, .1,
                    .1, .1, .1, .1,
                    .1, .1, .1, .1,
                    20]
        linkCollisionShapeIndices=[sh_roll, sh_hip, sh_knee, sh_foot,
                                sh_roll, sh_hip, sh_knee, sh_foot,
                                sh_roll, sh_hip, sh_knee, sh_foot,
                                sh_roll, sh_hip, sh_knee, sh_foot,
                                sh_extraweight]
        nlnk=len(link_Masses)
        linkVisualShapeIndices=[-1]*nlnk    #=[-1,-1,-1, ... , -1]
        #link positions wrt the link they are attached to
        self.xhipf=0.4
        self.xhipb=-0.4
        self.pelvis_width=0.2

        self.hip_offset_along_body=0.02
        self.hip_width=0.05
        self.thigh_length=0.3
        self.calf_length=0.3
        self.linkPositions=[[self.xhipf, self.pelvis_width, 0], [self.hip_offset_along_body, self.hip_width, 0], [0, 0, -self.thigh_length], [0, 0, -self.calf_length],
                    [self.xhipf, -self.pelvis_width, 0], [self.hip_offset_along_body, -self.hip_width, 0], [0, 0, -self.thigh_length], [0, 0, -self.calf_length],
                    [self.xhipb, self.pelvis_width, 0], [self.hip_offset_along_body, self.hip_width, 0], [0, 0, -self.thigh_length], [0, 0, -self.calf_length],
                    [self.xhipb, -self.pelvis_width, 0], [self.hip_offset_along_body, -self.hip_width, 0], [0, 0, -self.thigh_length], [0, 0, -self.calf_length],
                    [0,0,+0.029]]
        linkOrientations=[[0,0,0,1]]*nlnk
        linkInertialFramePositions=[[0,0,0]]*nlnk
        #Note the orientations are given in quaternions (4 params). There are function to convert of Euler angles and back
        linkInertialFrameOrientations=[[0,0,0,1]]*nlnk
        #indices determine for each link which other link it is attached to
        # for example 3rd index = 2 means that the front left knee joint is attached to the front left hip
        self.indices=[0, 1, 2, 3,
                0, 5, 6, 7,
                0, 9,10,11,
                0,13,14,15,
                0]
        #Most joint are revolving. The prismatic joints are kept fixed for now
        jointTypes=[p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_PRISMATIC,
                    p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_PRISMATIC,
                    p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_PRISMATIC,
                    p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_PRISMATIC,
                    p.JOINT_PRISMATIC]
        #revolution axis for each revolving joint
        axis=[[1,0,0], [0,1,0], [0,1,0], [0,0,1],
            [1,0,0], [0,1,0], [0,1,0], [0,0,1],
            [1,0,0], [0,1,0], [0,1,0], [0,0,1],
            [1,0,0], [0,1,0], [0,1,0], [0,0,1],
            [0,0,1]]

        #Drop the body in the scene at the following body coordinates
        basePosition = [0,0,1]
        baseOrientation = [0,0,0,1]
        #Main function that creates the dog
        self.dogId = p.createMultiBody(body_Mass,sh_body,visualShapeId,basePosition,baseOrientation,
                                linkMasses=link_Masses,
                                linkCollisionShapeIndices=linkCollisionShapeIndices,
                                linkVisualShapeIndices=linkVisualShapeIndices,
                                linkPositions=self.linkPositions,
                                linkOrientations=linkOrientations,
                                linkInertialFramePositions=linkInertialFramePositions,
                                linkInertialFrameOrientations=linkInertialFrameOrientations,
                                linkParentIndices=self.indices,
                                linkJointTypes=jointTypes,
                                linkJointAxis=axis)
        
        #Walking speed (changes the walking loop time)
        self.walkLoopSpd=400
        self.leg_cycle_length = self.walkLoopSpd / 2

    #Rotation matrix for yaw only between robot-frame and world-frame
    def RotYawr(self, yawr):
        Rhor=np.array([[np.cos(yawr),-np.sin(yawr),0], [np.sin(yawr),np.cos(yawr),0], [0,0,1]])
        return Rhor

    def set_init_pos_orient_pose(self, init_pos = None):
        #Init robot position, orientation and pose params
        # O means in world-centered coordinates
        # R means in robot-centered coordinates
        # r is for "of the robot"
        # i is initial

        yawri=1.3
        if init_pos is None:
            xrOi=np.array([1,1,0.5])
        else:
            xrOi = np.array(init_pos)

        legsRi=np.array([[self.xhipf,self.xhipf,self.xhipb,self.xhipb],
                    [self.pelvis_width+0.1,-self.pelvis_width-0.1,self.pelvis_width+0.1,-self.pelvis_width-0.1],
                    [-0.5,-0.5,-0.5,-0.5]])
        #Set body to the robot pos
        xbOi=xrOi
        
        #Init body position and orientation
        quat=p.getQuaternionFromEuler([0,0,yawri])
        p.resetBasePositionAndOrientation(self.dogId, xbOi, quat)
        #Init leg abs pos
        self.Ryawri = self.RotYawr(yawri)
        self.legsO=(np.dot(self.Ryawri,legsRi).T + xbOi).T   #Apply rotation plus translation

        #Set the non-initial variables and matrix
        self.yawr=yawri
        self.xrO=xrOi
        self.xbO=self.xrO
        self.Ryawr = self.RotYawr(yawri)

        #Recalc leg rel pos in robot frame and set the legs
        self.dlegsO = self.leg_pos_relative_to_reference(reference = self.xbO)
        dlegsR = self.leg_pos_from_world_to_robot_frame()

        self.setlegsxyz(dlegsR[0], dlegsR[1], dlegsR[2], [1,1,1,1], verbose=True)

        #Calculate a new robot center position from the average of the feet positions
        #Calculate a new robot yaw ditrection also from the feet positions
        self.xfO = self.legsO[:,:2].mean(1) # mean of front legs
        self.xbO = self.legsO[:,2:].mean(1) # mean of back legs

        xfmbO = self.xfO - self.xbO
        self.yawrn=np.arctan2(xfmbO[1],xfmbO[0])


        #Change general motor speed
        self.vvec=[12]*4

        #Current leg to change position
        l=0
        #Init the center for the robot rotation to the current robot pos
        self.xrcO = self.xrO
        #Set the body position to the robot position
        self.xoff = 0
        self.yoff = 0
        #Init to walking fwd
        self.dr = 0
        self.drp = 0
        #Leg sequence (for rotating the robot, I chose to chg legs in the order front-left, fr, br, bl)
        self.set_leg_sequence()

    def set_leg_sequence(self, lseq = [0,1,3,2]):
        self.lseqp = lseq
        self.lseq = lseq

    def set_leg_sequence_for_next_cycle(self, lseq = [0,1,3,2]):
        self.lseqp = lseq

    def set_weight_motor(self):
        #Due to the weight the prismatic extraweight block needs to be motored up
        joint=16
        p.setJointMotorControl2(self.dogId,joint,p.POSITION_CONTROL,targetPosition=0.01,force=1000,maxVelocity=3)
        #Same for the prismatic feet spheres
        joint=3
        p.setJointMotorControl2(self.dogId,joint,p.POSITION_CONTROL,targetPosition=0.0,force=1000,maxVelocity=3)
        joint=7
        p.setJointMotorControl2(self.dogId,joint,p.POSITION_CONTROL,targetPosition=0.0,force=1000,maxVelocity=3)
        joint=11
        p.setJointMotorControl2(self.dogId,joint,p.POSITION_CONTROL,targetPosition=0.0,force=1000,maxVelocity=3)
        joint=15
        p.setJointMotorControl2(self.dogId,joint,p.POSITION_CONTROL,targetPosition=0.0,force=1000,maxVelocity=3)

    def set_feet_friction(self):
        #Add extra lateral friction to the feet spheres
        p.changeDynamics(self.dogId,3,lateralFriction=2)
        p.changeDynamics(self.dogId,7,lateralFriction=2)
        p.changeDynamics(self.dogId,11,lateralFriction=2)
        p.changeDynamics(self.dogId,15,lateralFriction=2)

    def setJointAnglesLeg(self, leg, angles, speed):
        """
        Set the joint angles of the leg
        """
        for i, angle in enumerate(angles):
            joint = leg*4 + i
            p.setJointMotorControl2(self.dogId,joint,p.POSITION_CONTROL,targetPosition=angle,force=1000,maxVelocity=speed[i])
    
    def setlegsxyz(self, xvec,yvec,zvec,vvec, verbose = False):
        spd=1
        for leg in range(4):
            if leg == 0:
                args = xvec[leg]-self.xhipf, yvec[leg] - self.pelvis_width, zvec[leg], self.hip_width, self.thigh_length, self.calf_length
            elif leg == 1:
                args = xvec[leg]-self.xhipf, yvec[leg] + self.pelvis_width, zvec[leg], -self.hip_width, self.thigh_length, self.calf_length
            elif leg == 2:
                args = xvec[leg]-self.xhipb, yvec[leg] - self.pelvis_width, zvec[leg], self.hip_width, self.thigh_length, self.calf_length
            elif leg == 3:
                args = xvec[leg]-self.xhipb, yvec[leg] + self.pelvis_width, zvec[leg], -self.hip_width, self.thigh_length, self.calf_length
            a = xyztoang(*args)
            if verbose:
                print(f"Leg {leg}: {a}")
            
            self.setJointAnglesLeg(leg, a, [spd, vvec[leg], vvec[leg]])

    def leg_pos_relative_to_reference(self, reference = None):
        if reference is None:
            reference = self.xrO
        self.dlegsO = self.legsO - reference[None].T
        return self.dlegsO

    def leg_pos_from_world_to_robot_frame(self):
        dlegsR=np.dot(self.Ryawr.T,self.dlegsO)
        return dlegsR
                             
    def walk_loop(self, tv):
        # 800 units is one full walk cycle
        # with four legs, each leg moves in 200 units
        
        # Only change direction if we're at the start of a new cycle
        if tv < self.leg_cycle_length * 0.1 and (not self.dr==self.drp):
            self.dr = self.drp
            self.set_leg_sequence(self.lseqp)

        # select the leg to move
        l=int(tv / self.leg_cycle_length)
        
        #Actual leg to move
        leg_idx = self.lseq[l]

        self.shift_body_weight(leg_idx, tv % self.leg_cycle_length)
        
        #Calculate the change in leg position in the robot frame
        self.dlegsO = self.leg_pos_relative_to_reference()
        dlegsR = self.leg_pos_from_world_to_robot_frame()
        
        #Then apply the body movement and set the legs
        self.setlegsxyz(dlegsR[0]-self.xoff-0.03,
                        dlegsR[1]-self.yoff,
                        dlegsR[2],
                        self.vvec)  #0.03 is for tweaking the center of grav.
        
        # Calculate the new position of the leg to move
        self.get_leg_pos_delta(leg_idx, tv % self.leg_cycle_length)

        #Calculate vectors and matrix for the next loop
        xfrO = self.legsO[:,:2].mean(1) # mean of front legs
        xbkO = self.legsO[:,2:].mean(1) # mean of back legs
        self.xrO = self.legsO.mean(1)
        self.xrO[2]=0.5
        xfmbO=xfrO-xbkO
        self.yawr=np.arctan2(xfmbO[1],xfmbO[0])
        self.Ryawr=self.RotYawr(self.yawr)

    def get_leg_pos_delta(self, leg_idx, leg_t):
        """
        Sets a new legs0 and yawl0
        """
        # self.leg_cycle_length units is one full leg cycle
        if int(leg_t) <= self.leg_cycle_length * 0.4:
            # keep all legs down to the ground in the beginning of the leg cycle
            self.legsO[2, :] = 0.0
        else:
            self.dlegsO = self.leg_pos_relative_to_reference(reference = self.xrcO)
            self.yawlO = np.arctan2(self.dlegsO[1, leg_idx], self.dlegsO[0, leg_idx])
            self.rlO = np.sqrt(self.dlegsO[0, leg_idx]**2 + self.dlegsO[1, leg_idx]**2)

            if self.leg_cycle_length * 0.4 < int(leg_t) <= self.leg_cycle_length * 0.75:
                # move selected leg upwards
                self.legsO[2, leg_idx] += 0.006
            else:
                # move selected leg downwards
                self.legsO[2, leg_idx] -= 0.006
            
            if self.dr == 0:
                self.legsO[0, leg_idx] = self.rlO*np.cos(self.yawlO)+self.xrcO[0]+0.01*np.cos(self.yawr)
                self.legsO[1, leg_idx] = self.rlO*np.sin(self.yawlO)+self.xrcO[1]+0.01*np.sin(self.yawr)
            elif self.dr == 1:
                self.yawlO -= 0.015 
                self.legsO[0, leg_idx] = self.rlO*np.cos(self.yawlO)+self.xrcO[0]
                self.legsO[1, leg_idx] = self.rlO*np.sin(self.yawlO)+self.xrcO[1]
            elif self.dr == 2:
                self.legsO[0, leg_idx] = self.rlO*np.cos(self.yawlO)+self.xrcO[0]-0.01*np.cos(self.yawr)
                self.legsO[1, leg_idx] = self.rlO*np.sin(self.yawlO)+self.xrcO[1]-0.01*np.sin(self.yawr)
            elif self.dr == 3:
                self.yawlO += 0.015 
                self.legsO[0, leg_idx] = self.rlO*np.cos(self.yawlO)+self.xrcO[0]
                self.legsO[1, leg_idx] = self.rlO*np.sin(self.yawlO)+self.xrcO[1]


    def shift_body_weight(self, leg_idx, leg_t):
        #In the beginning of the leg cycle the body is centered at the robot center
        #then it gradually moves in the opposite direction of the leg to be moved 
        #to ensure the center of gravity remains on the other 3 legs
        #when the moving leg goes down again the body center returns to the robot center
        #The vars xoff and yoff move the body w.r.t. the robot center in the robot frame
        
        if self.leg_cycle_length * 0.05 < int(leg_t) < self.leg_cycle_length * 0.4:
            self.xoff += 0.002*(-1+2*int(leg_idx/2))  #Work it out on paper to see it moves opposite to the stepping leg
            self.yoff += 0.002*(-1+2*(leg_idx%2))     

        elif int(leg_t) > self.leg_cycle_length * 0.8:
            self.xoff -= 0.004*(-1+2*int(leg_idx/2))
            self.yoff -= 0.004*(-1+2*(leg_idx%2))


    def forward_kinematics(self, leg_idx, gamma, alpha, beta, t):
        """
        Calculate the forward kinematics for the selected leg, i.e.
        given the leg index and the angles of the joints
        return the [x, y, z] coordinates of the foot wrt the roll joint
        """
        # get current body center position
        current_center_of_mass_pos = p.getBasePositionAndOrientation(self.dogId)[0]
        current_center_of_mass_pos = np.array(current_center_of_mass_pos)

        # get current body center orientation
        current_center_of_mass_orientation = p.getBasePositionAndOrientation(self.dogId)[1]
        current_center_of_mass_orientation = np.array(current_center_of_mass_orientation)
        rotmat = np.array(p.getMatrixFromQuaternion(current_center_of_mass_orientation)).reshape(3,3)
        # # rotmat rotates from base frame to world frame
        
        roll_joint_pos = self.linkPositions[leg_idx*4]
        roll_joint_pos = np.array(roll_joint_pos)

        roll_joint_world_coords = current_center_of_mass_pos + rotmat @ roll_joint_pos
        assert t > 3 or np.allclose(roll_joint_world_coords, p.getLinkState(self.dogId, 0)[0], 0.03)

        # calculate the position of the hip joint
        # we know the length of the hip is hip_width
        # the angle of the roll joint is gamma
        # the roll joint rotates around the axis 
        # that is parallel to the line between the hip and the body
        # so the x and y coordinates of the hip joint are
        hip_joint_pos_delta = [self.hip_offset_along_body, self.hip_width * np.cos(gamma), self.hip_width * np.sin(gamma)]
        hip_joint_pos_delta = np.array(hip_joint_pos_delta)
        hip_joint_world_coords = rotmat @ hip_joint_pos_delta + roll_joint_world_coords
        assert t > 3 or np.allclose(hip_joint_world_coords, p.getLinkState(self.dogId, 1)[0], 0.03)

        # calculate the position of the knee joint
        # we know the length of the thigh is thigh_length
        # the angle of the hip joint is alpha
        knee_joint_pos_delta = [-self.thigh_length * np.sin(alpha), 0, -self.thigh_length * np.cos(alpha)]
        knee_joint_pos_delta = np.array(knee_joint_pos_delta)
        # knee_joint_pos_delta is defined wrt. the hip joint
        # so first need to rotate it to the hip frame
        # then rotate it to the world frame
        # print(rotmat)
        rotmat2 = np.array(p.getMatrixFromQuaternion(p.getLinkState(self.dogId, 0)[1])).reshape(3,3)
        # rotates from hip frame to world frame
        # hip frame's z axis is parallel to thigh
        np.set_printoptions(precision=4)
        knee_joint_world_coords = rotmat2 @ knee_joint_pos_delta + hip_joint_world_coords
        assert t < 0.3 or np.allclose(knee_joint_world_coords, p.getLinkState(self.dogId, 2)[0], 0.03)

        rotmat3 = np.array(p.getMatrixFromQuaternion(p.getLinkState(self.dogId, 1)[1])).reshape(3,3)
        # rotates from knee frame to world frame

        # calculate the position of the foot
        # we know the length of the calf is calf_length
        # the angle of the knee joint is beta
        foot_pos_delta = [-self.calf_length * np.sin(beta), 0, -self.calf_length  * np.cos(beta)]
        foot_pos_delta = np.array(foot_pos_delta)
        # foot_pos_delta -= np.array([0.02, 0, 0.02])
        foot_world_coords = rotmat3 @ foot_pos_delta + knee_joint_world_coords
        print(foot_pos_delta)
        print(rotmat3.T @ (np.array(p.getLinkState(self.dogId, 3)[0]) - knee_joint_world_coords))
        assert t < 2.2 or np.allclose(foot_world_coords, p.getLinkState(self.dogId, 3)[0], 0.03), f"{foot_world_coords} != {p.getLinkState(self.dogId, 3)[0]}, {t}"

        # print(p.getJointState(self.dogId, 0)[0]) # gamma
        # print(p.getJointState(self.dogId, 1)[0]) # alpha
        # print(p.getJointState(self.dogId, 2)[0]) # beta

    def inverse_kinematics(self, foot_x, foot_y, foot_z, hip_width, thigh_length, calf_length):
        """
        Calculate the roll, hip and knee angles from the x,y,z coords of the foot wrt the hip.
        """
        hip_to_foot_distance_along_y_z_plane = np.sqrt(foot_y**2 + foot_z**2)
        bent_leg_length = np.sqrt(hip_to_foot_distance_along_y_z_plane**2 - hip_width**2)
        
        gamma_yz =  -np.arctan(foot_y / foot_z)
        gamma_h_offset =  -np.arctan(-hip_width / bent_leg_length)
        gamma = gamma_yz - gamma_h_offset
        
        # print("hip_to_foot_distance_along_y_z_plane:", hip_to_foot_distance_along_y_z_plane)
        # print("bent_leg_length:", bent_leg_length)
        # print("gamma:", gamma)

        lxzp = np.sqrt(bent_leg_length**2 + foot_x**2)
        n = (lxzp**2 - calf_length**2 - thigh_length**2) / (2*thigh_length)
        beta =  -np.arccos(n / calf_length)
        # assert -1 <= n / calf_length <= 1, f"n / calf_length = {n / calf_length, n, calf_length}"

        alpha_xzp =  -np.arctan(foot_x / bent_leg_length)
        alpha_off = np.arccos((thigh_length + n) / lxzp)
        alpha = alpha_xzp + alpha_off
        if any( np.isnan([gamma,alpha,beta])):
            print([gamma,alpha,beta])
            print(foot_x, foot_y, foot_z, hip_width,thigh_length,calf_length)
            # pass
        return [gamma,alpha,beta]

    
    def shift_body_weight2(self, leg_idx, leg_t):
        xoff = [0, 0, 0]
        yoff = [0, 0, 0]
        self.setlegsxyz(xoff,
                yoff,
                [0, 0, 0],
                self.vvec)  #0.03 is for tweaking the center of grav.
