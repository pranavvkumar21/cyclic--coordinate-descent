#!/usr/bin/env python3
import pybullet as p
import time
import pybullet_data
import os
import math

client_mode = p.DIRECT

def setup_world():
    physicsClient = p.connect(client_mode)##or p.DIRECT for no    n-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("plane.urdf")
    startPos = [0,0,0]
    startOrientation = p.getQuaternionFromEuler([0,0,math.pi/2])
    boxId = p.loadURDF("../urdf/panda.urdf",startPos, startOrientation,useFixedBase=1)
    p.resetBasePositionAndOrientation(boxId, startPos,  startOrientation)
    return boxId

class Panda:
    def __init__(self,id):
        self.id = id
        self.num_joints = p.getNumJoints(self.id)
        self.joint_names = {}
        for i in range(self.num_joints):
            self.joint_names[str(p.getJointInfo(self.id,i)[1])[2:-1]] = i
        self.n_joints = [i for i in range(self.num_joints) ]
        self.getjointinfo()
        self.target_pos = self.joint_position.
    def getjointinfo(self):

        self.joint_position = []
        self.joint_velocity = []
        self.joint_force = []
        self.joint_torque = []
        n_joints = [i for i in range(self.num_joints) ]
        self.joint_state = p.getJointStates(id,n_joints)
        for i in self.joint_state:
            self.joint_position.append((i[0]))
            self.joint_velocity.append(i[1])
            self.joint_force.append(i[2])
            self.joint_torque.append(i[3]/50.0)
    def move(self,):
        self.target_pos = self.joint_position
        for i in range(len(self.target_pos)):

        max_force = [50]*11
        p.setJointMotorControlArray(self.id,self.n_joints,controlMode=p.POSITION_CONTROL,
                                    targetPositions = self.target_pos,forces=max_force)
id  = setup_world()
pan = Panda(id)
max_force =
for i in range (10000):
    for i in pan.n_joints:
        move(i,1)
        p.stepSimulation()
        time.sleep(1./240.)
print(pan.joint_names)
p.disconnect()
