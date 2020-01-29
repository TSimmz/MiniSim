#!/usr/bin/env python2.7

import math
from position import Position

class ServoArm():
    'Servo Arm class'
    def __init__(self):
        self.id = 0
        self.currentPWM = 0
        
        self.minPWM = 0
        self.maxPWM = 0

        self.minPos = 0
        self.maxPos = 0

        self.baseAngle = 0.0
        self.baseDistance = 0.0
        self.baseJoint = Position()

        self.platformAngle = 0.0
        self.platformDistance = 0.0
        self.platformJoint = Position()

        self.mirror = False;

        self.alphaAngle = 0.0
        self.betaAngle = 0.0

        self.LPrime = 0.0
        self.MPrime = 0.0
        self.NPrime = 0.0

        self.platformAnchorPoint_Q = Position()
        self.legLength_L = Position()

    def calculateBaseJointPosition():
        self.baseJoint.x_coord = self.baseDistance * math.sin(math.radians(self.baseAngle))
        self.baseJoint.y_coord = self.baseDistance * math.cos(math.radians(self.baseAngle))
        self.baseJoint.z_coord = 0.0

    def calculatePlatformJointPosition():
        self.platformJoint.x_coord = self.platformDistance * math.sin(math.radians(self.platformAngle))
        self.platformJoint.y_coord = self.platformDistance * math.cos(math.radians(self.platformAngle))
        self.platformJoint.z_coord = 0.0