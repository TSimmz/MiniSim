#!/usr/bin/env python2.7

import math
import numpy as np
from position import Position
from servoArm import ServoArm

SERVO_MIN   = 150       # Min pulse length out of 4096
SERVO_MAX   = 600       # Max pulse length out of 4096
BASE_DIST   = 122.1     # From center to servo pivot center
PLAT_DIST   = 140.5     # From center to joint pivot center
SERVO_LEN   = 40.0      # Length of servo arm
SERVO_DIST  = 162.8     # From center to servo arm pivot center
LEG_LEN     = 182.0     # Length of leg from base to platform
Z_HOME      = 144       # Height of platform above base
ANGLE_MIN   = -90       
ANGLE_MAX   = 90

requestedPlatformPosition = Position(0.0, 0.0, 0.0)
requestedPlatformRotation = Position(0.0, 0.0, 0.0)

baseHome = Position(0.0, 0.0, 0.0)
platformHome = Position(0.0, 0.0, Z_HOME)

translationalMatrix = Position()
rotationalMatrix = np.zeros((3,3))

def mapValues( x, in_min, in_max, out_min, out_max):
    return ((x - in_min) * (out_max - out_min) + out_min * (in_max - in_min)) / (in_max - in_min)

def setRequestedPlatformPosition(reqPos):
    requestedPlatformPosition.setNewPosition(reqPos.x_coord, reqPos.y_coord, reqPos.z_coord)

def setRequestedPlatformRotation(reqRot):
    requestedPlatformRotation.setNewPosition(reqRot.x_coord, reqRot.y_coord, reqRot.z_coord)

def calculateTranslationalMatrix():
    translationalMatrix.addLeftToRight(requestedPlatformPosition, platformHome)

def printKinematicsPositions():
    surge = requestedPlatformPosition.x_coord
    sway  = requestedPlatformPosition.y_coord
    heave = requestedPlatformPosition.z_coord
    roll  = requestedPlatformRotation.x_coord
    pitch = requestedPlatformRotation.y_coord
    yaw   = requestedPlatformRotation.z_coord
    print("[{}, {}, {}, {}, {}, {}]".format(surge, sway, heave, roll, pitch, yaw))

def calculateRotationalMatrix():

    phi   = math.radians(requestedPlatformRotation.x_coord)
    theta = math.radians(requestedPlatformRotation.y_coord)
    psi   = math.radians(requestedPlatformRotation.z_coord)

    #print("X: {} | Y: {} | Z: {}".format(requestedPlatformRotation.x_coord, requestedPlatformRotation.y_coord, requestedPlatformRotation.z_coord))
          
    rotationalMatrix[0][0] = math.cos(psi)*math.cos(theta)
    rotationalMatrix[0][1] = -math.sin(psi)*math.cos(phi)+math.cos(psi)*math.sin(theta)*math.sin(phi)
    rotationalMatrix[0][2] = math.sin(psi)*math.sin(phi)+math.cos(psi)*math.cos(phi)*math.sin(theta)
 
    rotationalMatrix[1][0] = math.sin(psi)*math.cos(theta)
    rotationalMatrix[1][1] = math.cos(psi)*math.cos(phi)+math.sin(psi)*math.sin(theta)*math.sin(phi)
    rotationalMatrix[1][2] = -math.cos(psi)*math.sin(phi)+math.sin(psi)*math.sin(theta)*math.cos(phi)

    rotationalMatrix[2][0] = -math.sin(theta)
    rotationalMatrix[2][1] = -math.cos(theta)*math.sin(psi)
    rotationalMatrix[2][2] = math.cos(theta)*math.cos(phi)

def calculatePlatformAnchors(legs):
    for leg in legs:
        leg.platformAnchorPoint_Q.x_coord = translationalMatrix.x_coord + (rotationalMatrix[0][0] * leg.platformJoint.x_coord + rotationalMatrix[1][0] * leg.platformJoint.y_coord + rotationalMatrix[2][0] * leg.platformJoint.z_coord )
        leg.platformAnchorPoint_Q.y_coord = translationalMatrix.y_coord + (rotationalMatrix[0][1] * leg.platformJoint.x_coord + rotationalMatrix[1][1] * leg.platformJoint.y_coord + rotationalMatrix[2][1] * leg.platformJoint.z_coord )
        leg.platformAnchorPoint_Q.z_coord = translationalMatrix.z_coord + (rotationalMatrix[0][2] * leg.platformJoint.x_coord + rotationalMatrix[1][2] * leg.platformJoint.y_coord + rotationalMatrix[2][2] * leg.platformJoint.z_coord )

        #print("\nLeg{} Q: [{}, {}, {}]".format(leg.id, leg.platformAnchorPoint_Q.x_coord, leg.platformAnchorPoint_Q.y_coord, leg.platformAnchorPoint_Q.z_coord))
    
def calculateLegLengths(legs):
    for leg in legs:
        leg.legLength_L.subtractLeftFromRight(leg.platformAnchorPoint_Q, leg.baseJoint)

def calculateAlphaServoAngles(legs):
    
    for leg in legs:
        
        leg.LPrime = (math.pow(leg.legLength_L.x_coord,2.0)+math.pow(leg.legLength_L.y_coord,2.0)+math.pow(leg.legLength_L.z_coord,2.0)) - (math.pow(LEG_LEN, 2) - math.pow(SERVO_LEN, 2))
        leg.MPrime = 2.0 * SERVO_LEN *  (leg.platformAnchorPoint_Q.z_coord - leg.baseJoint.z_coord)
        leg.NPrime = 2.0 * SERVO_LEN  * ((math.cos(leg.betaAngle) * (leg.platformAnchorPoint_Q.x_coord - leg.baseJoint.x_coord)) + (math.sin(leg.betaAngle) * (leg.platformAnchorPoint_Q.y_coord - leg.baseJoint.y_coord)))
        
        #print("\n\nLeg{}: Servo_Len: {} | Q: {} | BZ: {}\n\n".format(leg.id, SERVO_LEN, leg.platformAnchorPoint_Q.z_coord, leg.baseJoint.z_coord)) 
        #print("\n\nLeg{}: [{}, {}, {}]\n\n".format(leg.id, leg.LPrime, leg.MPrime, leg.NPrime))
            
        leg.alphaAngle = math.asin(leg.LPrime / math.sqrt(math.pow(leg.MPrime,2) + math.pow(leg.NPrime, 2))) - math.atan(leg.NPrime / leg.MPrime)
        
def calculateServoPWM(legs):
    for leg in legs:
       angle = math.degrees(leg.alphaAngle)

       if leg.isMirror():
           leg.currentPWM = mapValues(angle, ANGLE_MIN, ANGLE_MAX, SERVO_MAX, SERVO_MIN)
       else:
           leg.currentPWM = mapValues(angle, ANGLE_MIN, ANGLE_MAX, SERVO_MIN, SERVO_MAX)
