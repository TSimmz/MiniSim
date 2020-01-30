#!/usr/bin/env python2.7

import math
from position import Position
from servoArm import ServoArm

SERVO_MIN   = 150 	    # Min pulse length out of 4096
SERVO_MAX   = 600  	    # Max pulse length out of 4096
BASE_DIST   = 122.1	    # From center to servo pivot center
PLAT_DIST   = 140.5	    # From center to joint pivot center
SERVO_LEN   = 40.0 	    # Length of servo arm
SERVO_DIST  = 162.8 	# From center to servo arm pivot center
LEG_LEN     = 182.0 	# Length of leg from base to platform
Z_HOME      = 144	  	# Height of platform above base
ANGLE_MIN   = -90       
ANGLE_MAX   = 90

requestedPlatformPosition = Position()
requestedPlatformRotation = Position()

baseHome = Position()
platformHome = Position()

translationalMatrix = Position()
rotationalMatrix = np.zeroes((3,3))

def mapValues( x, in_min, in_max, out_min, out_max):
    return ((x - in_min) * (out_max - out_min) + out_min * (in_max - in_min)) / (in_max - in_min)

def calculateTranlationalMatrix():
    translationalMatrix.addLeftToRight(requestedPlatformPosition, platformHome)

def calculateRotationalMatrix():

    phi   = requestedPlatformRotation.x_coord
    theta = requestedPlatformRotation.y_coord
    psi   = requestedPlatformRotation.z_coord

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
        leg.platformAnchorPoint_Q.x_coord = translationalMatrix.x_coord + 
            ( rotationalMatrix[0][0] * leg.platformJoint.x_coord + 
              rotationalMatrix[0][1] * leg.platformJoint.y_coord +
              rotationalMatrix[0][2] * leg.platformJoint.z_coord )

        leg.platformAnchorPoint_Q.y_coord = translationalMatrix.y_coord + 
            ( rotationalMatrix[1][0] * leg.platformJoint.x_coord + 
              rotationalMatrix[1][1] * leg.platformJoint.y_coord +
              rotationalMatrix[1][2] * leg.platformJoint.z_coord )

        leg.platformAnchorPoint_Q.z_coord = translationalMatrix.z_coord + 
            ( rotationalMatrix[2][0] * leg.platformJoint.x_coord + 
              rotationalMatrix[2][1] * leg.platformJoint.y_coord +
              rotationalMatrix[2][2] * leg.platformJoint.z_coord )

def calculateLegLengths(legs):
    for leg in legs:
        leg.legLength_L.subtractLeftFromRight(leg.platformAnchorPoint_Q, leg.baseJoint)

def calculateAlphaServoAngles(legs):
    for leg in legs:
        leg.LPrime = leg.legLength_L.magnitudeSquared() - (math.pow(LEG_LEN, 2) + math.pow(SERVO_LEN, 2))
        leg.MPrime = 2.0 * SERVO_LEN *  (leg.platformAnchorPoint_Q.z_coord - leg.baseJoint.z_coord)
        leg.NPrime = 2.0 * SERVO_LEN  * 
            ((cos(leg.betaAngleToXAxis) * (leg.platformAnchorPoint_Q.x_coord - leg.baseJoint.x_coord)) +
             (sin(leg.betaAngleToXAxis) * (leg.platformAnchorPoint_Q.y_coord - leg.baseJoint.y_coord)))

        leg.alphaAngle = 
            math.asin(leg.Lprime / (math.sqrt((leg.Mprime * leg.Mprime) + (leg.Nprime * leg.Nprime)))) - 
            atan(leg.Nprime / leg.Mprime)
        
def calculateServoPWM(legs):
    for leg in legs:
       angle = math.degrees(leg.alphaAngle)

       if leg.isMirror():
           leg.currentPWM = mapValue(angle, )
        

def calculatePlatformPosition():
  calculateTranslationalMatrix()
  calculateRotationalMatrix()
  calculatePlatformAnchors()
  calculateLegLengths()
  calculateAlphaServoAngles()
  calculateServoPWM()