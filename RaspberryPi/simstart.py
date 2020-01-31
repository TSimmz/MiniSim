#!/usr/bin/env python2.7
from __future__ import division

import sys
import socket
import kinematics
import servoArmDefines

from serial import Serial
from threading import Thread
from controller import Controller
from position import Position

import ADIHSI
import Adafruit_PCA9685

import numpy as np
import math as mt

import time

servoArmList = servoArmDefines.SERVO_LIST

roll  = 0.0
pitch = 0.0
yaw   = 0.0

exitThread = False

PWM = Adafruit_PCA9685.PCA9685()
requestedPlatformPosition = Position(0.0, 0.0, 0.0)
requestedPlatformRotation = Position(0.0, 0.0, 0.0)

DS4 = Controller()
#myDisplay = ADIHSI.Display()

###########################################
# Helper function to initialize controls
###########################################
def initializeController():

    print("\n############################################")

    if DS4.controller_init():
        print("Controller found...")
        time.sleep(0.25)
    else:
        print("Controller not found...")
        print("############################################\n")
        time.sleep(1)
        sys.exit("Exiting...")

###########################################
# controls 
#   Retrieve pitch, roll, yaw values from controller input
###########################################
def controls(threadname):
    global exitThread
    global requestedPlatformPosition
    global requestedPlatformRotation 
    
    print("Starting controls thread...")
    
    while not exitThread:
        # Read inputs from controller
        DS4.read_input()
        
        surge = 0.0
        sway  = 0.0
        heave = 0.0
        roll  = 0.0
        pitch = kinematics.mapValues(DS4.inputKeyMap['y'], -1.0, 1.0, -25.0, 25.0)
        yaw   = 0.0
        
        requestedPlatformPosition.setNewPosition(surge, sway, heave)
        requestedPlatformRotation.setNewPosition(roll, pitch, yaw)
        
        #print("Surge: {} | Sway: {} | Heave: {} | Roll: {} | Pitch: {} | Yaw: {}".format(surge, sway, heave, roll, pitch, yaw))

###########################################
# kinematics 
#   
###########################################
def kinematicsCalc(threadname):
    global exitThread
    global requestedPlatformPosition
    global requestedPlatformRotation 

    print("Starting kinematics thread...")

    while not exitThread:
        
        kinematics.setRequestedPlatformPosition(requestedPlatformPosition)
        kinematics.setRequestedPlatformRotation(requestedPlatformRotation)

        kinematics.calculateTranslationalMatrix() 
        kinematics.calculateRotationalMatrix() 
        kinematics.calculatePlatformAnchors(servoArmList) 
        kinematics.calculateLegLengths(servoArmList) 
        kinematics.calculateAlphaServoAngles(servoArmList) 
        kinematics.calculateServoPWM(servoArmList)

        for leg in servoArmList:
            PWM.set_pwm(leg.id, 0, int(leg.currentPWM))


###########################################
# main 
#   initialize child threads
###########################################
def main():
    global exitThread
    global requestedPlatformPosition
    global requestedPlatformRotation
    
    print("Starting main thread...\n...")
    print("Starting setup...\n...") 

    PWM.set_pwm_freq(60)
    
    initializeController()
    
    # Setup and start the controls thread
    controls_thread = Thread(target=controls, args=("controls_thread",))
    controls_thread.start()

    #kinematics_thread = Thread(target=kinematicsCalc, args=("kinematics_thread",))
    #kinematics_thread.start()
    
    print("Setup complete!")
    time.sleep(1)
        
    while not exitThread:
        try:
            
            kinematicsCalc("k")
            time.sleep(1.0/60.0)
            #if DS4.inputKeyMap['start']:
            #    print("Start pushed");
                    
                       
            #print("\rRoll: {:>6.3f} | Pitch: {:>6.3f} | Yaw:{:>6.3f}\r".format(roll, pitch, yaw))
        except KeyboardInterrupt:
            raise
            exitThread = True
            break

    # Close down threads
    controls_thread.join()
    #kinematics_thread.join()
    
    print("Threads have been closed..")

###########################################
# Execute main 
###########################################
if __name__ == "__main__":
    main()
