#!/usr/bin/env python2.7
from __future__ import division

import sys
import socket
import kinematics
import servoArmDefines

from enum import IntEnum
from serial import Serial
from threading import Thread
from controller import Controller
from position import Position
from keys import Keys

import ADIHSI
import Adafruit_PCA9685

import numpy as np
import math as mt

import time

servoArmList = servoArmDefines.SERVO_LIST
keyMap = []

class KEY(IntEnum):
    Surge  = 0
    Sway   = 1
    Heave  = 2
    Roll   = 3
    Pitch  = 4
    Yaw    = 5
    SetAP  = 6
    NewAP  = 7
    Freeze = 8
    Reset  = 9
    Start  = 10
    Brake  = 11
    Throttle = 12
        
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

def initializeKeyMap():
    for i in range(21):
        newKey = Keys()
        keyMap.append(newKey)
    
###########################################
# controls 
#   Retrieve pitch, roll, yaw values from controller input
###########################################
def controls(threadname):
    global exitThread
    global keyMap
    global requestedPlatformPosition
    global requestedPlatformRotation 
    
    print("Starting controls thread...")
    
    MIN = -10
    MAX = 10
    
    while not exitThread:
        # Read inputs from controller
        DS4.read_input()
        
        keyMap[KEY.Start].setKeyState(DS4.inputKeyMap['start'])
        keyMap[KEY.Reset].setKeyState(DS4.inputKeyMap['ps'])
        keyMap[KEY.NewAP].setKeyState(DS4.inputKeyMap['sel'])
        keyMap[KEY.SetAP].setKeyState(DS4.inputKeyMap['l3'])
        keyMap[KEY.Freeze].setKeyState(DS4.inputKeyMap['r3'])
        keyMap[KEY.Surge].setAxis(DS4.inputKeyMap['dpad_y'])
        keyMap[KEY.Sway].setAxis(DS4.inputKeyMap['dpad_x'])
        keyMap[KEY.Heave].setAxis(DS4.inputKeyMap['rt_y'])
        keyMap[KEY.Roll].setAxis(DS4.inputKeyMap['lt_x'])
        keyMap[KEY.Pitch].setAxis(DS4.inputKeyMap['lt_y'])
        keyMap[KEY.Yaw].setAxis(DS4.inputKeyMap['rt_x'])
        keyMap[KEY.Throttle].setAxis(DS4.inputKeyMap['r2'])
        keyMap[KEY.Brake].setAxis(DS4.inputKeyMap['l2'])
        
#         surge = 0.0 #kinematics.mapValues(DS4.inputKeyMap['y'], -1.0, 1.0, MIN, MAX)
#         sway  = 0.0 #kinematics.mapValues(DS4.inputKeyMap['x'], -1.0, 1.0, MIN, MAX)
#         heave = 0.0 #kinematics.mapValues(DS4.inputKeyMap['ry'], -1.0, 1.0, MIN, MAX)
#         roll  = kinematics.mapValues(DS4.inputKeyMap['lt_x'], -1.0, 1.0, MIN, MAX)
#         pitch = kinematics.mapValues(DS4.inputKeyMap['lt_y'], -1.0, 1.0, MIN, MAX)
#         yaw   = kinematics.mapValues(DS4.inputKeyMap['rt_y'], -1.0, 1.0, MIN, MAX)
        
#        requestedPlatformPosition.setNewPosition(surge, sway, heave)
#        requestedPlatformRotation.setNewPosition(roll, pitch, yaw)
        
###########################################
# kinematics 
###########################################
def kinematicsCalc(threadname):
    global exitThread
    global requestedPlatformPosition
    global requestedPlatformRotation 

    #print("Starting kinematics thread...")
        
    kinematics.setRequestedPlatformPosition(requestedPlatformPosition)
    kinematics.setRequestedPlatformRotation(requestedPlatformRotation)
    
    #kinematics.printKinematicsPositions()

    kinematics.calculateTranslationalMatrix() 
    kinematics.calculateRotationalMatrix() 
    kinematics.calculatePlatformAnchors(servoArmList) 
    kinematics.calculateLegLengths(servoArmList) 
    kinematics.calculateAlphaServoAngles(servoArmList) 
    kinematics.calculateServoPWM(servoArmList)

    for leg in servoArmList:
        PWM.set_pwm(leg.id, 0, int(leg.currentPWM))

#def handleInputs():
    
    

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
    initializeKeyMap()
    
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
            #time.sleep(1.0/60.0)
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
