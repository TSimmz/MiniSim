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

class AP(IntEnum):
    CircleCW  = 0
    CircleCCW = 1
    SineWave  = 2
    Square    = 3
    Count     = 4


exitThread = False

Motion = False
AutoPilot = False
Frozen = False
AP_Routine = AP.CircleCW

reqPosition = Position(0.0, 0.0, 0.0)
reqRotation = Position(0.0, 0.0, 0.0)

zeroPosition = Position(0.0, 0.0, 0.0)
zeroRotation = Position(0.0, 0.0, 0.0)

ctrlPosition = Position(0.0, 0.0, 0.0)
ctrlRotation = Position(0.0, 0.0, 0.0)

autoPosition = Position(0.0, 0.0, 0.0)
autoRotation = Position(0.0, 0.0, 0.0)

frozenPosition = Position(0.0, 0.0, 0.0)
frozenRotation = Position(0.0, 0.0, 0.0)

PWM = Adafruit_PCA9685.PCA9685()
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
# 
###########################################
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
        
        
###########################################
# kinematics 
###########################################
def kinematicsCalc():

    kinematics.setRequestedPlatformPosition(reqPosition)
    kinematics.setRequestedPlatformRotation(reqRotation)
    kinematics.calculateTranslationalMatrix() 
    kinematics.calculateRotationalMatrix() 
    kinematics.calculatePlatformAnchors(servoArmList) 
    kinematics.calculateLegLengths(servoArmList) 
    kinematics.calculateAlphaServoAngles(servoArmList) 
    kinematics.calculateServoPWM(servoArmList)

    for leg in servoArmList:
        PWM.set_pwm(leg.id, 0, int(leg.currentPWM))

###########################################
# handles the button presses from DS4
###########################################
def handleButtons():
    
    if keyMap[KEY.Start].isPressed():
        Motion = not Motion
        
    if keyMap[KEY.SetAP].isPressed():
        AutoPilot = not AutoPilot

    if keyMap[KEY.NewAP].isPressed():
        AP_Routine = (AP_Routine + 1) % AP.Count
    
    if keyMap[KEY.Reset].isPressed():
        Motion = False
    
    if keyMap[KEY.Freeze].isPressed():
        Frozen = not Frozen

        if Frozen:
            frozenPosition.copyNewPosition(reqPosition)
            frozenPosition.copyNewPosition(reqRotation)


###########################################
# handles the axes changes from DS4
###########################################
def handleAxes():

    surge = surge + keyMap[KEY.Surge].axis
    sway = sway + keyMap[KEY.Sway].axis
    heave = keyMap[KEY.Heave].axis
    roll = keyMap[KEY.Roll].axis
    pitch = keyMap[KEY.Pitch].axis
    yaw = keyMap[KEY.Yaw].axis  
         
    ctrlPosition.setNewPosition(surge, sway, heave)
    ctrlRotation.setNewPosition(roll, pitch, yaw)

###########################################
# 
###########################################
def updatePositionRotation():

    if not Motion:
        reqPosition.copyNewPosition(zeroPosition)
        reqRotation.copyNewPosition(zeroRotation)

    elif Frozen:
        reqPosition.copyNewPosition(frozenPosition)
        reqRotation.copyNewPosition(frozenRotation)
    
    elif AutoPilot:
        reqPosition.copyNewPosition(autoPosition)
        reqRotation.copyNewPosition(autoRotation)
    
    else:
        reqPosition.copyNewPosition(ctrlPosition)
        reqRotation.copyNewPosition(ctrlRotation)

###########################################
# main 
#   initialize child threads
###########################################
def main():
    global exitThread
    
    print("Starting main thread...\n...")
    print("Starting setup...\n...") 

    PWM.set_pwm_freq(60)
    
    initializeController()
    initializeKeyMap()
    
    # Setup and start the controls thread
    controls_thread = Thread(target=controls, args=("controls_thread",))
    controls_thread.start()

    print("Setup complete!")
    time.sleep(1)
        
    while not exitThread:
        try:

            handleButtons()
            handleAxes()                    
            updatePositionRotation()

            kinematicsCalc()
                       
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
