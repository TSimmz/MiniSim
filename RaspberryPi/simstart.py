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
from autopilot import AutoPilot, APFunction
from gpiozero import LED

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

class OPMODE(IntEnum):
    NoMode = 0
    Motion = 1
    Frozen = 2
    AutoPilot = 3
    
exitThread = False

OperationalMode = OPMODE.NoMode
prevOperationalMode = OPMODE.NoMode

index = 0

MotionLed = LED(13)
AutoPilotLed = LED(19)
FrozenLed = LED(26)

AP_Routine = APFunction.CircleCW

masterPosition = Position(0.0, 0.0, 0.0)
masterRotation = Position(0.0, 0.0, 0.0)

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
AP  = AutoPilot()
INST = ADIHSI.Display()

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
    
    print("############################################")

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
                
        handleButtons()
        handleAxes()
    
def display():
    global masterPosition
    global masterRotation
    
    print("Starting display thread...")
    
    
    
###########################################
# kinematics 
###########################################
def kinematicsCalc():

    kinematics.calculateTranslationalMatrix() 
    kinematics.calculateRotationalMatrix() 
    kinematics.calculatePlatformAnchors(servoArmList) 
    kinematics.calculateLegLengths(servoArmList) 
    kinematics.calculateAlphaServoAngles(servoArmList) 
    kinematics.calculateServoPWM(servoArmList)

    for leg in servoArmList:
        PWM.set_pwm(leg.id, 0, int(leg.currentPWM))

###########################################
# 
###########################################
def setOperationalMode(newMode):
    global OperationalMode
    global prevOperationalMode
    
    if OperationalMode == newMode:
        OperationalMode = prevOperationalMode
    else:
        prevOperationalMode = OperationalMode
        OperationalMode = newMode
    
###########################################
# handles the button presses from DS4
###########################################
def handleButtons():
    global AP_Routine
    
    if keyMap[KEY.Start].isPressed():
        setOperationalMode(OPMODE.Motion)
        
    if keyMap[KEY.SetAP].isPressed():
        setOperationalMode(OPMODE.AutoPilot)

    if keyMap[KEY.NewAP].isPressed():
        AP_Routine = (AP_Routine + 1) % autopilot.Function.Count
    
    if keyMap[KEY.Reset].isPressed():
        setOperationalMode(OPMODE.NoMode)
    
    if keyMap[KEY.Freeze].isPressed():
        setOperationalMode(OPMODE.Frozen)

###########################################
# handles the axes changes from DS4
###########################################
def handleAxes():
    MIN = -20
    MAX = 20
    
    surge = ctrlPosition.x_coord + keyMap[KEY.Surge].axis
    sway  = ctrlPosition.y_coord + keyMap[KEY.Sway].axis
    heave = kinematics.mapValues(keyMap[KEY.Heave].axis, 1.0, -1.0, -40.0, 40.0)
    roll  = kinematics.mapValues(keyMap[KEY.Roll].axis , -1.0, 1.0, MIN, MAX)
    pitch = kinematics.mapValues(keyMap[KEY.Pitch].axis, -1.0, 1.0, MIN, MAX)
    yaw   = kinematics.mapValues(keyMap[KEY.Yaw].axis  , -1.0, 1.0, MIN, MAX) 
         
    ctrlPosition.setNewPosition(surge, sway, heave)
    ctrlRotation.setNewPosition(roll, pitch, yaw)

###########################################
# 
###########################################
def updatePositionRotation():
    global OperationalMode
    global index
    
    if OperationalMode == OPMODE.Motion:
        kinematics.requestedPlatformPosition.copyNewPosition(ctrlPosition)
        kinematics.requestedPlatformRotation.copyNewPosition(ctrlRotation)
        frozenPosition.copyNewPosition(ctrlPosition)
        frozenRotation.copyNewPosition(ctrlRotation)
        
        masterPosition.copyNewPosition(ctrlPosition)
        masterRotation.copyNewPosiiton(ctrlRotation)

    elif OperationalMode == OPMODE.AutoPilot:
        kinematics.requestedPlatformPosition.copyNewPosition(autoPosition)
        kinematics.requestedPlatformRotation.copyNewPosition(autoRotation)
        frozenPosition.copyNewPosition(autoPosition)
        frozenRotation.copyNewPosition(autoRotation)
        
        masterPosition.copyNewPosition(autoPosition)
        masterRotation.copyNewPosiiton(autoRotation)

    elif OperationalMode == OPMODE.Frozen:
        kinematics.requestedPlatformPosition.copyNewPosition(frozenPosition)
        kinematics.requestedPlatformRotation.copyNewPosition(frozenRotation)
        
        masterPosition.copyNewPosition(frozenPosition)
        masterRotation.copyNewPosiiton(frozenRotation)

    else:
        kinematics.requestedPlatformPosition.copyNewPosition(zeroPosition)
        kinematics.requestedPlatformRotation.copyNewPosition(zeroRotation)
        
        masterPosition.copyNewPosition(zeroPosition)
        masterRotation.copyNewPosiiton(zeroRotation)

###########################################
# main 
#   initialize child threads
###########################################
def main():
    global exitThread
    global OperationalMode
    global autoRotation
    
    print("Starting main thread...\n...")
    print("Starting setup...\n...") 

    PWM.set_pwm_freq(60)
    
    initializeController()
    initializeKeyMap()
    
    # Setup and start the controls thread
    controls_thread = Thread(target=controls, args=("controls_thread",))
    controls_thread.start()
    
    display_thread = Thread(target=display, args=("display_thread",))
    display_thread.start()

    print("Setup complete!")
    time.sleep(1)
        
    index = 0
    
    while not exitThread:
        try:
            
            updatePositionRotation()
            
            if OperationalMode == OPMODE.Motion:
                MotionLed.on()
            else:
                MotionLed.off()
            
            if OperationalMode == OPMODE.AutoPilot:
                AutoPilotLed.on()

                roll, pitch = AP.sinusoidal()
                autoRotation.setNewPosition(roll, pitch, 0.0)
                            
            else:
                AutoPilotLed.off()
            
            if OperationalMode == OPMODE.Frozen:
                FrozenLed.on()
            else:
                FrozenLed.off()
                
            if OperationalMode == OPMODE.NoMode:
                MotionLed.off()
                AutoPilotLed.off()
                FrozenLed.off()
                
                
            kinematicsCalc()
                       
        except KeyboardInterrupt:
            exitThread = True

    # Close down threads
    controls_thread.join()
    display_thread.join()
    
    print("Threads have been closed..")

###########################################
# Execute main 
###########################################
if __name__ == "__main__":
    main()
