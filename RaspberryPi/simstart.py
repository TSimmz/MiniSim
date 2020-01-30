#!/usr/bin/env python2.7
from __future__ import division

import sys
import socket
#import kinematics

from serial import Serial
from threading import Thread
from controller import Controller

import ADIHSI
import Adafruit_PCA9685

import numpy as np
import math as mt

import time

SERVO1 = 0
SERVO2 = 1
SERVO3 = 2
SERVO4 = 3
SERVO5 = 4
SERVO6 = 5

servoArmList = []

roll  = 0.0
pitch = 0.0
yaw   = 0.0

exitThread = False

###########################################
# Create Controller object
###########################################
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
# controls 
#   Retrieve pitch, roll, yaw values from 
#   controller input
###########################################
def controls(threadname):
    global roll
    global pitch
    global yaw
    global exit
   
    print("Starting controls thread...")
    
    while not exitThread:
        # Read inputs from controller
        DS4.read_input()    
        
        # Get pitch, roll, and yaw from controller          
        roll  = DS4.inputKeyMap['x']
        pitch = DS4.inputKeyMap['y']
        yaw   = DS4.inputKeyMap['rx']
        
###########################################
# main 
#   initialize child threads
###########################################
def main():
    
    print("Starting main thread...\n...")
    print("Starting setup...\n...") 

    PWM.set_pwm_freq(60)
    
    #initializeController()
    
    # Setup and start the controls thread
    #controls_thread = Thread(target=controls, args=("controls_thread",))
    #controls_thread.start()
    
    print("Setup complete!")
    time.sleep(1)
        
    while True:
        
        blah = raw_input("A: ")
        
        #if DS4.inputKeyMap['start']:
        #    print("Start pushed");
                
                   
        #print("\rRoll: {:>6.3f} | Pitch: {:>6.3f} | Yaw:{:>6.3f}\r".format(roll, pitch, yaw))

    # Close down threads
    #controls_thread.join()
    #updRX_thread.join()
    
    print "Threads have been closed.."
###########################################
# Execute main 
###########################################
if __name__ == "__main__":
    main()
