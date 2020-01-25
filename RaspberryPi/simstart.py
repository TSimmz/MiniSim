#!/usr/bin/env python2.7
from __future__ import division

import sys
from serial import Serial
from threading import Thread
from controller import Controller

import numpy as np
import math as mt

import time

roll  = 0.0
pitch = 0.0
yaw   = 0.0

###########################################
# Create Controller object
###########################################
DS4 = Controller()


###########################################
# Helper function to initialize controls
###########################################
def Initialize_Controller():

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
   
    print("Starting controls thread...")
    
    while True:
        # Read inputs from controller
        DS4.read_input()    
        
        # Get pitch, roll, and yaw from controller          
        roll  = DS4.control_map['x']
        pitch = DS4.control_map['y']
        yaw   = DS4.control_map['rx']
    
###########################################
# main 
#   initialize child threads
###########################################
def main():
    
    print("Starting main thread...\n...")
    print("Starting setup...\n...") 

    Initialize_Controller()
    
    # Setup and start the controls thread
    controls_thread = Thread(target=controls, args=("controls_thread",))
    controls_thread.start()

    print("Setup complete!")
    print("Press Start to enable Controls & Motion...")
    time.sleep(1)
        
    while True:

        #if DS4.control_map['start']:
        #   time.sleep(0.1333)
        #   motion = not motion
        #   if motion:
        #       print("Starting motion...")
        #   else:
        #       print("Stoping motion...")
        
       
        print("Roll: {:>6.3f} | Pitch: {:>6.3f} | Yaw:{:>6.3f}".format(roll, pitch, yaw))
        

###########################################
# Execute main 
###########################################
if __name__ == "__main__":
    main()
