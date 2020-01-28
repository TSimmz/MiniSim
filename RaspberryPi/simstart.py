#!/usr/bin/env python2.7
from __future__ import division

import sys
import socket
from serial import Serial
from threading import Thread
from controller import Controller

import numpy as np
import math as mt

import time

roll  = 0.0
pitch = 0.0
yaw   = 0.0

exitThread = False;

###########################################
# Set up UPD server
###########################################
SERVER_IP    = "127.0.0.1"
SERVER_PORT  = 9000
SERVER_ADDR  = (SERVER_IP, SERVER_PORT)

###########################################
# Set up Arduino client
###########################################
ARDUINO_IP   = "192.168.0.123"
ARDUINO_PORT = 1234
ARDUINO_ADDR = (ARDUINO_IP, ARDUINO_PORT)

###########################################
# Set up RaspPi client
###########################################
RASPPI_IP    = "192.168.0.210"
RASPPI_PORT  = 1235
RASPPI_ADDR  = (RASPPI_IP, RASPPI_PORT)

###########################################
# Create Controller object
###########################################
DS4 = Controller()

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
# Recieve UPD packets
###########################################
def recieveUPD():
    global exitThread
    
    print("Starting receive UPD thread...")
    
    dataFromArduino = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dataFromArduino.bind(("", SERVER_PORT))
    dataFromArduino.setblocking(0)    
    
    while not exitThread:
        try:
            data,addr = dataFromArduino.recvfrom(1024) 
            dataFromArduino.sendto(str(data),addr)
        except socket.error:
            pass

    time.sleep(.2)

###########################################
# main 
#   initialize child threads
###########################################
def main():
    
    print("Starting main thread...\n...")
    print("Starting setup...\n...") 

    initializeController()
    
    # Setup and start the controls thread
    #controls_thread = Thread(target=controls, args=("controls_thread",))
    #controls_thread.start()
    
    # Setup and start RX UPD thread
    updRX_thread = Thread(target=recieveUPD)
    updRX_thread.start()
    
    dataToArduino = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    dataToArduino.setblocking(0)

    print("Setup complete!")
    time.sleep(1)
        
    while True:
        try:
            data = "Hello, Arduino"
            #print("Sending to {}:{}".format(SERVER_IP, SERVER_PORT))
            dataToArduino.sendto(data, ARDUINO_ADDR)
            time.sleep(0.2)
            
            d, a = dataToArduino.recvfrom(1024)
            print ("Received: {}".format(d))
            
            #data_in, addr = arduinoSocket.recvfrom(1024)
            #print("IP: {} Message: {}".format(addr, data))
            
            #if DS4.inputKeyMap['start']:
            #    print("Start pushed");
                
                   
            #print("\rRoll: {:>6.3f} | Pitch: {:>6.3f} | Yaw:{:>6.3f}\r".format(roll, pitch, yaw))
        
        except socket.error,msg:    
            #If no data is received you end up here, but you can ignore
            #the error and continue
            pass   
        
        except KeyboardInterrupt:
            exit = True
            print "Received Ctrl+C... initiating exit"
            break
        
        time.sleep(.1)
        
    # Close down threads
    #controls_thread.join()
    updRX_thread.join()
    
    print "Threads have been closed.."
###########################################
# Execute main 
###########################################
if __name__ == "__main__":
    main()
