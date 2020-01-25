#!/usr/bin/python
from __future__ import division

import sys
from serial import Serial
from threading import Thread
from position import Position
from controller import Controller
from servo import Servo

import numpy as np
import math as mt

import time

import Adafruit_PCA9685

###########################################
# initialize global variables
###########################################
p = 0.0
r = 0.0
y = 0.0

###########################################
# Constants (measurements in mm)
###########################################
SERVO_MIN = 150 	# Min pulse length out of 4096
SERVO_MAX = 600  	# Max pulse length out of 4096
BASE_DIST = 122.1	# From center to servo pivot center
PLAT_DIST = 140.5	# From center to joint pivot center
SERVO_LEN = 40.0 	# Length of servo arm
SERVO_DIST = 162.8 	# From center to servo arm pivot center
LEG_LEN = 182.0 	# Length of leg from base to platform
Z_HOME = 144	  	# Height of platform above base
SERVO_LIST = []		# List of servo class objects
PI = mt.pi

###########################################
# Create PWM object to drive servos
###########################################
PWM = Adafruit_PCA9685.PCA9685()

###########################################
# Create Controller object
###########################################
DS4 = Controller()

###########################################
# Platform code
###########################################
p_position = Position()			# Requested position of platform
p_rotation = Position()			# Requested rotation of platform

ROT = np.zeros((3,3))			# Rotational matrix
T = Position()					# Translational matrix
H = Position()					# Center position of platform
H.z = Z_HOME

###########################################
# Function calculating translation vector
###########################################
def getTransMatrix():
	
	T = p_position.add(H)
    
###########################################
# Function calculating rotational matrix
###########################################
def getRotMatrix():
	
	psi = p_rotation.z		# yaw  - z
	theta = p_rotation.y	# pitch- y
	phi = p_rotation.x		# roll - x
	
	# Rotational matrix value calculation	
	ROT[0][0] = mt.cos(psi)*mt.cos(theta)
	ROT[0][1] = -mt.sin(psi)*mt.cos(phi)+mt.cos(psi)*mt.sin(theta)*mt.sin(phi)
	ROT[0][2] = mt.sin(psi)*mt.sin(phi)+mt.cos(psi)*mt.cos(phi)*mt.sin(theta)

	ROT[1][0] = mt.sin(psi)*mt.cos(theta)
	ROT[1][1] = mt.cos(psi)*mt.cos(phi)+mt.sin(psi)*mt.sin(theta)*mt.sin(phi)
	ROT[1][2] = -mt.cos(psi)*mt.sin(phi)+mt.sin(psi)*mt.sin(theta)*mt.cos(phi)

	ROT[2][0] = -mt.sin(theta)
	ROT[2][1] = -mt.cos(theta)*mt.sin(psi)
	ROT[2][2] = mt.cos(theta)*mt.cos(phi)
	
	#print ROT
	#print ("\n")
###########################################
# Helper function to calculate Q
###########################################
def getQandL():
	
	for s in SERVO_LIST:

		s.Q.x = T.x + ROT[0][0]*s.P.x + ROT[0][1]*s.P.y + ROT[0][2]*s.P.z
		s.Q.y = T.y + ROT[1][0]*s.P.x + ROT[1][1]*s.P.y + ROT[1][2]*s.P.z
		s.Q.z = T.z + ROT[2][0]*s.P.x + ROT[2][1]*s.P.y + ROT[2][2]*s.P.z

		s.L = s.Q.sub(s.B)
		
###########################################
# Helper function to calculate needed servo
#  rotation value
###########################################
def getAlpha():

    for s in SERVO_LIST:
		
		L = s.L.magSq() - (mt.pow(LEG_LEN,2) + mt.pow(SERVO_LEN,2))
		M = 2 * SERVO_LEN * (s.Q.z - s.B.z)
		N = 2 * SERVO_LEN * ((mt.cos(s.beta)*(s.Q.x - s.B.x))+(mt.sin(s.beta)*(s.Q.y - s.B.y)))
	
		MM = mt.pow(M,2)
		NN = mt.pow(N,2)
		D = mt.sqrt(MM+NN)
		
		A = mt.asin(mt.radians(L / D)) - mt.atan(mt.radians(N / M))
		
		s.alpha = A if s.inverse else (mt.pi - A)
		
###########################################
# Function to set the postion of servos
###########################################
def Move_Platform():
	
	getTransMatrix()
	getRotMatrix()
	getQandL()
	getAlpha()
	
	for s in SERVO_LIST:
		#s.set_pos_direct(mt.degrees(s.alpha))	
		print("ID {} => {} ".format(s.id, mt.degrees(s.alpha))),
		#s.B.printPos()
		#s.P.printPos()	
		
	print("\n")		

	#servo_pos[i] = constrain(zero[i] - (theta_a[i])*servo_mult, SERVO_MIN, SERVO_MAX)
	#servo_pos[i] = constrain(zero[i] + (theta_a[i])*servo_mult, SERVO_MIN, SERVO_MAX)				
	
###########################################
# Helper function to map controller values
#  to servo values
###########################################
def map(x, in_min, in_max, out_min, out_max):
	return (x - in_min) * (out_max - out_min)/(in_max - in_min) + out_min

###########################################
# Helper function to limit values to btwn
#  a range
###########################################
def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))


###########################################
# Helper function to initialize all servos
###########################################
def Initialize_Servos():
	
	PWM.set_pwm_freq(60)
	
	s0 = Servo(PWM, 0, SERVO_MIN, SERVO_MAX,
		308.0, BASE_DIST,
		273.1, PLAT_DIST,
		90, False)

	s1 = Servo(PWM, 1, SERVO_MIN, SERVO_MAX,
		352.0, BASE_DIST,
		 26.9, PLAT_DIST,
		150, True)

	s2 = Servo(PWM, 2, SERVO_MIN, SERVO_MAX, 
		 68.0, BASE_DIST,
		 33.1, PLAT_DIST,
		330, False)
	
	s3 = Servo(PWM, 3, SERVO_MIN, SERVO_MAX,
		112.0, BASE_DIST,
		146.9, PLAT_DIST,
		30, True)

	s4 = Servo(PWM, 4, SERVO_MIN, SERVO_MAX,
		188.0, BASE_DIST,
		153.1, PLAT_DIST,
		210, False)

	s5 = Servo(PWM, 5, SERVO_MIN, SERVO_MAX,
		232.0, BASE_DIST,
		266.9, PLAT_DIST,
		270,  True)
 
	SERVO_LIST.append(s0)
	SERVO_LIST.append(s1)
	SERVO_LIST.append(s2)
	SERVO_LIST.append(s3)
	SERVO_LIST.append(s4)
	SERVO_LIST.append(s5)
 
	print("\n###########################################")
	for s in SERVO_LIST: 
		print("Initializing Servo {}".format(s.id))
		s.set_coords(Z_HOME)

		time.sleep(0.25)

	print("Servos have been initialized...")
	print("###########################################\n")	

###########################################
# Helper function to initialize controls
###########################################
def Initialize_Controls():

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
	global p
	global r
	global y
   
	print("Starting controls thread...")
	
	while True:
		# Read inputs from controller
		DS4.read_input()	
		
		# Get pitch, roll, and yaw from controller			
		p = DS4.control_map['y']
		r = DS4.control_map['x']
		y = DS4.control_map['rx']

		# Map values to between +-45 degrees
		p_mapped = int(map(p * 100, -100, 100, -90, 90))
		r_mapped = int(map(r * 100, -100, 100, -90, 90))
		y_mapped = int(map(y * 100, -100, 100, -90, 90))
		
		# Assign them to position matrix
		p_rotation.z = mt.radians(y_mapped)
		p_rotation.y = mt.radians(p_mapped)
		p_rotation.x = mt.radians(r_mapped)

	
###########################################
# main 
#   initialize child threads
###########################################
def main():
    
	print("Starting main thread...\n...")
	print("Starting setup...\n...") 

	Initialize_Servos()
  	Initialize_Controls()
	
	# Setup and start the controls thread
	controls_thread = Thread(target=controls, args=("controls_thread",))
	controls_thread.start()

	PWM.set_pwm_freq(60)	# Set frequency to 60hz
	motion = False			# Used to track motion status

	print("Setup complete!")
	print("Press Start to enable Controls & Motion...")
	time.sleep(1)
		
	while True:
		#print("Pitch: {:>6.3f}  Roll: {:>6.3f}  Yaw:{:>6.3f}\r".format(p, r, y)),
		
		#if motion:
		# = 0
		#or s in SERVO_LIST:
		#	set_pos_direct(i,p)
		#	i  = i + 1
		
		#if DS4.control_map['start']:
		#	time.sleep(0.1333)
		#	motion = not motion
		#	if motion:
		#		print("Starting motion...")
		#	else:
		#		print("Stoping motion...")
		
		#print platform_pos
		
		#print("Pitch: {:>6.3f}  Roll: {:>6.3f}  Yaw:{:>6.3f}".format(p_mapped, r_mapped, y_mapped))
		
		Move_Platform()

###########################################
# Execute main 
###########################################
if __name__ == "__main__":
	main()
