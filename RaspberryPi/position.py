#!/usr/bin/env python2.7

import math

class Position():
    'Position class'
	def __init__(self, x = 0.0, y = 0.0, z = 0.0):
		self.x_coord = x
		self.y_coord = y
		self.z_coord = z
	
	def addLeftToRight(self, p1, p2):
        self.x_coord = p1.x_coord + p2.x_coord
        self.y_coord = p1.y_coord + p2.y_coord
        self.z_coord = p1.z_coord + p2.z_coord 

	def subtractLeftFromRight(self, p1, p2):
        self.x_coord = p1.x_coord - p2.x_coord
        self.y_coord = p1.y_coord - p2.y_coord
        self.z_coord = p1.z_coord - p2.z_coord

	def magnitudeSquared(self):
		return (math.pow(self.x, 2) + mt.pow(self.y, 2) + mt.pow(self.z, 2))
	
	def printPos(self):
		print("[{}, {}, {}]".format(self.x, self.y, self.z))	