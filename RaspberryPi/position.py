#!/usr/bin/env python2.7

import math

class Position():
    'Position class'
    def __init__(self, x = 0.0, y = 0.0, z = 0.0):
        self.x_coord = x
        self.y_coord = y
        self.z_coord = z
        
    def setNewPosition(self, x, y, z):
        self.x_coord = x
        self.y_coord = y
        self.z_coord = z

    def copyNewPosition(self, other):
        self.x_coord = other.x_coord
        self.y_coord = other.y_coord
        self.z_coord = other.z_coord
    
    def addLeftToRight(self, p1, p2):
        self.x_coord = p1.x_coord + p2.x_coord
        self.y_coord = p1.y_coord + p2.y_coord
        self.z_coord = p1.z_coord + p2.z_coord 

    def subtractLeftFromRight(self, p1, p2):
        self.x_coord = p1.x_coord - p2.x_coord
        self.y_coord = p1.y_coord - p2.y_coord
        self.z_coord = p1.z_coord - p2.z_coord

    def magnitudeSquared(self):
        return (math.pow(self.x_coord, 2) + math.pow(self.y_coord, 2) + math.pow(self.z_coord, 2))
    
    def printPos(self):
        print("[{}, {}, {}]".format(self.x_coord, self.y_coord, self.z_coord))    