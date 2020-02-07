#!/usr/bin/env python2.7

import math
from enum import IntEnum

ANGLE_MAX = 15

class APFunction(IntEnum):
    CircleCW  = 0
    CircleCCW = 1
    SineWave  = 2
    Square    = 3
    Count     = 4

class AutoPilot:
    def __init__(self):
        self.sinIndex = 0.0
        self.normIndex = 0.0
        self.radius = 5.0
        self.flip = False
        
    def circular(self):
        try:
            y = math.sqrt((self.radius * self.radius) - (self.normIndex * self.normIndex))
            
            if self.normIndex == self.radius:
                self.flip = True
            
            if self.normIndex == -self.radius:
                self.flip = False
            
            if self.flip:
                self.normIndex += 0.1
            else:
                self.normIndex -= 0.1
            
            print("X: {} | Y: {}".format(self.normIndex, y))
            return self.normIndex, y
        
        except Exception:
        
            print("X: {} | Y: {}".format(self.normIndex, y))
            
    def sinusoidal(self):
        angle1 = ANGLE_MAX * (math.sin(math.radians(self.sinIndex)))
        angle2 = ANGLE_MAX * (math.sin(math.radians(self.sinIndex) - (math.pi / 2)))
        
        if self.sinIndex == 360.0:
            self.sinIndex = 0.0
        else:
            self.sinIndex += 3.0
        
        return angle1, angle2


    #def helix(radius, axis1, axis2, axis3, dir):
        #similar to circle

    #def square(axis, axis1, axis2, dir):
        #draw square

    def chooseAP(ap):
        if ap == Function.CircleCW:
            rand = ap
