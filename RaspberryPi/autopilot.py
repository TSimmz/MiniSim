#!/usr/bin/env python2.7

import math
from enum import IntEnum

ANGLE_MAX = 15

class Function(IntEnum):
    CircleCW  = 0
    CircleCCW = 1
    SineWave  = 2
    Square    = 3
    Count     = 4

#def circular(radius, axis1, axis2, dir):
#    for i in range(-radius, radius)

def sinusoidal(index):
    angle1 = ANGLE_MAX * (math.sin(math.radians(index)))
    angle2 = ANGLE_MAX * (math.sin(math.radians(index) - (math.pi / 2)))
    
    return angle1, angle2


#def helix(radius, axis1, axis2, axis3, dir):
    #similar to circle

#def square(axis, axis1, axis2, dir):
    #draw square

def chooseAP(ap):
    if ap == Function.CircleCW:
        rand = ap