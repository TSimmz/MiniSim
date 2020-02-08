#!/usr/bin/python2.7
import turtle
from adi import ADI
from hsi import HSI
import os, sys, time

import defines as defs

class Display:
    'Display class'
    def __init__(self):
        
        self.display = turtle.Screen()
        self.display.title("ADI HSI")
        self.display.bgcolor("black")
        
        self.display.setup(defs.DISPLAY_WIDTH, defs.DISPLAY_HEIGHT, None, None)
        
        self.adi = ADI()
        self.hsi = HSI()
        
    def updateADI(mPos, mRot):
        self.adi.updatePitch(mRot.y_coord)