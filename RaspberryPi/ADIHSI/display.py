#!/usr/bin/python2.7
import turtle
from adi import ADI
from hsi import HSI
import os, sys, time

import defines as defs

class Display:
    'Display class'
    def __init__(self):
        
        display = turtle.Screen()
        display.title("ADI HSI")
        display.bgcolor("black")
        
        display.setup(defs.DISPLAY_WIDTH, defs.DISPLAY_HEIGHT, None, None)
        
        adi = ADI()
        hsi = HSI()
    
    def setBackgroundColor(color):
        display.bgcolor(color)