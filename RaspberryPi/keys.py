#!/usr/bin/env python2.7

import math

class Keys:
    
    axisCount = 0
    buttonCount = 0
    
    def __init__(self):
        self.value    = False
        self.prevVal  = False
        self.press    = False
        self.hold     = False
        self.release  = False
        self.axis     = 0.0
        
    def setAxis(self, value):
        self.axis = value
        
    def setKeyState(self, value):
        self.value = value
        
        if self.value and not self.prevVal:
            self.press = True
            self.hold  = False
            self.release = False
        
        if self.value and self.prevVal:
            self.press = False
            self.hold  = True
            self.release = False
            
        if not self.value and self.prevVal:
            self.press = False
            self.hold = False
            self.release = True
        
        if not self.value and not self.prevVal:
            self.press = False
            self.hold = False
            self.release = False
            
        self.prevVal = self.value
    
    def isPressed(self):
        return self.press
    
    def isHeld(self):
        return self.hold
    
    def isReleased(self):
        return self.release
        
    
