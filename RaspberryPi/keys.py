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
        self.prevVal = self.value
        self.value = value
        
        #print("Value: {} | {} :Prev".format(self.value, self.prevVal))
        
        if self.value and not self.prevVal:
            self.press = True
            self.hold  = False
            self.release = False
            #print("Key pressed")
        
        elif self.value and self.prevVal:
            self.press = False
            self.hold  = True
            self.release = False
            #print("Key held")
            
        elif not self.value and self.prevVal:
            self.press = False
            self.hold = False
            self.release = True
            #print("Key released")
        
#         if not self.value and not self.prevVal:
#             self.press = False
#             self.hold = False
#             self.release = False
    
    def isPressed(self):
        return self.press
    
    def isHeld(self):
        return self.hold
    
    def isReleased(self):
        return self.release
        
    
