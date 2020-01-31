#!/usr/bin/env python2.7

import math

class Keys:
    def __init__(self, name, address):
        self.name    = ''
        self.address = 0x00
        self.value   = False
        self.press   = False
        self.hold    = False
        self.release = False
        self.axis    = 0.0
    
