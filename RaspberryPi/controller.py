#!/usr/bin/env python2.7

import os, struct, array
from fcntl import ioctl
from keys import Keys

class Controller:
    'Controller class'
    def __init__(self):
        self.jsdev = '' 

        self.inputKeyMap = {
            'start': False,
            'select': False,
            'x': 0.0,
            'y': 0.0,
            'ry': 0.0,
            'rx': 0.0,          
        }
        
        self.keyMap = []

        self.prevInputKeyMap = self.inputKeyMap

        self.axis = ''
        self.axis_map = []
        self.axis_states = {}
        self.axis_names = {
            0x00 : 'x',
            0x01 : 'y',
            0x02 : 'z',
            0x03 : 'rx',
            0x04 : 'ry',
            0x05 : 'rz',
            0x06 : 'trottle',
            0x07 : 'rudder',
            0x08 : 'wheel',
            0x09 : 'gas',
            0x0a : 'brake',
            0x10 : 'hat0x',
            0x11 : 'hat0y',
            0x12 : 'hat1x',
            0x13 : 'hat1y',
            0x14 : 'hat2x',
            0x15 : 'hat2y',
            0x16 : 'hat3x',
            0x17 : 'hat3y',
            0x18 : 'pressure',
            0x19 : 'distance',
            0x1a : 'tilt_x',
            0x1b : 'tilt_y',
            0x1c : 'tool_width',
            0x20 : 'volume',
            0x28 : 'misc',
        }
        
        self.button = ''
        self.button_map = []
        self.button_states = {}
        self.button_names = {           
            0x120 : 'trigger',
            0x121 : 'thumb',
            0x122 : 'thumb2',
            0x123 : 'top',
            0x124 : 'top2',
            0x125 : 'pinkie',
            0x126 : 'base',
            0x127 : 'base2',
            0x128 : 'base3',
            0x129 : 'base4',
            0x12a : 'base5',
            0x12b : 'base6',
            0x12f : 'dead',
            0x130 : 'a',
            0x131 : 'b',
            0x132 : 'c',
            0x133 : 'x',
            0x134 : 'y',
            0x135 : 'z',
            0x136 : 'tl',
            0x137 : 'tr',
            0x138 : 'tl2',
            0x139 : 'tr2',
            0x13a : 'select',
            0x13b : 'start',
            0x13c : 'mode',
            0x13d : 'thumbl',
            0x13e : 'thumbr',
        
            0x220 : 'dpad_up',
            0x221 : 'dpad_down',
            0x222 : 'dpad_left',
            0x223 : 'dpad_right',
        
            # XBox 360 controller uses these codes.
            0x2c0 : 'dpad_left',
            0x2c1 : 'dpad_right',
            0x2c2 : 'dpad_up',
            0x2c3 : 'dpad_down',
        }       
    
    ###########################################
    # Controller initialization 
    ###########################################
    def controller_init(self):
        
        # Iterate over the joystick devices.
        print('Available devices:')
        
        found = False
        for fn in os.listdir('/dev/input'):
            if fn.startswith('js'):
                #print('  /dev/input/%s' % (fn))
                found = True
        
        if not found:
            print("No input detected")
            return False
        else:
            print("Input detected in /dev/input")

        # Open the joystick device.
        fn = '/dev/input/js0'
        print('Opening %s...' % fn)
        self.jsdev = open(fn, 'rb')

        # Get the device name.
        buf = array.array('c', ['\0'] * 64)
        ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
        js_name = buf.tostring()
        
        print('Device name: %s' % js_name)

        # Get number of axes and buttons.
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a11, buf) # JSIOCGAXES
        num_axes = buf[0]

        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
        num_buttons = buf[0]

        # Get the axis map.
        buf = array.array('B', [0] * 0x40)
        ioctl(self.jsdev, 0x80406a32, buf) # JSIOCGAXMAP

        for axis in buf[:num_axes]:
            axis_name = self.axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)
            self.axis_states[axis_name] = 0.0
        
        # Get the button map.
        buf = array.array('H', [0] * 200)
        ioctl(self.jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

        for btn in buf[:num_buttons]:
            btn_name = self.button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0

        print('{} axes found: {}'.format(num_axes, ', '.join(self.axis_map)))
        print('{} buttons found: {}'.format(num_buttons, ', '.join(self.button_map)))
        return True
    
    ###########################################
    # 
    ###########################################            
    def setControllerMap(self):
        self.keyMap.append(Keys('reset',     0x13b))
        self.keyMap.append(Keys('autopilot', 0x13a))
        self.keyMap.append(Keys('surgePos',  0x220))
        self.keyMap.append(Keys('surgeNeg',  0x221))
        self.keyMap.append(Keys('swayPos',   0x222))
        self.keyMap.append(Keys('swayNeg',   0x223))
        self.keyMap.append(Keys('changeAP',  0x121))
        self.keyMap.append(Keys('freeze',    0x122))
        self.keyMap.append(Keys('pitch',     0x01))
        self.keyMap.append(Keys('roll',      0x00))
        self.keyMap.append(Keys('heave',     0x04))
        self.keyMap.append(Keys('yaw',       0x03))
        self.keyMap.append(Keys('throttle',  0x02))
        self.keyMap.append(Keys('brake',     0x05))

    ###########################################
    # Check for input changes
    ###########################################            
    def isInputUpdated(self):
        newInput = False
        
        for state in self.inputKeyMap:
            if self.prevInputKeyMap[state] != self.inputKeyMap[state]:
               self.prevInputKeyMap[state] = self.inputKeyMap[state]
               newInput = True
        
        return newInput
    
    ###########################################
    # Read the controller inputs 
    ###########################################
    def read_input(self):
        
        change = False
        
        evbuf = self.jsdev.read(8)
        if evbuf:
            time, value, type, number = struct.unpack('IhBB', evbuf)

            #if type & 0x80:
            #   print "(initial)"

            # Checks the button states
            if type & 0x01:
                self.button = self.button_map[number]
                
                if self.button:
                    self.button_states[self.button] = value
                    if self.button in self.inputKeyMap:
                        self.inputKeyMap[self.button] = value
                    
            # Checks the axis states
            if type & 0x02:
                self.axis = self.axis_map[number]
                
                if self.axis:
                    fvalue = value / 32767.0
                    self.axis_states[self.axis] = fvalue
                    
                    if self.axis in self.inputKeyMap:
                        self.inputKeyMap[self.axis] = fvalue
            
            #change = self.isInputUpdated()
            
            return change