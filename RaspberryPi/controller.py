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
            'sel' : False, 
            'ps'  : False,
            'x'   : False,
            'c'   : False,
            't'   : False,
            's'   : False,
            'lt1' : False,
            'lt2' : False,
            'l3'  : False,
            'rt1' : False,
            'rt2' : False,
            'r3'  : False,
            'l2'  : 0.0,
            'r2'  : 0.0,
            'lt_y': 0.0,
            'lt_x': 0.0,
            'rt_y': 0.0,
            'rt_x': 0.0,
            'dpad_y': 0.0,
            'dpad_x': 0.0,           
        }
                
        self.axis = ''
        self.axis_map = []
        self.axis_states = {}
        self.axis_names = {
            0x00 : 'lt_x',
            0x01 : 'lt_y',
            0x02 : 'l2',
            0x03 : 'rt_x',
            0x04 : 'rt_y',
            0x05 : 'r2',
            0x10 : 'dpad_x',
            0x11 : 'dpad_y',
        }
        
        self.button = ''
        self.button_map = []
        self.button_states = {}
        self.button_names = {           
            0x130 : 'x',
            0x131 : 'c',
            0x133 : 't',
            0x134 : 's',
            0x136 : 'lt1',
            0x137 : 'rt1',
            0x138 : 'lt2',
            0x139 : 'rt2',
            0x13a : 'sel',
            0x13b : 'start',
            0x13c : 'ps',
            0x13d : 'l3',
            0x13e : 'r3',
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
    # Read the controller inputs 
    ###########################################
    def read_input(self):

        evbuf = self.jsdev.read(8)
        if evbuf:
            time, value, type, number = struct.unpack('IhBB', evbuf)

            # Checks the button states
            if type & 0x01:
                self.button = self.button_map[number]
                
                if self.button:
                    self.button_states[self.button] = value
                    
                    #print("Value: {} |\tType: {} |\tButton: {} |\t\tNumber: {}".format(value, type, self.button, number))
                    if self.button in self.inputKeyMap:
                        self.inputKeyMap[self.button] = value
                    
            # Checks the axis states
            if type & 0x02:
                self.axis = self.axis_map[number]
                
                if self.axis:
                    fvalue = value / 32767.0
                    self.axis_states[self.axis] = fvalue
                    
                    #print("Value: {} |\tType: {} |\tAxis: {} |\t\tNumber: {}".format(value, type, self.axis, number))
                    if self.axis in self.inputKeyMap:
                        self.inputKeyMap[self.axis] = fvalue
