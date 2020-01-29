#!/usr/bin/python2.7
import turtle
import defines as defs

class HSI:
    'HSI class'
    def __init__(self):
        self.border = turtle.Turtle()
        
        self.border.hideturtle()
        self.border.speed(0)
        self.border.color("green")
        self.border.penup()
        self.border.setposition(0, (defs.DISPLAY_HEIGHT / - 2) + 50)
        self.border.pendown()
        self.border.pensize(3)
        
        self.border.circle(defs.HSI_DIAMETER / 2)
