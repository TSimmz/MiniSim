#!/usr/bin/python2.7
import turtle
import defines as defs

class ADI:
    'ADI class'
    def __init__(self):
        self.border = turtle.Turtle()
        
        self.border.hideturtle()
        self.border.speed(0)
        self.border.color("white", '#0099ff')
        self.border.penup()
        self.border.setposition(defs.ADI_BDR_SIZE_X / -2, (defs.DISPLAY_HEIGHT / 2) - 75)
        self.border.pendown()
        self.border.pensize(3)

        self.border.begin_fill()
        
        for side in range(4):
            self.border.fd(defs.ADI_BDR_SIZE_X)
            self.border.rt(90)
            
        self.border.end_fill()


#adi_lower = turtle.Turtle()
#adi_lower.speed(0)
#adi_lower.color("white", '#b35900')
#adi_lower.penup()
#adi_lower.setposition(ADI_BDR_SIZE_X / -2, -ADI_BDR_SIZE_Y / 2)
#adi_lower.pendown()
#adi_lower.pensize(3)

#adi_lower.begin_fill()
#adi_lower.fd(ADI_BDR_SIZE_X)
#adi_lower.lt(90)
#adi_lower.fd(ADI_BDR_SIZE_Y / 2)
#adi_lower.lt(90)
#adi_lower.fd(ADI_BDR_SIZE_X)
#adi_lower.lt(90)
#adi_lower.fd(ADI_BDR_SIZE_Y / 2)
#adi_lower.lt(90)
#adi_lower.hideturtle()
#adi_lower.end_fill()

#adi_center = turtle.Turtle()
#adi_center.color("white")
#adi_center.shape("square")
#adi_center.penup()
#adi_center.pensize(6);
#adi_center.speed(0)
#adi_center.setposition(0, 0)


#for i in range(90):
    #adi_border.tilt(i)
    #adi_lower.tilt(i)
    #time.sleep(100)