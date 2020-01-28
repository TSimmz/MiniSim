#!/usr/bin/python2.7
import turtle
import os, sys, time

panel = turtle.Screen()
panel.bgcolor("black")
panel.title("ADI HSI")

ADI_BDR_SIZE_X = 300
ADI_BDR_SIZE_Y = 300

adi_border = turtle.Turtle()
adi_border.speed(0)
adi_border.color("white", '#0099ff')
adi_border.penup()
adi_border.setposition(ADI_BDR_SIZE_X / -2, -ADI_BDR_SIZE_Y / 2)
adi_border.pendown()
adi_border.pensize(3)

adi_border.begin_fill()
for side in range(4):
    adi_border.fd(ADI_BDR_SIZE_X)
    adi_border.lt(90)
adi_border.hideturtle()
adi_border.end_fill()

adi_lower = turtle.Turtle()
adi_lower.speed(0)
adi_lower.color("white", '#b35900')
adi_lower.penup()
adi_lower.setposition(ADI_BDR_SIZE_X / -2, -ADI_BDR_SIZE_Y / 2)
adi_lower.pendown()
adi_lower.pensize(3)

adi_lower.begin_fill()
adi_lower.fd(ADI_BDR_SIZE_X)
adi_lower.lt(90)
adi_lower.fd(ADI_BDR_SIZE_Y / 2)
adi_lower.lt(90)
adi_lower.fd(ADI_BDR_SIZE_X)
adi_lower.lt(90)
adi_lower.fd(ADI_BDR_SIZE_Y / 2)
adi_lower.lt(90)
adi_lower.hideturtle()
adi_lower.end_fill()

adi_center = turtle.Turtle()
adi_center.color("white")
adi_center.shape("square")
adi_center.penup()
adi_center.pensize(6);
adi_center.speed(0)
adi_center.setposition(0, 0)


for i in range(90):
    adi_border.tilt(i)
    adi_lower.tilt(i)
    time.sleep(100)

def main():
    print("Starting ADI/HSI..")

if __name__ == "__main__":
    main()