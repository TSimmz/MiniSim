#!/usr/bin/env python2.7

import math
import kinematics
from servoArm import ServoArm

SERVO1 = 0
SERVO2 = 1
SERVO3 = 2
SERVO4 = 3
SERVO5 = 4
SERVO6 = 5

SERVO_LIST = []

Servo1 = ServoArm()
Servo1.id = SERVO1
Servo1.currentPWM = 0
Servo1.minPWM = kinematics.SERVO_MIN
Servo1.maxPWM = kinematics.SERVO_MAX
Servo1.minPos = kinematics.ANGLE_MIN
Servo1.maxPos = kinematics.ANGLE_MAX
Servo1.baseAngle = 352.0
Servo1.baseDistance = kinematics.BASE_DIST
Servo1.platformAngle = 26.9
Servo1.platformDistance = kinematics.PLAT_DIST
Servo1.betaAngle = math.radians(150.0)
Servo1.mirror = False

Servo2 = ServoArm()
Servo2.id = SERVO2
Servo2.currentPWM = 0
Servo2.minPWM = kinematics.SERVO_MIN
Servo2.maxPWM = kinematics.SERVO_MAX
Servo2.minPos = kinematics.ANGLE_MIN
Servo2.maxPos = kinematics.ANGLE_MAX
Servo2.baseAngle = 308.0
Servo2.baseDistance = kinematics.BASE_DIST
Servo2.platformAngle = 273.1
Servo2.platformDistance = kinematics.PLAT_DIST
Servo2.betaAngle = math.radians(90)
Servo2.mirror = True

Servo3 = ServoArm()
Servo3.id = SERVO3
Servo3.currentPWM = 0
Servo3.minPWM = kinematics.SERVO_MIN
Servo3.maxPWM = kinematics.SERVO_MAX
Servo3.minPos = kinematics.ANGLE_MIN
Servo3.maxPos = kinematics.ANGLE_MAX
Servo3.baseAngle = 232.0
Servo3.baseDistance = kinematics.BASE_DIST
Servo3.platformAngle = 266.9
Servo3.platformDistance = kinematics.PLAT_DIST
Servo3.betaAngle = math.radians(-90)
Servo3.mirror = False

Servo4 = ServoArm()
Servo4.id = SERVO4
Servo4.currentPWM = 0
Servo4.minPWM = kinematics.SERVO_MIN
Servo4.maxPWM = kinematics.SERVO_MAX
Servo4.minPos = kinematics.ANGLE_MIN
Servo4.maxPos = kinematics.ANGLE_MAX
Servo4.baseAngle = 188.0
Servo4.baseDistance = kinematics.BASE_DIST
Servo4.platformAngle = 153.1
Servo4.platformDistance = kinematics.PLAT_DIST
Servo4.betaAngle = math.radians(-150.0)
Servo4.mirror = True

Servo5 = ServoArm()
Servo5.id = SERVO5
Servo5.currentPWM = 0
Servo5.minPWM = kinematics.SERVO_MIN
Servo5.maxPWM = kinematics.SERVO_MAX
Servo5.minPos = kinematics.ANGLE_MIN
Servo5.maxPos = kinematics.ANGLE_MAX
Servo5.baseAngle = 112.0
Servo5.baseDistance = kinematics.BASE_DIST
Servo5.platformAngle = 146.9
Servo5.platformDistance = kinematics.PLAT_DIST
Servo5.betaAngle = math.radians(30.0)
Servo5.mirror = False

Servo6 = ServoArm()
Servo6.id = SERVO6
Servo6.currentPWM = 0
Servo6.minPWM = kinematics.SERVO_MIN
Servo6.maxPWM = kinematics.SERVO_MAX
Servo6.minPos = kinematics.ANGLE_MIN
Servo6.maxPos = kinematics.ANGLE_MAX
Servo6.baseAngle = 68.0
Servo6.baseDistance = kinematics.BASE_DIST
Servo6.platformAngle = 33.1
Servo6.platformDistance = kinematics.PLAT_DIST
Servo6.betaAngle = math.radians(-30.0)
Servo6.mirror = True

SERVO_LIST.append(Servo1)
SERVO_LIST.append(Servo2)
SERVO_LIST.append(Servo3)
SERVO_LIST.append(Servo4)
SERVO_LIST.append(Servo5)
SERVO_LIST.append(Servo6)

for servo in SERVO_LIST:
    servo.calculateBaseJointPosition()
    servo.calculatePlatformJointPosition()