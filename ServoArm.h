#ifndef _SERVOARM_H_
#define _SERVOARM_H_

#include "Defines.h"
#include "Position.h"

class ServoArm
{
public:
  ServoArm();

  void setServoArmPosition(int angle);
  
  bool isMirrored();

  void calculateBaseJointPosition();
  void calculatePlatformJointPosition();

public:
  int servoID;
  int pwm;

  int minPulseWidth;
  int maxPulseWidth;

  int minPosition;
  int maxPosition;

  float baseAngle;
  float baseDistance;
  Position baseJoint;

  float platformAngle;
  float platformDistance;
  Position platformJoint;

  bool mirrorServo;

  float position;
  float alphaAngleToHorizontal;
  float betaAngleToXAxis;

  Position platformAnchorPoint_Q;
  Position lengthOfLeg_L;

};

#endif
