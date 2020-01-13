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
  
  void setMinMaxPulseWidth(int min, int max);
  void setMinMaxPosition(int min, int max);
  
public:
  int servoID;
  int pwm;

  int minPulseWidth;
  int maxPulseWidth;

  int minPosition;
  int maxPosition;

  int baseAngle;
  Position baseJoint;

  int platformAngle;
  Position platformJoint;

  bool mirrorServo;

  float position;
  float alphaAngleToHorizontal;
  float betaAngleToXAxis;

  Position platformAnchorPoint_Q;
  Position lengthOfLeg_L;

};

#endif
