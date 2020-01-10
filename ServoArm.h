#ifndef _SERVOARM_H_
#define _SERVOARM_H_

#include "Position.h"
#include "Defines.h"

class ServoArm
{
public:
  ServoArm();
  ~ServoArm();

  bool isMirrored();
  
  void setMinMaxPulseWidth(int min, int max);
  void setMinMaxPosition(int min, int max);
  
private:
  //int servoID;
  //int pwm;

  int minPulseWidth;
  int maxPulseWidth;

  int minPosition;
  int maxPosition;

  int baseAngle;
  int platformAngle;

  bool mirrorServo;

  float position;
  float alphaAngleToHorizontal;
  float betaAngleToXAxis;

  Position platformAnchorPoint_Q;
  Position lengthOfLeg_L;

};

#endif
