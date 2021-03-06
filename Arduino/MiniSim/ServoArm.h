//================================================================
// ServoArm.h
//================================================================

#ifndef _SERVOARM_H_
#define _SERVOARM_H_

#include "Defines.h"
#include "Position.h"

class ServoArm
{
public:
  ServoArm();
  
  bool isMirrored();

  void calculateBaseJointPosition();
  void calculatePlatformJointPosition();
  void calculateHomeAngle();
  void calculateAlphaKnot();

public:
  int servoID;
  int currentPWM;

  int minPulseWidth;
  int maxPulseWidth;

  int minPosition;
  int maxPosition;

  float homeAngle;
  
  float baseAngle;
  float baseDistance;
  Position baseJoint;

  float platformAngle;
  float platformDistance;
  Position platformJoint;

  bool mirrorServo;

  float alphaKnot;
  
  float alphaAngleToHorizontal;
  float betaAngleToXAxis;

  float Lprime;
  float Mprime;
  float Nprime;

  Position platformAnchorPoint_Q;
  Position lengthOfLeg_L;

};

#endif
