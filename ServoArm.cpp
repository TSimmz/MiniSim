#include "ServoArm.h"

ServoArm::ServoArm()
{
  baseJoint = Position();
  platformJoint = Position();

  platformAnchorPoint_Q = Position();
  lengthOfLeg_L = Position();
}

void ServoArm::setServoArmPosition(int angle)
{
  
}

bool ServoArm::isMirrored()
{
  return mirrorServo;
}

void ServoArm::setMinMaxPulseWidth(int min, int max)
{
  minPulseWidth = min;
  maxPulseWidth = max;
}

void ServoArm::setMinMaxPosition(int min, int max)
{
  minPosition = min;
  maxPosition = max;
}
