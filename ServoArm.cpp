#include "ServoArm.h"

ServoArm::ServoArm()
{
  
}

ServoArm::~ServoArm()
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
