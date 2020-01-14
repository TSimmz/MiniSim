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

void ServoArm::calculateBasePosition()
{
  basePosition.x_coord = baseDistance * sin(radians(baseAngle));
  basePosition.y_coord = baseDistance * cos(radians(baseAngle));
  basePosition.z_coord = 0.0;
}

void ServoArm::calculatePlatformPosition()
{
  platformPosition.x_coord = platformDistance * sin(radians(platformAngle));
  platformPosition.y_coord = platformDistance * cos(radians(platformAngle));
  platformPosition.z_coord = Z_HOME;
}