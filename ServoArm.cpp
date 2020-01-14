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

void ServoArm::calculateBaseJointPosition()
{
  baseJoint.x_coord = baseDistance * sin(radians(baseAngle));
  baseJoint.y_coord = baseDistance * cos(radians(baseAngle));
  baseJoint.z_coord = 0.0;
}

void ServoArm::calculatePlatformJointPosition()
{
  platformJoint.x_coord = platformDistance * sin(radians(platformAngle));
  platformJoint.y_coord = platformDistance * cos(radians(platformAngle));
  platformJoint.z_coord = Z_HOME;
}