//================================================================
// ServoArm.cpp
//================================================================

#include "ServoArm.h"

//======================================================
// Constructor
//======================================================
ServoArm::ServoArm()
{
  baseJoint = Position();
  platformJoint = Position();

  platformAnchorPoint_Q = Position();
  lengthOfLeg_L = Position();
}

//======================================================
// Returns the state of the servo (mirror or not)
//======================================================
bool ServoArm::isMirrored()
{
  return mirrorServo;
}

//======================================================
// Calculates the base joint position
//======================================================
void ServoArm::calculateBaseJointPosition()
{
  baseJoint.x_coord = baseDistance * sin(radians(baseAngle));
  baseJoint.y_coord = baseDistance * cos(radians(baseAngle));
  baseJoint.z_coord = 0.0;
}

//======================================================
// Calculates the platform joint positions
//======================================================
void ServoArm::calculatePlatformJointPosition()
{
  platformJoint.x_coord = platformDistance * sin(radians(platformAngle));
  platformJoint.y_coord = platformDistance * cos(radians(platformAngle));
  platformJoint.z_coord = Z_HOME;
}