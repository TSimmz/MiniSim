//================================================================
// ServoArm.cpp
//================================================================

#include "ServoArm.h"

//======================================================
// Constructor
//======================================================
ServoArm::ServoArm()
{
  Lprime = 0.0;
  Mprime = 0.0;
  Nprime = 0.0;
  
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
  platformJoint.z_coord = 0.0;
}

//======================================================
// 
//======================================================
void ServoArm::calculateHomeAngle()
{
  homeAngle = sqrt( sq(LEG_LEN) + sq(SERVO_LEN) - sq(platformJoint.x_coord - baseJoint.x_coord) - sq(platformJoint.y_coord - baseJoint.y_coord)) - platformJoint.z_coord;
}

//======================================================
// 
//======================================================
void ServoArm::calculateAlphaKnot()
{
  float Aknot = 0.0;
  float Lknot = 0.0;
  float Mknot = 0.0;
  float Nknot = 0.0;

  Lknot = 2.0 * sq(SERVO_LEN);
  Mknot = 2.0 * SERVO_LEN * (platformJoint.x_coord - baseJoint.x_coord);
  Nknot = 2.0 * SERVO_LEN * (homeAngle + platformJoint.z_coord);

  Aknot = asin(Lknot / sqrt(sq(Mknot) + sq(Nknot))) - atan(Mknot / Nknot);
}
