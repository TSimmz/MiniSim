//================================================================
// Simulation.cpp
//================================================================

#include "Simulation.h"

Simulation::Simulation()
{
  pwm = Adafruit_PWMServoDriver();
  basePosition = Position(0, 0, 0);
  platformPosition = Position(0, 0, Z_HOME);
}

int Simulation::init()
{
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  
  delay(10); 
  
  servoArmInitialization();
}

int Simulation::run(int pitch, int roll, int yaw)
{
  float pitchMapped = radians(map(pitch, 0, 1023, -45, 45)); 
  float rollMapped = radians(map(roll, 0, 1023, -45, 45)); 
  float yawMapped = radians(map(yaw, 0, 1023, -45, 45)); 
  
  requestedPlatformRotation.y_coord = pitchMapped;
  requestedPlatformRotation.x_coord = 0; //rollMapped;
  requestedPlatformRotation.z_coord = 0; //yawMapped;
}

int Simulation::servoArmInitialization()
{
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
  
  }
  servoArmArray[0] = ServoArmArray();
}

void Simulation::updatePlatformPosition()
{
  calculateTranslationalMatrix();
  calculateRotationalMatrix();
  calculatePlatformAnchor();
  calculateLegLength();
  calculateAlphaServoAngle();

  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    pwm.setPWM(arm, 0, servoArmArray[arm].alphaAngleToHorizontal);
  }
}

void Simulation::calculateTranslationalMatrix()
{
  translationalMatrix = requestedPlatformPosition.addPositionToThis(platformHome);
}

void Simulation::calculateRotationalMatrix()
{
  int psi;   // rotation about z-axis (yaw)
  int theta; // rotation about y-axis (pitch)
  int phi;   // rotation about x-axis (roll)

  psi   = requestedPlatformRotation.z_coord;
  theta = requestedPlatformRotation.y_coord;
  phi   = requestedPlatformRotation.x_coord;

  rotationalMatrix[0][0] = cos(psi)*cos(theta);
  rotationalMatrix[0][1] = (-1*sin(psi)*cos(phi)) + (cos(psi)*sin(theta)*sin(phi));
  rotationalMatrix[0][2] = (sin(psi)*sin(phi)) + (cos(psi)*cos(phi)*sin(theta));

  rotationalMatrix[1][0] = sin(psi)*cos(theta);
  rotationalMatrix[1][1] = (cos(psi)*cos(phi)) + (sin(psi)*sin(theta)*sin(phi));
  rotationalMatrix[1][2] = (-1*cos(psi)*sin(phi))+(sin(psi)*sin(theta)*cos(phi));

  rotationalMatrix[2][0] = sin(theta);
  rotationalMatrix[2][1] = -1*cos(theta)*sin(psi);
  rotationalMatrix[2][2] = cos(theta)*cos(phi);
}

void Simulation::calculatePlatformAnchor()
{
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    servoArmArray[arm].platformAnchorPoint_Q.x_coord = translationalMatrix.x_coord + 
        ( rotationalMatrix[0][0] * servoArmArray[arm].platformJoint.x_coord +
          rotationalMatrix[0][1] * servoArmArray[arm].platformJoint.y_coord +
          rotationalMatrix[0][2] * servoArmArray[arm].platformJoint.z_coord );

    servoArmArray[arm].platformAnchorPoint_Q.y_coord = translationalMatrix.y_coord + 
        ( rotationalMatrix[1][0] * servoArmArray[arm].platformJoint.x_coord +
          rotationalMatrix[1][1] * servoArmArray[arm].platformJoint.y_coord +
          rotationalMatrix[1][2] * servoArmArray[arm].platformJoint.z_coord );

    servoArmArray[arm].platformAnchorPoint_Q.z_coord = translationalMatrix.z_coord + 
        ( rotationalMatrix[2][0] * servoArmArray[arm].platformJoint.x_coord +
          rotationalMatrix[2][1] * servoArmArray[arm].platformJoint.y_coord +
          rotationalMatrix[2][2] * servoArmArray[arm].platformJoint.z_coord );

  }
}

void Simulation::calculateLegLength()
{
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    servoArmArray[arm].lengthOfLeg_L = servoArmArray[arm].platformAnchorPoint_Q.subPositionFromThis(servoArmArray[arm].baseJoint);
  }
}

void Simulation::calculateAlphaServoAngle()
{
  float Aknot = 0.0;
  float Lknot = 0.0;
  float Mknot = 0.0;
  float Nknot = 0.0;

  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    Lknot = servoArmArray[arm].lengthOfLeg_L.posMagnitudeSquared() - (float)(pow(LEG_LEN, 2) + pow(SERVO_LEN, 2));
    Mknot = 2.0 * SERVO_LEN * servoArmArray[arm].lengthOfLeg_L.z_coord;
    Nknot = 2.0 * SERVO_LEN * ((cos(servoArmArray[arm].betaAngleToXAxis) * (float)servoArmArray[arm].lengthOfLeg_L.x_coord) + 
                               (sin(servoArmArray[arm].betaAngleToXAxis) * (float)servoArmArray[arm].lengthOfLeg_L.y_coord));

    Aknot = asin(Lknot / (pow(Mknot, 2) + pow(Nknot, 2))) - atan(Nknot / Mknot);

    servoArmArray[arm].alphaAngleToHorizontal = (servoArmArray[arm].isMirrored()) ? Aknot : (M_PI - Aknot);
  }
}
