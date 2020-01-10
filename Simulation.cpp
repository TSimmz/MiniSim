//================================================================
// Simulation.cpp
//================================================================

#include "Simulation.h"

Simulation::Simulation()
{

}

Simulation::~Simulation()
{

}

int Simulation::init()
{
    servoArmInitialization();
    controllerInitialization();
}

int Simulation::run()
{

}

int Simulation::servoArmInitialization()
{

}

int Simulation::controllerInitialization()
{

}

void Simulation::updatePlatformPosition()
{

}

void Simulation::calculateTranslationalMatrix()
{
  //translationalMatrix = requestedPlatformPosition.addPostionToThis(platformHome);
}

void Simulation::calculateRotationalMatrix()
{
  int psi;   // rotation about z-axis (yaw)
  int theta; // rotation about y-axis (pitch)
  int phi;   // rotation about x-axis (roll)

  psi   = requestedPlatformRotation.getZCoord();
  theta = requestedPlatformRotation.getYCoord();
  phi   = requestedPlatformRotation.getXCoord();

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

}

void Simulation::calculateLegLength()
{
  
}

int Simulation::getAlphaAngle()
{

}
