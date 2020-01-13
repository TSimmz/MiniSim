//================================================================
// Simulation.h
//================================================================

#ifndef __SIM_H__
#define __SIM_H__

#include "Defines.h"
#include "Position.h"
#include "ServoArm.h"

class Simulation
{
public:
    Simulation();

    int init();
    int run(int pitch, int roll, int yaw);

private:
    // Simulation Initializtion
    int servoArmInitialization();

    // Stewart Platform calculation functions
    void updatePlatformPosition();
    void calculateTranslationalMatrix();
    void calculateRotationalMatrix();
    void calculatePlatformAnchor();
    void calculateLegLength();
    void calculateAlphaServoAngle();

    Position basePosition;
    Position platformPosition;

    // Servo information
    Adafruit_PWMServoDriver pwm;
    ServoArm servoArmArray[6];

    // Requested translational/rotational positions
    Position requestedPlatformPosition;
    Position requestedPlatformRotation;

    // Matrices 
    Position translationalMatrix;
    float rotationalMatrix[3][3];

    Position platformHome;
};

#endif
