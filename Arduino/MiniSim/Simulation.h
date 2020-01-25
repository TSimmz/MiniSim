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

    void init();
    void run(int * moveArray);

private:
    // Simulation Initializtion
    int servoArmInitialization();
    void runServoTest(int servoID);

    // Stewart Platform calculation functions
    void calculatePlatformPosition();
    void updateServoPosition();
    
    void calculateRotationalMatrix();
    void calculateTranslationalMatrix();
    void calculatePlatformAnchor();
    void calculateLegLength();
    void calculateAlphaServoAngle();
    void calculateServoPWM();

    void debugPrintOuts();

    Position baseHome;
    Position platformHome;

    // Servo information
    Adafruit_PWMServoDriver pwm;
    ServoArm servoArmArray[6];

    // Requested translational/rotational positions
    Position requestedPlatformPosition;
    Position requestedPlatformRotation;

    // Matrices 
    Position translationalMatrix;
    float rotationalMatrix[3][3];
};

#endif
