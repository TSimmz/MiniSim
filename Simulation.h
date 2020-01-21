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
    void runServoTest(int servoID);

    // Stewart Platform calculation functions
    void calculatePlatformPosition();
    void updatePlatformPosition();
    
    
    void calculateRotationalMatrix();
    
    void calculateTranslationalMatrix();
    void calculatePlatformAnchor();
    void calculateLegLength();
    
    void calculateAlphaServoAngle();
    
    void calculateServoPWM();

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
