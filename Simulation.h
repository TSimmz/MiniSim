//================================================================
// Simulation.h
//================================================================

#ifndef __SIM_H__
#define __SIM_H__

#include "Defines.h"
#include "Position.h"

class Simulation
{
public:
    Simulation();
    ~Simulation();

    int init();
    int run();

private:
    int servoArmInitialization();
    int controllerInitialization();

    void updatePlatformPosition();

    void calculateTranslationalMatrix();
    void calculateRotationalMatrix();

    void calculatePlatformAnchor();
    void calculateLegLength();

    int getAlphaAngle();

    //Servo servo_list[6];

    Position requestedPlatformPosition;
    Position requestedPlatformRotation;

    float rotationalMatrix[3][3];

    Position translationalMatrix;
    Position platformHome;

};

#endif
