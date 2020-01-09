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
    int servo_init();
    int cntrl_init();

    int movePlatform();

    int getTranslationalMatrix();
    int getRotationalMatrix();

    int getQandL();

    int getAlpha();

private:
    Servo servo_list[6];

    Position p_pos;     // Requested position of platform
    Position p_rot;     // Requested rotation of platform

    float rotMatrix[3][3]; // Rotational matrix

    Position T;     // Translational matrix
    Position H;     // Center position of platform

};

#endif
