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
    p_pos = Position();
    p_rot = Position();
    T     = Position();
    H     = Position();

    rotMatrix = 
    {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    }

    servo_init()
    cntrl_init();
}

int Simulation::run()
{

}

int Simulation::servo_init()
{

}

int Simulation::cntrl_init()
{

}

int Simulation::movePlatform()
{

}

int Simulation::getTranslationalMatrix()
{

}

int Simulation::getRotationalMatrix()
{

}

int Simulation::getQandL()
{

}

int Simulation::getAlpha()
{

}