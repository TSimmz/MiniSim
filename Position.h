#ifndef _POSITION_H_
#define _POSITION_H_

#include <math.h>
#include "Defines.h"

class Position
{
    
public:
    Position();
    Position(float x, float y, float z);
    ~Position();

    //Overloaded equal operator needed here;
    
private:
    float x_coord;
    float y_coord;
    float z_coord;
    
public:
    void setXCoord(float x);
    void setYCoord(float y);
    void setZCoord(float z);

    float getXCoord();
    float getYCoord();
    float getZCoord();

    void addPostionToThis(Position p);
    void subPositionFromThis(Position p);
    
    float posMagnitudeSquared();
    
};

#endif //_POSITION_H_
