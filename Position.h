//================================================================
// Position.h
//================================================================

#ifndef _POSITION_H_
#define _POSITION_H_

#include "Defines.h"

class Position
{
    
public:
    Position();
    Position(float x, float y, float z);

    Position & operator= (const Position & other);

    float x_coord;
    float y_coord;
    float z_coord;

    Position addPositionToThis(Position p);
    Position subPositionFromThis(Position p);
    
    float posMagnitudeSquared();
};

#endif //_POSITION_H_
