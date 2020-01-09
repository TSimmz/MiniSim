#ifndef _POSITION_H_
#define _POSITION_H_

#include <math.h>

class Position
{
    
public:
    Position();
    Position(int x, int y, int z);
    ~Position();
    
private:
    int x_coord;
    int y_coord;
    int z_coord;
    
public:
    void setXCoord(int x);
    void setYCoord(int y);
    void setZCoord(int z);

    int getXCoord();
    int getYCoord();
    int getZCoord();

    void addPostionToThis(Position p);
    void subPositionFromThis(Position p);
    
    int posMagnitudeSquared();
    
};

#endif //_POSITION_H_