#include "Position.h"

Position::Position()
{
    x_coord = 0.0;
    y_coord = 0.0;
    z_coord = 0.0;
}

Position::Position(float x, float y, float z)
{
    x_coord = x;
    y_coord = y;
    z_coord = z;
}

void Position::setXCoord(float x)
{
    x_coord = x;
}

void Position::setYCoord(float y)
{
    y_coord = y;
}

void Position::setZCoord(float z)
{
    z_coord = z;
}

float Position::getXCoord()
{
    return x_coord;
}

float Position::getYCoord()
{
    return y_coord;
}

float Position::getZCoord()
{
    return z_coord;
}

void Position::addPostionToThis(Position p)
{
    x_coord += p.x_coord;
    y_coord += p.y_coord;
    z_coord += p.z_coord;
}

void Position::subPositionFromThis(Position p)
{
    x_coord -= p.x_coord;
    y_coord -= p.y_coord;
    z_coord -= p.z_coord;
}

float Position::posMagnitudeSquared()
{
    return pow(x_coord, 2.0) + pow(y_coord, 2.0) + pow(z_coord, 2.0);
}
