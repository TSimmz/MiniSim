#include "Position.h"

Position::Position()
{
    x_coord = 0;
    y_coord = 0;
    z_coord = 0;
}

Position::Position(int x, int y, int z)
{
    x_coord = x;
    y_coord = y;
    z_coord = z;
}

void Position::setXCoord(int x)
{
    x_coord = x;
}

void Position::setYCoord(int y)
{
    y_coord = y;
}

void Position::setZCoord(int z)
{
    z_coord = z;
}

int Position::getXCoord()
{
    return x_coord;
}

int Position::getYCoord()
{
    return y_coord;
}

int Position::getZCoord()
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

int Position::posMagnitudeSquared()
{
    return pow(x_coord, 2) + pow(y_coord, 2) + pow(z_coord, 2);
}