//================================================================
// Position.cpp
//================================================================

#include "Position.h"

//======================================================
// Constructor - Empty
//======================================================
Position::Position()
{
    x_coord = 0.0;
    y_coord = 0.0;
    z_coord = 0.0;
}

//======================================================
// Constructor - Set all variables
//======================================================
Position::Position(float x, float y, float z)
{
    x_coord = x;
    y_coord = y;
    z_coord = z;
}

//======================================================
// Overloaded equality operator
//======================================================
Position & Position::operator=(const Position& other)
{
    this->x_coord = other.x_coord;
    this->y_coord = other.y_coord;
    this->z_coord = other.z_coord;
}

//======================================================
// Adds two positions and returns the new position
//======================================================
Position Position::addPositionToThis(Position p)
{
    Position newPos = Position();

    newPos.x_coord = this->x_coord + p.x_coord;
    newPos.y_coord = this->y_coord + p.y_coord;
    newPos.z_coord = this->z_coord + p.z_coord;
    
    return newPos;
}

//======================================================
// Subtracts two positions and returns the new position
//======================================================
Position Position::subPositionFromThis(Position p)
{
    Position newPos = Position();

    newPos.x_coord = this->x_coord - p.x_coord;
    newPos.y_coord = this->y_coord - p.y_coord;
    newPos.z_coord = this->z_coord - p.z_coord;
    
    return newPos;
}

//======================================================
// Returns the magnitude squared of the position
//======================================================
float Position::posMagnitudeSquared()
{
    return pow(x_coord, 2.0) + pow(y_coord, 2.0) + pow(z_coord, 2.0);
}
