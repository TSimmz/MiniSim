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
void Position::addPositions(Position p1, Position p2)
{
    this->x_coord = p1.x_coord + p2.x_coord;
    this->y_coord = p1.y_coord + p2.y_coord;
    this->z_coord = p1.z_coord + p2.z_coord;
}

//======================================================
// Subtracts two positions and returns the new position
//======================================================
void Position::subtractPositions(Position p1, Position p2)
{
    this->x_coord = p1.x_coord - p2.x_coord;
    this->y_coord = p1.y_coord - p2.y_coord;
    this->z_coord = p1.z_coord - p2.z_coord;
}

//======================================================
// Returns the magnitude squared of the position
//======================================================
float Position::posMagnitudeSquared()
{
    return (pow(x_coord, 2.0) + pow(y_coord, 2.0) + pow(z_coord, 2.0));
}
