#include "Platform.h"

Platform::Platform()
{
  basePosition = Position(0, 0, 0);
  platformPosition = Position(0, 0, Z_HOME);
}

Platform::~Platform()
{

}
