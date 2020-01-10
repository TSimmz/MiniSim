#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include "Position.h"
#include "Defines.h"

class Platform
{
public:
  Platform();
  ~Platform();

  Position basePosition;
  Position platformPosition;
  
  int zeroValue;
};

#endif
