//================================================================
// Defines.h
//================================================================

#ifndef __DEFINE_H__
#define __DEFINE_H__

#include <math.h>
#include <Adafruit_PWMServoDriver.h>

static const int   SERVO_NUM    = 6;
static const int   SERVO_MIN    = 150;
static const int   SERVO_MAX    = 600;
static const int   SERVO_FREQ   = 60;

static const float BASE_DIST    = 122.1;
static const float PLAT_DIST    = 140.5;
static const float SERVO_LEN    = 40.0;
static const float SERVO_DIST   = 162.8;
static const float LEG_LEN      = 182.0;

static const float Z_HOME       = 144.0;

typedef enum __servo_id__
{
  SERVO1,
  SERVO2,
  SERVO3,
  SERVO4,
  SERVO5,
  SERVO6,
} Servo_ID;

#endif
