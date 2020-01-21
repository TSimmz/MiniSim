//================================================================
// Defines.h
//================================================================

#ifndef __DEFINE_H__
#define __DEFINE_H__

#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define METRIC 

#define ALL_DEBUG  

#ifdef ALL_DEBUG
  //#define TRANS_DEBUG
  //#define ROT_DEBUG
  //#define Q_DEBUG
  //#define LEG_DEBUG
  #define ANGLE_DEBUG
  #define OUTPUT_DEBUG
#endif

//#define TRANS_DEBUG
//#define ROT_DEBUG
//#define Q_DEBUG
//#define LEG_DEBUG
//#define ANGLE_DEBUG
//#define OUTPUT_DEBUG

static const int   SERVO_NUM    = 6;
static const int   SERVO_MIN    = 150;
static const int   SERVO_MAX    = 630;
static const int   SERVO_FREQ   = 60;
static const int   ANGLE_MIN    = -90;
static const int   ANGLE_MAX    = 90;

#ifdef METRIC
static const float BASE_DIST    = 122.1;
static const float PLAT_DIST    = 140.5;
static const float SERVO_LEN    = 40.0;
static const float SERVO_DIST   = 162.8;
static const float LEG_LEN      = 182.0;
static const float Z_HOME       = 120.0;
#endif

#ifndef METRIC
static const float BASE_DIST    = 4.81;
static const float PLAT_DIST    = 5.53;
static const float SERVO_LEN    = 1.57;
static const float SERVO_DIST   = 6.41;
static const float LEG_LEN      = 7.17;
static const float Z_HOME       = 4.72;
#endif

const int FLAT = map(0, -90, 90, SERVO_MIN, SERVO_MAX);



static const int DEFAULT_DELAY  = 3000;

typedef enum __servo_id__
{
  SERVO1,
  SERVO2,
  SERVO3,
  SERVO4,
  SERVO5,
  SERVO6,
} Servo_ID;

inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif
