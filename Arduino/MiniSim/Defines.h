//================================================================
// Defines.h
//================================================================

#ifndef __DEFINE_H__
#define __DEFINE_H__

#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ClickEncoder.h>
#include <TimerOne.h>

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#define METRIC 
#define DEBUG

//#define BASE_DEBUG
//#define PLAT_DEBUG
//#define TRANS_DEBUG
//#define ROT_DEBUG
//#define Q_DEBUG
//#define LEG_DEBUG
//#define A_CALC_DEBUG
//#define ANGLE_DEBUG
#define PWM_DEBUG

static const int   SERVO_NUM    = 6;
static const int   SERVO_MIN    = 150;
static const int   SERVO_MAX    = 600;
static const int   SERVO_FREQ   = 60;
static const int   ANGLE_MIN    = -90;
static const int   ANGLE_MAX    = 90;

static const int   ROT_MIN      = -45;
static const int   ROT_MAX      =  45;

#ifdef METRIC
static const float BASE_DIST    = 122.1;
static const float PLAT_DIST    = 140.5;
static const float SERVO_LEN    = 40.0;
static const float SERVO_DIST   = 162.8;
static const float LEG_LEN      = 182.0;
static const float Z_HOME       = 144.0;
#endif

#ifndef METRIC
static const float BASE_DIST    = 4.81;
static const float PLAT_DIST    = 5.53;
static const float SERVO_LEN    = 1.57;
static const float SERVO_DIST   = 6.41;
static const float LEG_LEN      = 7.17;
static const float Z_HOME       = 4.72;
#endif

const int FLAT = map(0, ANGLE_MIN, ANGLE_MAX, SERVO_MIN, SERVO_MAX);

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

typedef enum __movement__
{
  SURGE,
  SWAY,
  HEAVE,
  ROLL,
  PITCH,
  YAW
} Movement;

inline float map2(float x, float in_min, float in_max, float out_min, float out_max) 
{
  return ((x - in_min) * (out_max - out_min) + out_min * (in_max - in_min)) / (in_max - in_min);
}

#endif
