//================================================================
// Defines.h
//================================================================

#ifndef __DEFINE_H__
#define __DEFINE_H__

#include "Position.h"

static const int SERVO_NUM = 6;
static const int SERVO_MIN = 150;
static const int SERVO_MAX = 600;
static const float BASE_DIST = 122.1;
static const float PLAT_DIST = 140.5;
static const float SERVO_LEN = 40.0;
static const float SERVO_DIST = 162.8;
static const float LEG_LEN = 182.0;
static const int Z_HOME = 144;

typedef struct __servo__
{
    int id;
    int pwm;
    int min_pulse_val;
    int max_pulse_val;

    int base_angle;
    int base_dist;

    Position B;

    int plat_angle;
    int plat_dist;
    
    Position P;

    bool inverse;

    int zero;

    float position;
    float alpha;
    float beta;

    Position Q;
    Position L;

    int min;
    int max;

    __servo__()
    {

    }


} Servo;

#endif
