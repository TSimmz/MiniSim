//================================================================
// MiniSim.ino
//================================================================

#include "Defines.h"
#include "Simulation.h"

#include <Wire.h>

#define PITCH_STICK 0
#define ROLL_STICK  1
#define YAW_STICK   2

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Simulation sim = Simulation();

int pitchInput;
int rollInput;
int yawInput;

int mappedVal;

uint8_t bt;

void setup() 
{
  pitchInput = 0;
  rollInput = 0;
  yawInput = 0;

  Serial.begin(9600);
  
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ); 
   
  delay(10);
  
  Serial.println("Testing Analog Stick");
  //sim.init();
}

void loop() 
{
  for (int i = 0; i < 10; i++)
  {
    pitchInput += analogRead(PITCH_STICK);  
  }

  pitchInput /= 10;  

  for (int i = 0; i < 6; i++)
  {
    if (i % 2 == 0)
    {
      mappedVal = map(pitchInput, 0, 1023, SERVO_MAX, SERVO_MIN);
      pwm.setPWM(i, 0, mappedVal);    
    }
    else
    {
      mappedVal = map(pitchInput, 0, 1023, SERVO_MIN, SERVO_MAX);
      pwm.setPWM(i, 0, mappedVal);  
    }
  }
  
  Serial.print("PotVal: "); Serial.print(pitchInput);
  Serial.print("  MapVal: "); Serial.println(mappedVal);

  
  sim.run(pitchInput, rollInput, yawInput);

  pitchInput = 0;
}
