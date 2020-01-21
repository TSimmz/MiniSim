//================================================================
// MiniSim.ino
//================================================================

#include "Defines.h"
#include "Simulation.h"

#include <Wire.h>
#include <ClickEncoder.h>
#include <TimerOne.h>

#define PITCH_STICK 0
#define ROLL_STICK  1
#define YAW_STICK   2

Simulation sim = Simulation();

ClickEncoder *encoder;
int16_t last, value;

int pitchInput;
int rollInput;
int yawInput;

int mappedVal;

uint8_t bt;

void timerIsr() {
  encoder->service();
}

void setup() 
{
  pitchInput = 0;
  rollInput = 0;
  yawInput = 0;

  encoder = new ClickEncoder(A1, A0, A2);

  Serial.begin(38400);
  
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr); 
  
  Serial.println("Initializing Simulator...");
  sim.init();
}

void loop() 
{
//  for (int i = 0; i < 10; i++)
//  {
//    pitchInput += analogRead(PITCH_STICK);  
//  }
//
//  pitchInput /= 10;  

  value += encoder->getValue();
  pitchInput = constrain(value, -60, 60);  
  mappedVal = map(pitchInput, -90, 90, SERVO_MIN, SERVO_MAX);
  
  if (value != last)
  {
    Serial.print("Rot Val: "); Serial.print(pitchInput);
    Serial.print("  MapVal: "); Serial.println(mappedVal);
  
    last = value;
  }

  ClickEncoder::Button b = encoder->getButton();
  if (b != ClickEncoder::Open)
  {
    value = 0;
  }

  sim.run(pitchInput, rollInput, yawInput);

  #ifdef ALL_DEBUG
  Serial.println();
  //delay(DEFAULT_DELAY);
  #endif
}
