// Main entry point of simulation

#include "Simulation.h"
#include "Defines.h"

#define PITCH_STICK 0
#define ROLL_STICK  1
#define YAW_STICK   2

//Simulation sim = Simulation();

int pitchInput;
int rollInput;
int yawInput;

uint8_t bt;

void setup() 
{
  pitchInput = 0;
  rollInput = 0;
  yawInput = 0;

  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.println("Testing Analog Stick");
  //sim.init();
}

void loop() 
{ 
  if (Serial1.available())
  {
    bt = Serial1.read();
  }

  Serial.print("BT Read: "); Serial.println(bt);
  
  pitchInput = analogRead(PITCH_STICK);
  //Serial.print("Pitch Val: "); 
  //Serial.print(pitchInput);
  delay(50);
  rollInput  = analogRead(ROLL_STICK);
  //Serial.print("   Roll Val: "); 
  //Serial.println(rollInput);
  delay(50);
  yawInput   = analogRead(YAW_STICK);
  delay(50);
  
  //sim.run(pitchInput, rollInput, yawInput);
}
