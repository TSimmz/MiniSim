//================================================================
// MiniSim.ino
//================================================================

#include "Defines.h"
//#include "Simulation.h"

#include <Wire.h>
#include <ClickEncoder.h>
#include <TimerOne.h>

#define PITCH_STICK 0
#define ROLL_STICK  1
#define YAW_STICK   2

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//Simulation sim = Simulation();

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

  Serial.begin(9600);
  
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ); 
   
  delay(10);

  pwm.setPWM(1, 0, 150);
  delay(500);
  pwm.setPWM(1, 0, 600);

  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr); 
  
  Serial.println("Initializing Simulator...");
  //sim.init();
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

  pitchInput = constrain(value, -90, 90);
  //if (value != last)
  {
    //last = value;
    Serial.print("Rot Val: "); Serial.print(pitchInput);
    mappedVal = map(pitchInput, -90, 90, 150, 600);
    Serial.print("  MapVal: "); Serial.println(mappedVal);
    pwm.setPWM(0, 0, mappedVal);
  }

  
  
  ClickEncoder::Button b = encoder->getButton();
  if (b != ClickEncoder::Open)
  {
    value = 0;
  }

  //sim.run(pitchInput, rollInput, yawInput);

  //pitchInput = 0;
}
