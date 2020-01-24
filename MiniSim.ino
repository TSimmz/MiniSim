//================================================================
// MiniSim.ino
//================================================================

#include "Defines.h"
#include "Simulation.h"

#define PRESSED ClickEncoder::Open

#define PITCH_STICK 0
#define ROLL_STICK  1
#define YAW_STICK   2

Simulation sim = Simulation();

ClickEncoder *encoder;
ClickEncoder::Button button;

int movementArray[6] = {0};
int prevMovementArray[6] = {0};

int16_t last, value;

//======================================================
// 
//======================================================
void timerIsr() {
  encoder->service();
}

//======================================================
// 
//======================================================
void setup() 
{
  encoder = new ClickEncoder(A1, A0, A2);

  Serial.begin(38400);
  
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr); 
  
  Serial.println("Initializing Simulator...");
  sim.init();
}

//======================================================
// 
//======================================================
void loop() 
{
  value += encoder->getValue();
  movementArray[YAW] = value = constrain(value, ROT_MIN, ROT_MAX);

  if (value != last)
  {
    #ifdef DEBUG
    Serial.print("Value: "); Serial.println(movementArray[YAW]);
    #endif

    last = value;
  }
  
  button = encoder->getButton();
  if (button != PRESSED)
  {
    value = 0;
  }

  if ( prevMovementArray[SURGE] != movementArray[SURGE] ||
       prevMovementArray[SWAY]  != movementArray[SWAY]  ||
       prevMovementArray[HEAVE] != movementArray[HEAVE] ||
       prevMovementArray[ROLL]  != movementArray[ROLL]  ||
       prevMovementArray[PITCH] != movementArray[PITCH] ||
       prevMovementArray[YAW]   != movementArray[YAW]   )
  
  {
    sim.run(movementArray);
    copyMovementPositions();
  }
}

//======================================================
// 
//======================================================
void copyMovementPositions()
{
  prevMovementArray[SURGE] = movementArray[SURGE];
  prevMovementArray[SWAY]  = movementArray[SWAY];
  prevMovementArray[HEAVE] = movementArray[HEAVE];
  prevMovementArray[ROLL]  = movementArray[ROLL];
  prevMovementArray[PITCH] = movementArray[PITCH];
  prevMovementArray[YAW]   = movementArray[YAW];
}
