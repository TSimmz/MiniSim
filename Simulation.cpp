//================================================================
// Simulation.cpp
//================================================================

#include "Simulation.h"

//======================================================
// Constructor
//======================================================
Simulation::Simulation()
{
  pwm = Adafruit_PWMServoDriver();
  baseHome = Position(0, 0, 0);
  platformHome = Position(0, 0, Z_HOME);
}

//======================================================
// Initialize the simulation
//======================================================
int Simulation::init()
{
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  
  delay(1000); 

  servoArmInitialization();

  Serial.println("Running simulator...");
  
}

//======================================================
// Runs the simulation in a loop
//======================================================
int Simulation::run(int pitch, int roll, int yaw)
{
  //float pitchMapped = radians(map(pitch, 0, 1023, -45, 45));
  float pitchMapped = radians(pitch);

  #ifdef ALL_DEBUG
  Serial.print("Pitch Radians: "); Serial.println(pitchMapped);
  #endif

  //Serial.print("Pitch Deg: "); Serial.print(pitch);
  //Serial.print(" Pitch Rad: "); Serial.println(pitchMapped);
  
  //float rollMapped = radians(map(roll, 0, 1023, -45, 45)); 
  //float yawMapped = radians(map(yaw, 0, 1023, -45, 45)); 

  requestedPlatformPosition.x_coord = platformHome.x_coord;
  requestedPlatformPosition.y_coord = platformHome.y_coord;
  requestedPlatformPosition.z_coord = platformHome.z_coord + pitch;
  
  requestedPlatformRotation.x_coord = 0; //rollMapped;
  requestedPlatformRotation.y_coord = 0;
  requestedPlatformRotation.z_coord = 0; //yawMapped;

  calculatePlatformPosition();
  updatePlatformPosition();

  
}

//======================================================
// Initializes all the servos arms
//======================================================
int Simulation::servoArmInitialization()
{
  Serial.println("Initializing servos..");
  
  servoArmArray[SERVO1].servoID          = SERVO1;
  servoArmArray[SERVO1].minPulseWidth    = SERVO_MIN;
  servoArmArray[SERVO1].maxPulseWidth    = SERVO_MAX;
  servoArmArray[SERVO1].baseAngle        = 352.0;
  servoArmArray[SERVO1].baseDistance     = BASE_DIST;
  servoArmArray[SERVO1].platformAngle    = 26.9;
  servoArmArray[SERVO1].platformDistance = PLAT_DIST;
  servoArmArray[SERVO1].betaAngleToXAxis = radians(120);
  servoArmArray[SERVO1].mirrorServo      = false;

  //runServoTest(SERVO1);
  Serial.println("Servo 1 Initialized!");
  
  servoArmArray[SERVO2].servoID          = SERVO2;
  servoArmArray[SERVO2].minPulseWidth    = SERVO_MIN;
  servoArmArray[SERVO2].maxPulseWidth    = SERVO_MAX;
  servoArmArray[SERVO2].baseAngle        = 308.0;
  servoArmArray[SERVO2].baseDistance     = BASE_DIST;
  servoArmArray[SERVO2].platformAngle    = 273.1; 
  servoArmArray[SERVO2].platformDistance = PLAT_DIST;
  servoArmArray[SERVO2].betaAngleToXAxis = radians(90);
  servoArmArray[SERVO2].mirrorServo      = true;

  //runServoTest(SERVO2);
  Serial.println("Servo 2 Initialized!");

  servoArmArray[SERVO3].servoID          = SERVO3;
  servoArmArray[SERVO3].minPulseWidth    = SERVO_MIN;
  servoArmArray[SERVO3].maxPulseWidth    = SERVO_MAX;
  servoArmArray[SERVO3].baseAngle        = 232.0;
  servoArmArray[SERVO3].baseDistance     = BASE_DIST;
  servoArmArray[SERVO3].platformAngle    = 266.9; 
  servoArmArray[SERVO3].platformDistance = PLAT_DIST;
  servoArmArray[SERVO3].betaAngleToXAxis = radians(-90);
  servoArmArray[SERVO3].mirrorServo      = false;

  //runServoTest(SERVO3);
  Serial.println("Servo 3 Initialized!");
  
  servoArmArray[SERVO4].servoID          = SERVO4;
  servoArmArray[SERVO4].minPulseWidth    = SERVO_MIN;
  servoArmArray[SERVO4].maxPulseWidth    = SERVO_MAX;
  servoArmArray[SERVO4].baseAngle        = 188.0;
  servoArmArray[SERVO4].baseDistance     = BASE_DIST;
  servoArmArray[SERVO4].platformAngle    = 153.1;
  servoArmArray[SERVO4].platformDistance = PLAT_DIST;
  servoArmArray[SERVO4].betaAngleToXAxis = radians(-120);
  servoArmArray[SERVO4].mirrorServo      = true;

  //runServoTest(SERVO4);
  Serial.println("Servo 4 Initialized!");
  
  servoArmArray[SERVO5].servoID          = SERVO5;
  servoArmArray[SERVO5].minPulseWidth    = SERVO_MIN;
  servoArmArray[SERVO5].maxPulseWidth    = SERVO_MAX;
  servoArmArray[SERVO5].baseAngle        = 112.0; 
  servoArmArray[SERVO5].baseDistance     = BASE_DIST;
  servoArmArray[SERVO5].platformAngle    = 146.9; 
  servoArmArray[SERVO5].platformDistance = PLAT_DIST;
  servoArmArray[SERVO5].betaAngleToXAxis = radians(30);
  servoArmArray[SERVO5].mirrorServo      = false;

  //runServoTest(SERVO5);
  Serial.println("Servo 5 Initialized!");
  
  servoArmArray[SERVO6].servoID          = SERVO6;
  servoArmArray[SERVO6].minPulseWidth    = SERVO_MIN;
  servoArmArray[SERVO6].maxPulseWidth    = SERVO_MAX;
  servoArmArray[SERVO6].baseAngle        = 68.0;
  servoArmArray[SERVO6].baseDistance     = BASE_DIST;
  servoArmArray[SERVO6].platformAngle    = 33.1;
  servoArmArray[SERVO6].platformDistance = PLAT_DIST;
  servoArmArray[SERVO6].betaAngleToXAxis = radians(-60);
  servoArmArray[SERVO6].mirrorServo      = true;

  //runServoTest(SERVO6);
  Serial.println("Servo 6 Initialized!");

  Serial.println("Calculating servo arm joint positions.."); 
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    servoArmArray[arm].calculateBaseJointPosition();
    servoArmArray[arm].calculatePlatformJointPosition();
    servoArmArray[arm].calculateHomeAngle();
  }

  Serial.println("Servo Initialization Complete!");
}

void Simulation::runServoTest(int servoID)
{
  pwm.setPWM(servoID, 0, SERVO_MIN);
  delay(1000);
  pwm.setPWM(servoID, 0, SERVO_MAX);
  delay(1000);
  pwm.setPWM(servoID, 0, FLAT);
}

//======================================================
// Calculate all the platform variables and set position
//======================================================
void Simulation::calculatePlatformPosition()
{
  calculateTranslationalMatrix();
  calculateRotationalMatrix();
  calculatePlatformAnchor();
  calculateLegLength();
  calculateAlphaServoAngle();
  calculateServoPWM();

//  #ifndef ALL_DEBUG
//  Serial.println();
//  delay(DEFAULT_DELAY);
//  #endif
}

//======================================================
// Calculates the translational matrix of the platform
//======================================================
void Simulation::calculateTranslationalMatrix()
{
  translationalMatrix = requestedPlatformPosition.addPositionToThis(platformHome);

  #ifdef TRANS_DEBUG
  Serial.println("Trans Matrix------------------");
  Serial.print("["); 
  Serial.print(translationalMatrix.x_coord); Serial.print(", "); 
  Serial.print(translationalMatrix.y_coord); Serial.print(", ");
  Serial.print(translationalMatrix.z_coord); Serial.println("]");
  
//  #ifndef ALL_DEBUG
//  Serial.print("\n");
//  delay(DEFAULT_DELAY);
//  #endif
  
  #endif
}

//======================================================
// Calculates the rotational matrix of the platform
//======================================================
void Simulation::calculateRotationalMatrix()
{
  float psi;   // rotation about z-axis (yaw) in radians
  float theta; // rotation about y-axis (pitch) in radians
  float phi;   // rotation about x-axis (roll) in radians

  psi   = requestedPlatformRotation.z_coord;
  theta = requestedPlatformRotation.y_coord;
  phi   = requestedPlatformRotation.x_coord;

  
  rotationalMatrix[0][0] = cos(psi)*cos(theta); 
  rotationalMatrix[0][1] = (-1*sin(psi)*cos(phi)) + (cos(psi)*sin(theta)*sin(phi));
  rotationalMatrix[0][2] = (sin(psi)*sin(phi)) + (cos(psi)*sin(theta)*cos(phi));
 
  rotationalMatrix[1][0] = sin(psi)*cos(theta);
  rotationalMatrix[1][1] = (cos(psi)*cos(phi)) + (sin(psi)*sin(theta)*sin(phi));
  rotationalMatrix[1][2] = (-1*cos(psi)*sin(phi))+(sin(psi)*sin(theta)*cos(phi));

  rotationalMatrix[2][0] = -1*sin(theta);
  rotationalMatrix[2][1] = cos(theta)*sin(psi);
  rotationalMatrix[2][2] = cos(theta)*cos(phi);

  #ifdef ROT_DEBUG
  Serial.println("R Matrix----------------------");
  Serial.print("["); 
  Serial.print(rotationalMatrix[0][0]); Serial.print(", "); 
  Serial.print(rotationalMatrix[0][1]); Serial.print(", ");
  Serial.print(rotationalMatrix[0][2]); Serial.println("]");
  Serial.print("["); 
  Serial.print(rotationalMatrix[1][0]); Serial.print(", "); 
  Serial.print(rotationalMatrix[1][1]); Serial.print(", ");
  Serial.print(rotationalMatrix[1][2]); Serial.println("]");
  Serial.print("["); 
  Serial.print(rotationalMatrix[2][0]); Serial.print(", "); 
  Serial.print(rotationalMatrix[2][1]); Serial.print(", ");
  Serial.print(rotationalMatrix[2][2]); Serial.println("]");

//  #ifndef ALL_DEBUG
//  Serial.print("\n");
//  delay(DEFAULT_DELAY);
//  #endif
 
  #endif
}

//======================================================
// Calculates the new position of each servo arms joint
//======================================================
void Simulation::calculatePlatformAnchor()
{
  #ifdef Q_DEBUG
  Serial.println("Q Matrix----------------------");
  #endif
  
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    servoArmArray[arm].platformAnchorPoint_Q.x_coord = translationalMatrix.x_coord + 
        ( rotationalMatrix[0][0] * servoArmArray[arm].platformJoint.x_coord +
          rotationalMatrix[0][1] * servoArmArray[arm].platformJoint.y_coord +
          rotationalMatrix[0][2] * servoArmArray[arm].platformJoint.z_coord );

    servoArmArray[arm].platformAnchorPoint_Q.y_coord = translationalMatrix.y_coord + 
        ( rotationalMatrix[1][0] * servoArmArray[arm].platformJoint.x_coord +
          rotationalMatrix[1][1] * servoArmArray[arm].platformJoint.y_coord +
          rotationalMatrix[1][2] * servoArmArray[arm].platformJoint.z_coord );

    servoArmArray[arm].platformAnchorPoint_Q.z_coord = translationalMatrix.z_coord + 
        ( rotationalMatrix[2][0] * servoArmArray[arm].platformJoint.x_coord +
          rotationalMatrix[2][1] * servoArmArray[arm].platformJoint.y_coord +
          rotationalMatrix[2][2] * servoArmArray[arm].platformJoint.z_coord );

    #ifdef Q_DEBUG
    Serial.print(arm);
    Serial.print(": ["); 
    Serial.print(servoArmArray[arm].platformAnchorPoint_Q.x_coord); Serial.print(", "); 
    Serial.print(servoArmArray[arm].platformAnchorPoint_Q.y_coord); Serial.print(", ");
    Serial.print(servoArmArray[arm].platformAnchorPoint_Q.z_coord); Serial.println("]");
    #endif
  }

//  #ifndef ALL_DEBUG
//  Serial.print("\n");
//  delay(DEFAULT_DELAY);
//  #endif
}

//======================================================
// Calculates the new leg lengths
//======================================================
void Simulation::calculateLegLength()
{
  #ifdef LEG_DEBUG
  Serial.println("Leg Matrix--------------------");

  #endif
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    servoArmArray[arm].lengthOfLeg_L = servoArmArray[arm].platformAnchorPoint_Q.subPositionFromThis(servoArmArray[arm].baseJoint);
    
    #ifdef LEG_DEBUG
    Serial.print(arm);
    Serial.print(": ["); 
    Serial.print(servoArmArray[arm].lengthOfLeg_L.x_coord); Serial.print(", "); 
    Serial.print(servoArmArray[arm].lengthOfLeg_L.y_coord); Serial.print(", ");
    Serial.print(servoArmArray[arm].lengthOfLeg_L.z_coord); Serial.println("]");
    #endif
  }

//  #ifndef ALL_DEBUG
//  Serial.println();
//  delay(DEFAULT_DELAY);
//  #endif
}

//======================================================
// Calculates the new servo angles
//======================================================
void Simulation::calculateAlphaServoAngle()
{
  float Aprime = 0.0;
  float Lprime = 0.0;
  float Mprime = 0.0;
  float Nprime = 0.0;

  #ifdef ANGLE_DEBUG
  Serial.println("Angle Matrix-----------------");
  Serial.print("[");
  #endif
  
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    Lprime = servoArmArray[arm].lengthOfLeg_L.posMagnitudeSquared() - (float)(LEG_LEN * LEG_LEN - SERVO_LEN * SERVO_LEN);
    Mprime = 2.0 * SERVO_LEN * (servoArmArray[arm].platformAnchorPoint_Q.z_coord - servoArmArray[arm].baseJoint.z_coord);
    Nprime = 2.0 * SERVO_LEN * (cos(servoArmArray[arm].betaAngleToXAxis) * (servoArmArray[arm].platformAnchorPoint_Q.x_coord - servoArmArray[arm].baseJoint.x_coord) +
                                sin(servoArmArray[arm].betaAngleToXAxis) * (servoArmArray[arm].platformAnchorPoint_Q.y_coord - servoArmArray[arm].baseJoint.y_coord) );

    Aprime = asin(Lprime / (sqrt(Mprime * Mprime + Nprime * Nprime))) - atan(Nprime / Mprime);

    servoArmArray[arm].alphaAngleToHorizontal = Aprime;
    
    #ifdef ANGLE_DEBUG
    Serial.print(Aprime); Serial.print(", ");
    #endif
  }

  #ifdef ANGLE_DEBUG
  Serial.println("]");
  #endif

//  #ifndef ALL_DEBUG
//  Serial.println();
//  delay(DEFAULT_DELAY);
//  #endif
}

void Simulation::calculateServoPWM()
{
  float deg_A = 0.0;

  #ifdef OUTPUT_DEBUG
  Serial.println("Output PWM----------------------");
  Serial.print("[");
  #endif
 
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    deg_A = degrees(servoArmArray[arm].alphaAngleToHorizontal);
    servoArmArray[arm].currentPWM = mapFloat(deg_A, ANGLE_MIN, ANGLE_MAX, SERVO_MIN, SERVO_MAX);

    #ifdef OUTPUT_DEBUG
    Serial.print(servoArmArray[arm].currentPWM); Serial.print(", ");
    #endif
    
    deg_A = 0.0;
  }

  #ifdef OUTPUT_DEBUG
  Serial.println("]");
  #endif
}

void Simulation::updatePlatformPosition()
{
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    pwm.setPWM(arm, 0, servoArmArray[arm].currentPWM);
    delay(10);    
  }
}
