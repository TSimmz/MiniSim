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
void Simulation::init()
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
void Simulation::run(int * moveArray)
{
  requestedPlatformPosition.x_coord = moveArray[SURGE]; //platformHome.x_coord;
  requestedPlatformPosition.y_coord = moveArray[SWAY]; //platformHome.y_coord;
  requestedPlatformPosition.z_coord = moveArray[HEAVE]; //platformHome.z_coord;
  
  requestedPlatformRotation.x_coord = radians(moveArray[ROLL]); 
  requestedPlatformRotation.y_coord = radians(moveArray[PITCH]); 
  requestedPlatformRotation.z_coord = radians(moveArray[YAW]); 

  calculatePlatformPosition();
  updateServoPosition();
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
  servoArmArray[SERVO1].betaAngleToXAxis = radians(150);
  servoArmArray[SERVO1].mirrorServo      = false;

  runServoTest(SERVO1);
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

  runServoTest(SERVO2);
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

  runServoTest(SERVO3);
  Serial.println("Servo 3 Initialized!");
  
  servoArmArray[SERVO4].servoID          = SERVO4;
  servoArmArray[SERVO4].minPulseWidth    = SERVO_MIN;
  servoArmArray[SERVO4].maxPulseWidth    = SERVO_MAX;
  servoArmArray[SERVO4].baseAngle        = 188.0;
  servoArmArray[SERVO4].baseDistance     = BASE_DIST;
  servoArmArray[SERVO4].platformAngle    = 153.1;
  servoArmArray[SERVO4].platformDistance = PLAT_DIST;
  servoArmArray[SERVO4].betaAngleToXAxis = radians(-150);
  servoArmArray[SERVO4].mirrorServo      = true;

  runServoTest(SERVO4);
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

  runServoTest(SERVO5);
  Serial.println("Servo 5 Initialized!");
  
  servoArmArray[SERVO6].servoID          = SERVO6;
  servoArmArray[SERVO6].minPulseWidth    = SERVO_MIN;
  servoArmArray[SERVO6].maxPulseWidth    = SERVO_MAX;
  servoArmArray[SERVO6].baseAngle        = 68.0;
  servoArmArray[SERVO6].baseDistance     = BASE_DIST;
  servoArmArray[SERVO6].platformAngle    = 33.1;
  servoArmArray[SERVO6].platformDistance = PLAT_DIST;
  servoArmArray[SERVO6].betaAngleToXAxis = radians(-30);
  servoArmArray[SERVO6].mirrorServo      = true;

  runServoTest(SERVO6);
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

//======================================================
// 
//======================================================
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

  #ifdef DEBUG
  debugPrintOuts();
  #endif
}

//======================================================
// Calculates the translational matrix of the platform
//======================================================
void Simulation::calculateTranslationalMatrix()
{
  translationalMatrix.addPositions(requestedPlatformPosition, platformHome);
}

//======================================================
// Calculates the rotational matrix of the platform
//======================================================
void Simulation::calculateRotationalMatrix()
{
  float psi;   
  float theta;
  float phi;

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
}

//======================================================
// Calculates the new position of each servo arms joint
//======================================================
void Simulation::calculatePlatformAnchor()
{
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
  }
}

//======================================================
// Calculates the new leg lengths
//======================================================
void Simulation::calculateLegLength()
{
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    servoArmArray[arm].lengthOfLeg_L.subtractPositions(servoArmArray[arm].platformAnchorPoint_Q, servoArmArray[arm].baseJoint);
  }

}

//======================================================
// Calculates the new servo angles
//======================================================
void Simulation::calculateAlphaServoAngle()
{
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    servoArmArray[arm].Lprime = 
      ((float)pow(servoArmArray[arm].lengthOfLeg_L.x_coord, 2.0) + 
       (float)pow(servoArmArray[arm].lengthOfLeg_L.y_coord, 2.0) + 
       (float)pow(servoArmArray[arm].lengthOfLeg_L.z_coord, 2.0))- 
      ((float)pow(LEG_LEN, 2.0) - (float)pow(SERVO_LEN, 2.0));
      
    servoArmArray[arm].Mprime = 2.0 * SERVO_LEN * (servoArmArray[arm].platformAnchorPoint_Q.z_coord - servoArmArray[arm].baseJoint.z_coord);
    
    servoArmArray[arm].Nprime = 2.0 * SERVO_LEN  * 
      ((cos(servoArmArray[arm].betaAngleToXAxis) * (servoArmArray[arm].platformAnchorPoint_Q.x_coord - servoArmArray[arm].baseJoint.x_coord)) +
       (sin(servoArmArray[arm].betaAngleToXAxis) * (servoArmArray[arm].platformAnchorPoint_Q.y_coord - servoArmArray[arm].baseJoint.y_coord)));

    servoArmArray[arm].alphaAngleToHorizontal =
      asin(servoArmArray[arm].Lprime / (sqrt((servoArmArray[arm].Mprime * servoArmArray[arm].Mprime) + (servoArmArray[arm].Nprime * servoArmArray[arm].Nprime)))) - 
      atan(servoArmArray[arm].Nprime / servoArmArray[arm].Mprime);
  }
}

//======================================================
// 
//======================================================
void Simulation::calculateServoPWM()
{
  float angle = 0.0;

  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    angle = degrees(servoArmArray[arm].alphaAngleToHorizontal);

    if (servoArmArray[arm].isMirrored())
    {
      servoArmArray[arm].currentPWM = (int)map2(angle, ANGLE_MIN, ANGLE_MAX, SERVO_MAX, SERVO_MIN);
    }
    else
    {
      servoArmArray[arm].currentPWM = (int)map2(angle, ANGLE_MIN, ANGLE_MAX, SERVO_MIN, SERVO_MAX);
    }
    
    angle = 0.0;
  }
}

//======================================================
// 
//======================================================
void Simulation::updateServoPosition()
{
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    pwm.setPWM(arm, 0, servoArmArray[arm].currentPWM);
    delay(10);    
  }
}

//======================================================
// 
//======================================================
void Simulation::debugPrintOuts()
{
  #ifdef BASE_DEBUG
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    Serial.print("B"); Serial.print(arm); Serial.print(": ["); 
    Serial.print(servoArmArray[arm].baseJoint.x_coord); Serial.print(", "); 
    Serial.print(servoArmArray[arm].baseJoint.y_coord); Serial.print(", ");
    Serial.print(servoArmArray[arm].baseJoint.z_coord); Serial.println("]");
  }
  Serial.println();
  #endif

  #ifdef PLAT_DEBUG
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    Serial.print("P"); Serial.print(arm); Serial.print(": ["); 
    Serial.print(servoArmArray[arm].platformJoint.x_coord); Serial.print(", "); 
    Serial.print(servoArmArray[arm].platformJoint.y_coord); Serial.print(", ");
    Serial.print(servoArmArray[arm].platformJoint.z_coord); Serial.println("]");
  }
  Serial.println();
  #endif
  
  #ifdef TRANS_DEBUG
  Serial.print(" T: [");
  Serial.print(translationalMatrix.x_coord); Serial.print(", "); 
  Serial.print(translationalMatrix.y_coord); Serial.print(", ");
  Serial.print(translationalMatrix.z_coord); Serial.println("]");
  #endif
  
  #ifdef ROT_DEBUG
  Serial.print(" R: [");
  Serial.print(rotationalMatrix[0][0]); Serial.print(", "); 
  Serial.print(rotationalMatrix[0][1]); Serial.print(", ");
  Serial.print(rotationalMatrix[0][2]); Serial.print("][");
  
  Serial.print(rotationalMatrix[1][0]); Serial.print(", "); 
  Serial.print(rotationalMatrix[1][1]); Serial.print(", ");
  Serial.print(rotationalMatrix[1][2]); Serial.print("][");
  
  Serial.print(rotationalMatrix[2][0]); Serial.print(", "); 
  Serial.print(rotationalMatrix[2][1]); Serial.print(", ");
  Serial.print(rotationalMatrix[2][2]); Serial.println("]");
  #endif
  
  #ifdef Q_DEBUG
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    Serial.print("Q"); Serial.print(arm); Serial.print(": ["); 
    Serial.print(servoArmArray[arm].platformAnchorPoint_Q.x_coord); Serial.print(", "); 
    Serial.print(servoArmArray[arm].platformAnchorPoint_Q.y_coord); Serial.print(", ");
    Serial.print(servoArmArray[arm].platformAnchorPoint_Q.z_coord); Serial.println("]");
  }
  Serial.println();
  #endif
  
  #ifdef LEG_DEBUG
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    Serial.print("L"); Serial.print(arm); Serial.print(": ["); 
    Serial.print(servoArmArray[arm].lengthOfLeg_L.x_coord); Serial.print(", "); 
    Serial.print(servoArmArray[arm].lengthOfLeg_L.y_coord); Serial.print(", ");
    Serial.print(servoArmArray[arm].lengthOfLeg_L.z_coord); Serial.println("]");
  }
  Serial.println();
  #endif
  
  #ifdef ANGLE_DEBUG
  Serial.print(" A: [");
  for (int arm = 0; arm < SERVO_NUM; arm++) {
    Serial.print(servoArmArray[arm].alphaAngleToHorizontal); Serial.print(", ");
  }
  Serial.println("]");
  
  #endif

  #ifdef A_CALC_DEBUG
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    Serial.print(arm);
    Serial.print(": L | "); 
    Serial.print(servoArmArray[arm].Lprime); Serial.print(", "); 
    Serial.print("  M | ");
    Serial.print(servoArmArray[arm].Mprime); Serial.print(", ");
    Serial.print("  N | ");
    Serial.print(servoArmArray[arm].Nprime); Serial.println("]");
  }
  Serial.println();
  #endif
    
  #ifdef PWM_DEBUG
  Serial.print(" [ ");
  for (int arm = 0; arm < SERVO_NUM; arm++) {
    Serial.print(servoArmArray[arm].currentPWM); Serial.print(", ");
  }
  Serial.println("]");
  #endif

}
