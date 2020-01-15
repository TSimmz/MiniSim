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
  delay(10); 
  
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

  //Serial.print("Pitch Deg: "); Serial.print(pitch);
  //Serial.print(" Pitch Rad: "); Serial.println(pitchMapped);
  
  //float rollMapped = radians(map(roll, 0, 1023, -45, 45)); 
  //float yawMapped = radians(map(yaw, 0, 1023, -45, 45)); 
  
  requestedPlatformRotation.y_coord = pitchMapped;
  requestedPlatformRotation.x_coord = 0; //rollMapped;
  requestedPlatformRotation.z_coord = 0; //yawMapped;

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
  servoArmArray[SERVO1].baseAngle        = 308.0;
  servoArmArray[SERVO1].baseDistance     = BASE_DIST;
  servoArmArray[SERVO1].platformAngle    = 273.1;
  servoArmArray[SERVO1].platformDistance = PLAT_DIST;
  servoArmArray[SERVO1].betaAngleToXAxis = radians(90);
  servoArmArray[SERVO1].mirrorServo      = false;

  Serial.println("Servo 1 Initialized!");
  delay(100);
  
  servoArmArray[SERVO2].servoID          = SERVO2;
  servoArmArray[SERVO2].minPulseWidth    = SERVO_MIN;
  servoArmArray[SERVO2].maxPulseWidth    = SERVO_MAX;
  servoArmArray[SERVO2].baseAngle        = 352.0;
  servoArmArray[SERVO2].baseDistance     = BASE_DIST;
  servoArmArray[SERVO2].platformAngle    = 26.9;
  servoArmArray[SERVO2].platformDistance = PLAT_DIST;
  servoArmArray[SERVO2].betaAngleToXAxis = radians(150);
  servoArmArray[SERVO2].mirrorServo      = true;

  Serial.println("Servo 2 Initialized!");
  delay(100);

  servoArmArray[SERVO3].servoID          = SERVO3;
  servoArmArray[SERVO3].minPulseWidth    = SERVO_MIN;
  servoArmArray[SERVO3].maxPulseWidth    = SERVO_MAX;
  servoArmArray[SERVO3].baseAngle        = 68.0;
  servoArmArray[SERVO3].baseDistance     = BASE_DIST;
  servoArmArray[SERVO3].platformAngle    = 33.1;
  servoArmArray[SERVO3].platformDistance = PLAT_DIST;
  servoArmArray[SERVO3].betaAngleToXAxis = radians(330);
  servoArmArray[SERVO3].mirrorServo      = false;

  Serial.println("Servo 3 Initialized!");
  delay(100);
  
  servoArmArray[SERVO4].servoID          = SERVO4;
  servoArmArray[SERVO4].minPulseWidth    = SERVO_MIN;
  servoArmArray[SERVO4].maxPulseWidth    = SERVO_MAX;
  servoArmArray[SERVO4].baseAngle        = 112.0;
  servoArmArray[SERVO4].baseDistance     = BASE_DIST;
  servoArmArray[SERVO4].platformAngle    = 146.9;
  servoArmArray[SERVO4].platformDistance = PLAT_DIST;
  servoArmArray[SERVO4].betaAngleToXAxis = radians(30);
  servoArmArray[SERVO4].mirrorServo      = true;

  Serial.println("Servo 4 Initialized!");
  delay(100);
  
  servoArmArray[SERVO5].servoID          = SERVO5;
  servoArmArray[SERVO5].minPulseWidth    = SERVO_MIN;
  servoArmArray[SERVO5].maxPulseWidth    = SERVO_MAX;
  servoArmArray[SERVO5].baseAngle        = 188.0;
  servoArmArray[SERVO5].baseDistance     = BASE_DIST;
  servoArmArray[SERVO5].platformAngle    = 153.1;
  servoArmArray[SERVO5].platformDistance = PLAT_DIST;
  servoArmArray[SERVO5].betaAngleToXAxis = radians(210);
  servoArmArray[SERVO5].mirrorServo      = false;

  Serial.println("Servo 5 Initialized!");
  delay(100);
  
  servoArmArray[SERVO6].servoID          = SERVO6;
  servoArmArray[SERVO6].minPulseWidth    = SERVO_MIN;
  servoArmArray[SERVO6].maxPulseWidth    = SERVO_MAX;
  servoArmArray[SERVO6].baseAngle        = 232.0;
  servoArmArray[SERVO6].baseDistance     = BASE_DIST;
  servoArmArray[SERVO6].platformAngle    = 266.9;
  servoArmArray[SERVO6].platformDistance = PLAT_DIST;
  servoArmArray[SERVO6].betaAngleToXAxis = radians(270);
  servoArmArray[SERVO6].mirrorServo      = true;

  Serial.println("Servo 6 Initialized!");
  delay(100);

  Serial.println("Calculating servo arm joint positions.."); 
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    servoArmArray[arm].calculateBaseJointPosition();
    servoArmArray[arm].calculatePlatformJointPosition();
  }

  Serial.println("Servo Initialization Complete!");
}

//======================================================
// Calculate all the platform variables and set position
//======================================================
void Simulation::updatePlatformPosition()
{
  int mappedAngle = 0;
  
  calculateTranslationalMatrix();
  calculateRotationalMatrix();
  calculatePlatformAnchor();
  calculateLegLength();
  calculateAlphaServoAngle();

  #ifdef OUTPUT_DEBUG
  Serial.print("[");
  #endif
  
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    if (arm % 2 != 0)
    {
      mappedAngle = map(degrees(servoArmArray[arm].alphaAngleToHorizontal),-180, 180, 150, 600);
    }
    else
    {
      mappedAngle = map(degrees(servoArmArray[arm].alphaAngleToHorizontal), -180 , 180, 600, 150);
    }

    #ifdef OUTPUT_DEBUG
    Serial.print(degrees(servoArmArray[arm].alphaAngleToHorizontal)); Serial.print(", ");
    #endif
    
    pwm.setPWM(arm, 0, mappedAngle);
  }

  #ifdef OUTPUT_DEBUG
  Serial.println("]\n");
  delay(500);
  #endif
}

//======================================================
// Calculates the translational matrix of the platform
//======================================================
void Simulation::calculateTranslationalMatrix()
{
  translationalMatrix = requestedPlatformPosition.addPositionToThis(platformHome);

  #ifdef TRANS_DEBUG
  Serial.print("["); 
  Serial.print(translationalMatrix.x_coord); Serial.print(", "); 
  Serial.print(translationalMatrix.y_coord); Serial.print(", ");
  Serial.print(translationalMatrix.z_coord); Serial.println("]\n");
  delay(500);
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
  rotationalMatrix[0][2] = (sin(psi)*sin(phi)) + (cos(psi)*cos(phi)*sin(theta));
 
  rotationalMatrix[1][0] = sin(psi)*cos(theta);
  rotationalMatrix[1][1] = (cos(psi)*cos(phi)) + (sin(psi)*sin(theta)*sin(phi));
  rotationalMatrix[1][2] = (-1*cos(psi)*sin(phi))+(sin(psi)*sin(theta)*cos(phi));

  rotationalMatrix[2][0] = sin(theta);
  rotationalMatrix[2][1] = -1*cos(theta)*sin(psi);
  rotationalMatrix[2][2] = cos(theta)*cos(phi);

  #ifdef ROT_DEBUG
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
  Serial.print(rotationalMatrix[2][2]); Serial.println("]\n");
  delay(500);
  #endif
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
    servoArmArray[arm].lengthOfLeg_L = servoArmArray[arm].platformAnchorPoint_Q.subPositionFromThis(servoArmArray[arm].baseJoint);
  }
}

//======================================================
// Calculates the new servo angles
//======================================================
void Simulation::calculateAlphaServoAngle()
{
  float Aknot = 0.0;
  float Lknot = 0.0;
  float Mknot = 0.0;
  float Nknot = 0.0;

  #ifdef ANGLE_DEBUG
  Serial.print("[");
  #endif
  
  for (int arm = 0; arm < SERVO_NUM; arm++)
  {
    Lknot = servoArmArray[arm].lengthOfLeg_L.posMagnitudeSquared() - (float)(sq(LEG_LEN) + sq(SERVO_LEN));
    Mknot = 2.0 * SERVO_LEN * servoArmArray[arm].lengthOfLeg_L.z_coord;
    Nknot = 2.0 * SERVO_LEN * ((cos(servoArmArray[arm].betaAngleToXAxis) * servoArmArray[arm].lengthOfLeg_L.x_coord) + 
                               (sin(servoArmArray[arm].betaAngleToXAxis) * servoArmArray[arm].lengthOfLeg_L.y_coord));

    Aknot = asin(Lknot / (pow(Mknot, 2) + pow(Nknot, 2))) - atan(Nknot / Mknot);

    servoArmArray[arm].alphaAngleToHorizontal = (servoArmArray[arm].isMirrored()) ? Aknot : (M_PI - Aknot);

    #ifdef ANGLE_DEBUG
    Serial.print(servoArmArray[arm].alphaAngleToHorizontal); Serial.print(", ");
    #endif
  }

  #ifdef ANGLE_DEBUG
  Serial.println("]\n");
  delay(500);
  #endif
}
