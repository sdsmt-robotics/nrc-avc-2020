#include <Servo.h>
#include "LedStrip.h"
#include "PID.h"
#include "Encoder.h"
#include "Imu.h"

#define DEBUG true

//set up servo objects and default values found through testing
Servo turnR;
const int default_angle_R = 91;
const int default_lower_angle_R = 0;
const int default_upper_angle_R = 170;
Servo turnL;
const int default_angle_L = 91;
const int default_lower_angle_L = 0;
const int default_upper_angle_L = 170;
Servo motor;
const int default_angle_M = 90;
const int default_lower_angle_M = 0;
const int default_upper_angle_M = 170;

const float WHEEL_DIAMETER = 140; // Wheel diameter in mm
const float WHEEL_CIRC = WHEEL_DIAMETER * PI / 1000; // Wheel cir in m
const float RPM_TO_M_S = WHEEL_CIRC / 60;  // Conversion from RPM to M/s

const float DEG2RAD = PI / 180.0f;
const float FT2M = 3.28084;
const float ROT_2_M = WHEEL_CIRC / FT2M;

//Maximum power allowed
const float maxPower = 0.2;


// Controller update interval in milliseconds
const unsigned long UPDATE_INTERVAL = 10;

LedStrip ledStrip(8, 7, 6);

float kp = 0.08, ki = 0.1, kd = 0.0, N = 10;
PID speedController(kp, ki, kd, N, UPDATE_INTERVAL);

Encoder encoder(5, 4.5, 100);

Imu imu;


//================SETUP==============================
void setup() {
  // Initialize the LED Strip
  ledStrip.init();
  ledStrip.setBrightness(255);
  ledStrip.setColorRGB(255, 0, 0);
  
  Serial.begin(115200);
  if (DEBUG) Serial.println("Start.");
  
  //Initialize servos and main motor
  if (DEBUG) Serial.println("Initializeing motors...");
  speedController.setLimits(0, 0.30);
  initMotors();
  
  if (DEBUG) Serial.println("Initializeing encoder...");
  encoder.init();

  // Init IMU
  imu.init();
  
  ledStrip.setColorRGB(0, 255, 0);
  if (DEBUG) Serial.println("Ready!");
}


//================LOOP==============================
void loop() {
  float speedStraight = 5.0;
  float speedTurn = 3.0;

  delay(20*1000);

  // Around first pilon
  driveDist(21 * FT2M, speedStraight);
  turnAngle(-120, speedTurn);

  // Around the center pilon
  driveDist(33 * FT2M, speedStraight);
  turnAngle(60, speedTurn);
  driveDist(33 * FT2M, speedStraight);

  // Off the ramp
  turnAngle(-120, speedTurn);
  driveDist(42 * FT2M, speedStraight);

  // Through th eloop
  turnAngle(-95, speedTurn);
  driveDist(31 * FT2M, speedStraight);
  turnAngle(10, speedTurn);
  driveDist(31 * FT2M, speedStraight);
  
  // Through finish line
  turnAngle(-95, speedTurn);
  driveDist(30 * FT2M, speedStraight);

  while (true) {}
}



//================CAR Functions======================
/**
 * Drive a distance at a set speed.
 * 
 * @param dist - Distance to drive (feet).
 * @param speed - Speed to drive (fps).
 */
void driveDist(float dist, float speed) {
  // Convert  ft to meters
  dist *= FT2M;  
  speed *= FT2M;

  // Reset position and set driving to straight
  encoder.resetRevCount();
  setDriveAngle(0.0);
  
  // Drive until gone the appropriate distance
  while(getDriveDist() < dist) {
    setDriveSpeed(speed);
  }
  
  setDrivePower(0);
}

/**
 * Drive with wheels turned until reach a certain angle.
 */
void turnAngle(float angle, float speed) {
  float curAngle = 0;
  float wheelAngle = 25;
  bool neg = angle < 0;

  angle *= DEG2RAD;

  imu.reset();                       

  if (angle > 0) {
    setDriveAngle(wheelAngle);
  } else {
    setDriveAngle(-wheelAngle);
  }

  // TODO: This is very likely increadibly wrong.
  while ((neg && curAngle > angle) || (!neg && curAngle < angle)) {
    setDriveSpeed(speed);
    imu.update();
    curAngle = imu.Get_val();

    // Acount for the nromalizing to [0,2*PI]
    if (curAngle > PI) {
      curAngle -= 2 * PI;
    }
    
    delay(1);
    Serial.println(curAngle);
  }
  
  setDriveAngle(0.0);
  setDrivePower(0);
}

//Attempts to set the motor to a reasonable speed based on input.
//Returns false on invalid input although velocity will be set to max or min depending.
float power;
void setDriveSpeed(float speed) //m/s
{
  static unsigned long lastUpdate = millis();
  speedController.setTarget(speed);

  // Do a reset if it's been a while
  // if (millis() - lastUpdate > UPDATE_INTERVAL * 5) {
  //   speedController.reset();
  // }

  if (millis() - lastUpdate > UPDATE_INTERVAL) {
    encoder.estimateSpeed();
    
    //Get the power level from the speed controller
    power = speedController.calculateOutput(getDriveSpeed());

    //set the motor power
    setDrivePower(power);
    
    lastUpdate = millis();
  }
}

// Return the drive distance in meters since last reset
float getDriveDist() {
  return encoder.getRevCount() * WHEEL_CIRC;
}

/**
* Get the current speed of the car in m/s.
*/
float getDriveSpeed() {
  return encoder.getFilteredSpeed() * RPM_TO_M_S;
}

/**
* Set the power level for the main motor.
* 
* @param power - power level for the motor [0 to 1.0] 
*/
void setDrivePower(float power)
{
  int angM = 0;

  // Convert to a motor power level (0 to 180)
  angM = default_angle_M + power * (default_upper_angle_M - default_angle_M);
  
  // Constrain to valid
  angM = constrain(angM, default_angle_M, default_upper_angle_M);

  motor.write(angM);
}


/**
* Set the steering angle for the servos. Zero is measured from forward on vehicle.
* 
* @param dir - Angle to set steering [-25 to 25] 
*/
void setDriveAngle(int dir) // max -25(left) to 25(right)
{
  bool  valid = true;
  int   angR = 0;
  int   angL = 0;

  //check input value for validity and constrain it if it is not valid
  if ((dir > 25) || (dir < -25))
  {
    dir = constrain(dir, -25, 25);
  }

  //map input angle to corresponding servo angle
   angR = int(map(dir,-25,25,default_lower_angle_R,default_upper_angle_R));
   angL = int(map(dir,-25,25,default_lower_angle_L,default_upper_angle_L));

   //turn servos
   turnR.write(angR);
   turnL.write(angL);
}

/**
 * Initialize AVC servos and main motor.
 */
void initMotors() {
  // Steering servos
  turnR.attach(9);
  turnR.write(default_angle_R);
  turnL.attach(10);
  turnL.write(default_angle_L);

  // Main motor
  motor.attach(11);
  motor.write(90);

  // Wait for ESC to be OK with life
  delay(2500);
}

//================TESTING FUNCTIONS===================
/**
 * Test the various car functions.
 */
void testCarFunctions() {
  Serial.println("Running Servos...");
  delay(2000);
  setDriveAngle(-25);
  delay(800);
  setDriveAngle(25);
  delay(800);
  setDriveAngle(0);
  
  Serial.println("Running LEDs...");
  delay(1000);
  ledStrip.setBrightness(255);
  ledStrip.setColorRGB(255, 0, 0);
  
  Serial.println("Driving...");
  delay(500);
  setDrivePower(0.12);

  delay(3000);
  setDrivePower(0);
  while (true) {
    Serial.println("Done.");
    delay(1000);
  }
}

/**
 * Test outputing encoder speeds.
 */
void testEncoder() {
  static unsigned long lastUpdate = millis();


  if (millis() - lastUpdate > UPDATE_INTERVAL) {
    Serial.print("0, ");
    Serial.print(encoder.estimateSpeed());
    Serial.print(", ");
    Serial.println(encoder.getSpeed());
    //Serial.print(", ");

    //set the motor power
    setDrivePower(0.1);
    
    lastUpdate = millis();
  }
}

/**
 * Test changing the speed to a new value every three seconds.
 */
void testChangingSpeed() {
  const float speeds[] = {2, 3, 1};
  const int numSpeeds = 3;
  static int speedIndex = 0;
  static unsigned long lastSpeedChange = millis();
  static unsigned long lastPrint = millis();
  static unsigned long lastUpdate = millis();

  // Adjust PID vals
  // Format: Xd.dddd where X is the thing to change and d.dddd is the new value.
  if (Serial.available() > 0) {
    char change = Serial.read();
    Serial.setTimeout(10);
    float val = Serial.parseFloat();

    switch (change) {
      case 'p':
        kp = val;
      break;
      case 'i':
        ki = val;
      break;
      case 'd':
        kd = val;
      break;
      case 'n':
        N = val;
      break;
      
    }
    speedController.setConstants(kp, ki, kd, N, UPDATE_INTERVAL);
    delay(500);
  }

  //go to the next speed if past time
  if (millis() - lastSpeedChange > 3000) {
    speedIndex++;
    lastSpeedChange = millis();
    if (speedIndex >= numSpeeds) {
      speedIndex = 0;
    }
  }


  //Get the power level from the speed controller
  if (millis() - lastUpdate > UPDATE_INTERVAL) {
    //set the target speed
    setDriveSpeed(speeds[speedIndex]);
    lastUpdate = millis();
  }
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print("0,\t");
    Serial.print(speedController.getTarget()*100);
    Serial.print("0,\t");
    Serial.print(power*100);
    Serial.print(",\t");
    Serial.println(getDriveSpeed()*100);
    
    lastPrint = millis();
  }
}
