/*******************************************************************************
Worcester Polytechnic Institute
RBE 522 - Continuum Robotics
Professor Fichera
A Term 2022

Educational Continuum Robot
*******************************************************************************/

#include <Dynamixel2Arduino.h>
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;

// Constant Definitions
const uint8_t DXL1_ID = 1;
const uint8_t DXL2_ID = 2;
const uint8_t DXL3_ID = 3;
const uint8_t DEFAULT_SPEED = 50;
const float DXL_PROTOCOL_VERSION = 2.0;

// Variable Definitions
float currPos1 = 0.0;
float currPos2 = 0.0;
float currPos3 = 0.0;

// Dynamixel Configuration
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

/*******************************************************************************
Function Definitions
*******************************************************************************/

/* Function: torqueEnable
 *  
 *  Turns on torque for all 3 motors.
 *  
 *  Params:
 *    None
 *    
 *  Output:
 *    None
*/
void torqueEnable() {
  dxl.torqueOn(DXL1_ID);
  dxl.torqueOn(DXL2_ID);
  dxl.torqueOn(DXL3_ID);
}

/* Function: torqueDisable
 *  
 *  Turns off torque for all 3 motors.
 *  
 *  Params:
 *    None
 *    
 *  Output:
 *    None
*/
void torqueDisable() {
  dxl.torqueOff(DXL1_ID);
  dxl.torqueOff(DXL2_ID);
  dxl.torqueOff(DXL3_ID);
}

/* Function: updateMotorPositions
 *  
 *  Updates variables with current positions of all 3 motors.
 *  
 *  Params:
 *    None
 *    
 *  Output:
 *    None
*/
void updateMotorPositions() {
  currPos1 = dxl.getPresentPosition(DXL1_ID, UNIT_DEGREE);
  currPos2 = dxl.getPresentPosition(DXL2_ID, UNIT_DEGREE);
  currPos3 = dxl.getPresentPosition(DXL3_ID, UNIT_DEGREE);
}

/* Function: positionMode
 *  
 *  Sets operating mode of all motors to POSITION.
 *  
 *  Params:
 *    None
 *    
 *  Output:
 *    None
*/
void positionMode() {
  
  // Disable motor torque
  torqueDisable();

  // Set operating mode to POSITION for each motor
  dxl.setOperatingMode(DXL1_ID, OP_POSITION);
  dxl.setOperatingMode(DXL2_ID, OP_POSITION);
  dxl.setOperatingMode(DXL3_ID, OP_POSITION);

  // Enable motor torque
  torqueEnable();
}

/* Function: velocityMode
 *  
 *  Sets operating mode of all motors to VELOCITY.
 *  
 *  Params:
 *    None
 *    
 *  Output:
 *    None
*/
void velocityMode() {
  
  // Disable motor torque
  torqueDisable();

  // Set operating mode to VELOCITY for each motor
  dxl.setOperatingMode(DXL1_ID, OP_VELOCITY);
  dxl.setOperatingMode(DXL2_ID, OP_VELOCITY);
  dxl.setOperatingMode(DXL3_ID, OP_VELOCITY);

  // Enable motor torque
  torqueEnable();

}

/* Function: currentMode
 *  
 *  Sets operating mode of all motors to CURRENT.
 *  
 *  Params:
 *    None
 *    
 *  Output:
 *    None
*/
void currentMode() {
  
  // Disable motor torque
  torqueDisable();

  // Set operating mode to VELOCITY for each motor
  dxl.setOperatingMode(DXL1_ID, OP_CURRENT);
  dxl.setOperatingMode(DXL2_ID, OP_CURRENT);
  dxl.setOperatingMode(DXL3_ID, OP_CURRENT);

  // Enable motor torque
  torqueEnable();

}

/* Function: setProfileVelocities
 *  
 *  Sets profile velocities to desired values. This controls the maximum speed of the motor when moving to a position setpoint.
 *  
 *  Params:
 *    int setVel1 - profile velocity value for motor 1
 *    int setVel2 - profile velocity value for motor 2
 *    int setVel3 - profile velocity value for motor 3
 *    
 *  Output:
 *    None
*/
void setProfileVelocities(int setVel1, int setVel2, int setVel3) {

  // Disable motor torque
  torqueDisable();
  
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL1_ID, setVel1);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL1_ID, setVel2);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL1_ID, setVel3);

  // Enable motor torque
  torqueEnable();
}

/* Function: setMotorPositions
 *  
 *  Sets motor target positions for use in POSITION mode.
 *  
 *  Params:
 *    float pos1 - target position for motor 1 in DEGREES
 *    float pos2 - target position for motor 2 in DEGREES
 *    float pos3 - target position for motor 3 in DEGREES
 *    
 *  Output:
 *    None
*/
void setMotorPositions(float pos1, float pos2, float pos3) {

  // Set motors to position mode
  positionMode();

  // Set desired goal positions
  dxl.setGoalPosition(DXL1_ID, pos1, UNIT_DEGREE);
  dxl.setGoalPosition(DXL2_ID, pos2, UNIT_DEGREE);
  dxl.setGoalPosition(DXL3_ID, pos3, UNIT_DEGREE);
  
}

/* Function: setMotorVelocities
 *  
 *  Sets motor target velocities for use in VELOCITY mode.
 *  
 *  Params:
 *    float vel1 - target velocity for motor 1 in RPM
 *    float vel2 - target velocity for motor 2 in RPM
 *    float vel3 - target velocity for motor 3 in RPM
 *    
 *  Output:
 *    None
*/
void setMotorVelocities(float vel1, float vel2, float vel3) {

  // Set motors to velocity mode
  velocityMode();

  // Set desired velocities
  dxl.setGoalVelocity(DXL1_ID, vel1, UNIT_RPM);
  dxl.setGoalVelocity(DXL2_ID, vel2, UNIT_RPM);
  dxl.setGoalVelocity(DXL3_ID, vel3, UNIT_RPM);
  
}

/* Function: moveToPoseInSecs
 *  
 *  Sets motor target positions for use in POSITION mode.
 *  
 *  Params:
 *    float goalPos1 - target position for motor 1 in DEGREES
 *    float goalPos2 - target position for motor 2 in DEGREES
 *    float goalPos3 - target position for motor 3 in DEGREES
 *    float seconds - time to complete movement in SECONDS
 *    
 *  Output:
 *    None
*/
void moveToPoseInSecs(float goalPos1, float goalPos2, float goalPos3, float seconds) {

  // Get starting motor positions
  float startPos1 = dxl.getPresentPosition(DXL1_ID, UNIT_DEGREE);
  float startPos2 = dxl.getPresentPosition(DXL2_ID, UNIT_DEGREE);
  float startPos3 = dxl.getPresentPosition(DXL3_ID, UNIT_DEGREE);

  // Calculate motor speeds
  float speed1 = abs(goalPos1 - startPos1)/(6*seconds);
  float speed2 = abs(goalPos2 - startPos2)/(6*seconds);
  float speed3 = abs(goalPos3 - startPos3)/(6*seconds);
  
  // Set profile velocities
  setProfileVelocities(speed1, speed2, speed3);

  // Command motor positions
  setMotorPositions(goalPos1, goalPos2, goalPos3);

  // Wait for completion
  delay(seconds*1000);

}

/* Function: homeConfig
 *  
 *  Command robot to move to home configuration.
 *  
 *  Params:
 *    None
 *    
 *  Output:
 *    None
*/
void homeConfig() {

  // Set motors to CURRENT mode
  currentMode();

//  // Get current readings
//  float curr1 = dxl.getPresentCurrent(DXL1_ID, UNIT_PERCENT);
//  float curr2 = dxl.getPresentCurrent(DXL2_ID, UNIT_PERCENT);
//  float curr3 = dxl.getPresentCurrent(DXL3_ID, UNIT_PERCENT);
//
//  while (curr1 < 10.0 && curr2 < 10.0 && curr3 < 10.0) {
//
//    dxl.setGoalCurrent(DXL1_ID, -15.0, UNIT_PERCENT);
//    dxl.setGoalCurrent(DXL2_ID, -15.0, UNIT_PERCENT);
//    dxl.setGoalCurrent(DXL3_ID, -15.0, UNIT_PERCENT);
//
//    curr1 = dxl.getPresentCurrent(DXL1_ID, UNIT_PERCENT);
//    curr2 = dxl.getPresentCurrent(DXL2_ID, UNIT_PERCENT);
//    curr3 = dxl.getPresentCurrent(DXL3_ID, UNIT_PERCENT);
//  }

  dxl.setGoalCurrent(DXL1_ID, -15.0, UNIT_PERCENT);
  dxl.setGoalCurrent(DXL2_ID, -15.0, UNIT_PERCENT);
  dxl.setGoalCurrent(DXL3_ID, -15.0, UNIT_PERCENT);

  delay(2000);
  
  // Set motors to POSITION mode
  positionMode();

  // Get motor positions in home configuration
  updateMotorPositions();

  // Set goal positions to maintain current pose
  setMotorPositions(currPos1, currPos2, currPos3);
  
}

/*******************************************************************************
Setup and Loop
*******************************************************************************/

void setup() {

  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL1_ID);
  dxl.ping(DXL2_ID);
  dxl.ping(DXL3_ID);

  // Limit the maximum velocity in POSITION mode
  setProfileVelocities(DEFAULT_SPEED, DEFAULT_SPEED, DEFAULT_SPEED);

  // Move robot to home position
  delay(2000);
  homeConfig();
  DEBUG_SERIAL.println("Done moving to home position.");
  
}

void loop() {
 
}
