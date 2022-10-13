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

// Constant Definitions
const uint8_t DXL1_ID = 1; // ID of Dynamixel motor 1
const uint8_t DXL2_ID = 2; // ID of Dynamixel motor 2
const uint8_t DXL3_ID = 3; // ID of Dynamixel motor 3
const uint8_t DEFAULT_SPEED = 50; // Default profile velocity in POSITION mode [RPM]
const int DXL_DIR_PIN = -1; // Dynamixel direction pin, set to -1 because not needed with OpenRB-150 board
const float DXL_PROTOCOL_VERSION = 2.0; // Using Dynamixel protocol version 2.0
const float FORCE_LIMIT = 15.0; // Force limit during homing operation [percent]
const float HOMING_TIME = 2.0; // Time to pull on all tendons while homing [seconds]
const float MAX_CURVATURE = 10.0; // Curvature limit to prevent damage from overextension [1/meters]

// Variable Definitions
float initialRot1 = 0.0; // Initial rotation of motor 1 after homing
float initialRot2 = 0.0; // Initial rotation of motor 2 after homing
float initialRot3 = 0.0; // Initial rotation of motor 3 after homing

// Design Parameters
const float R_PRIME = 0.020; // Radial distance from spine to tendons [meters]
const float NOTCH_HEIGHT = 0.030; // Height of cutout sections [meters]
const float DISC_HEIGHT = 0.005; // Height of discs [meters]
const int NUM_NOTCHES = 6; // Number of notched sections
const float SPOOL_DIAMETER = 0.0515; // Diameter of cable spools [meters]
const float GAMMA = (2.0*PI)/3.0; // Angular distance between tendons [radians]

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

/* Function: angleToDeltaL
 *  
 *  Takes in rotation angle of motor and returns tendon dispalcement 
 *  
 *  Params:
 *    angle -> rotation angle of motor in DEGREES  
 *    
 *  Output:
 *    deltaL -> tendon displacement in METERS
*/
float angleToDeltaL(float angle) {

  // Calcualte tendon disaplcement from angle
  float deltaL = (angle/360)*PI*SPOOL_DIAMETER;

  // Return tendon displacement
  return deltaL;
}

/* Function: deltaLtoAngle
 *  
 *  Takes in rotation angle of motor and returns tendon dispalcement 
 *  
 *  Params:
 *    deltaL -> tendon displacement in METERS
 *    
 *  Output:
 *    angle -> rotation angle of motor in DEGREES
*/
float deltaLtoAngle(float deltaL) {

  // Calcualte tendon disaplcement from angle
  float angle = (deltaL*360)/(PI*SPOOL_DIAMETER);
  
  // Return tendon displacement
  return angle;
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

/* Function: recordInitRotations
 *  
 *  Sets the initial rotation variables to the current angular positions of the motors.
 *  
 *  Params:
 *    None
 *    
 *  Output:
 *    None
*/
void recordInitRotations() {

  // Set initial rotation angles for each of the three motors
  initialRot1 = dxl.getPresentPosition(DXL1_ID, UNIT_DEGREE);
  initialRot2 = dxl.getPresentPosition(DXL2_ID, UNIT_DEGREE);
  initialRot3 = dxl.getPresentPosition(DXL3_ID, UNIT_DEGREE);

}

/* Function: setProfileVelocities
 *  
 *  Sets profile velocities to desired values. This controls the maximum speed of the motor when moving to a position setpoint.
 *  
 *  Params:
 *    int setVel1 -> profile velocity value for motor 1
 *    int setVel2 -> profile velocity value for motor 2
 *    int setVel3 -> profile velocity value for motor 3
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
 *    float pos1 -> target position for motor 1 in DEGREES
 *    float pos2 -> target position for motor 2 in DEGREES
 *    float pos3 -> target position for motor 3 in DEGREES
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
 *    float vel1 -> target velocity for motor 1 in RPM
 *    float vel2 -> target velocity for motor 2 in RPM
 *    float vel3 -> target velocity for motor 3 in RPM
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

/* Function: stopMotors
 *  
 *  Sets the velocities of all motors to zero.
 *  
 *  Params:
 *    None
 *    
 *  Output:
 *    None
*/
void stopMotors() {
  
  // Set motor velocities to sero.
  setMotorVelocities(0.0, 0.0, 0.0);

}

/* Function: moveToPositionInSecs
 *  
 *  Sets motor target positions for use in POSITION mode.
 *  
 *  Params:
 *    float goalPos1 -> target position for motor 1 in DEGREES
 *    float goalPos2 -> target position for motor 2 in DEGREES
 *    float goalPos3 -> target position for motor 3 in DEGREES
 *    float seconds -> time to complete movement in SECONDS
 *    
 *  Output:
 *    None
*/
void moveToPositionInSecs(float goalPos1, float goalPos2, float goalPos3, float seconds) {

  // Set motors to POSITION mode
  positionMode();
  
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

/* Function: moveToPoseInSecs
 *  
 *  Sets motor target positions for use in VELOCITY mode. Then stops motors to hold position.
 *  
 *  Params:
 *    float rot1 -> desired rotation for motor 1 in DEGREES
 *    float rot2 -> desired rotation for motor 2 in DEGREES
 *    float rot3 -> desired rotation for motor 3 in DEGREES
 *    float seconds -> time to complete movement in SECONDS
 *    
 *  Output:
 *    None
*/
void moveToPoseSecs(float rot1, float rot2, float rot3, float seconds) {

  // Calculate motor speeds as a function of distance to target
  //  Converts from dps to RPM
  float motorSpeed1 = rot1/(6*seconds);
  float motorSpeed2 = rot2/(6*seconds);
  float motorSpeed3 = rot3/(6*seconds);

  // Command motor velocities to these values for given time
  setMotorVelocities(motorSpeed1, motorSpeed2, motorSpeed3);
  delay(seconds*1000);

  // Set motor velocities to zero
  stopMotors();

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

  // Pull on all tendons without exceeding preset current limit
  dxl.setGoalCurrent(DXL1_ID, -FORCE_LIMIT, UNIT_PERCENT);
  dxl.setGoalCurrent(DXL2_ID, -FORCE_LIMIT, UNIT_PERCENT);
  dxl.setGoalCurrent(DXL3_ID, -FORCE_LIMIT, UNIT_PERCENT);

  // Stop after specified homing time
  delay(HOMING_TIME*1000);

  // Store initial angular position of motors
  recordInitRotations();

  // Hold motors in their current positions
  stopMotors();
  
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
  delay(1000);
  homeConfig();
  DEBUG_SERIAL.println("Done moving to home position.");
  delay(1000);

}

void loop() {

  // Write loop code here
 
}
