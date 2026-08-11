/**
 * This example demonstrates how to make the PeanutKingSoccerV4 robot
 * rotate to face a specific direction using the compass module.
 * 
 * The robot will rotate clockwise or counter-clockwise based on its current heading
 * to align with the target direction (0°).
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

// Set the speed of each motor
void motors(int LF_spd, int RF_spd, int RB_spd, int LB_spd) {
  robot.motorSetSpeed(LeftFront,  LF_spd);
  robot.motorSetSpeed(RightFront, RF_spd);
  robot.motorSetSpeed(RightBack,  RB_spd);
  robot.motorSetSpeed(LeftBack,   LB_spd);
}

void setup() {
  robot.init();
}

void loop() {
  // Read compass heading
  uint16_t heading = robot.compassReadHeading();
  int rotationSpeed = 100;  // Set rotation speed
  int compassDeadZone = 10; // +- 10° deadzone

  // robot is facing forward (350° to 10°), stop motors
  if ((360 - compassDeadZone) <= heading || heading <= compassDeadZone) {
    motors(0, 0, 0, 0); // Stop
  }
  // robot is facing right (10° to 180°), rotate counter-clockwise
  else if (10 < heading && heading <= 180) {
    motors(-rotationSpeed, -rotationSpeed, -rotationSpeed, -rotationSpeed); // Rotate CCW
  }
  // robot is facing left (180° to 350°), rotate clockwise
  else if (180 < heading && heading < 350) {
    motors(rotationSpeed, rotationSpeed, rotationSpeed, rotationSpeed); // Rotate CW
  }
}