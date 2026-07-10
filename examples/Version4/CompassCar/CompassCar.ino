#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();
static Motor&             motor = robot.motor;
static Movement&           move = robot.move;

void setup() {
  robot.init();
}

// Set the speed of each motor
void motors(int LF_spd, int RF_spd, int RB_spd, int LB_spd) {
  motor.setSpeed(M1, LF_spd);
  motor.setSpeed(M2, RF_spd);
  motor.setSpeed(M3, RB_spd);
  motor.setSpeed(M4, LB_spd);
}

void loop() {
  // Read compass heading
  uint16_t heading = robot.compass.read();
  int rotationSpeed = 100; // Set rotation speed

  // robot is facing forward (350° to 10°), stop motors
  if (350 <= heading || heading <= 10) {
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