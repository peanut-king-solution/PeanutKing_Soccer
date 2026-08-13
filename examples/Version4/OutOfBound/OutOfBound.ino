// Simple square movement test with white line detection
// Robot moves in square: Forward -> Right -> Back -> Left -> repeat
// If white line detected, move in opposite direction

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();

  // You should configure the motor mapping if needed (refer to Motor.ino example)
  // robot.motorConfiguration(M1, M2, M3, M4);

  // You should also configure the movement coordinate system if needed (refer to Movement.ino example)
  // robot.movementCoordinateReset(); // Reset to default coordinate system
  // robot.movementCoordinateRotate(90, CW); // Rotate coordinate system by 90 degrees clockwise
  // robot.movementCoordinateFlip(); // Flip coordinate system direction (CW <-> CCW

  // You should also configure the color sensor mapping if needed (refer to ColorSensor.ino example)
  // robot.colorSensorConfiguration(CL1, CL2, CL3, CL4);

  // You can change the coordinate system of the compass if needed (refer to Compass.ino example)
  // robot.compassCoordinateReset(); // Reset to default coordinate system
  // robot.compassCoordinateRotate(90, CW); // Rotate coordinate system by 90 degrees clockwise
  // robot.compassCoordinateFlip(); // Flip coordinate system direction (CW <-> CCW)

  // Enable compass correction, disable out-of-bounds prevention
  // as this example just demonstrates simple out of bounds prevention and square movement
  robot.compassCorrectEnabled = true;
  robot.outBoundPreventEnabled = false;
}

// Parameters
const int MOVE_SPD  = 80;  // Movement speed
const int BOUND_SPD = 200; // Speed when white line detected
const int TIME  = 2000;    // Time per side (ms)

int angle = -1;        // Current direction: 0, 90, 180, 270
int speed = MOVE_SPD;  // current speed, either MOVE_SPD or BOUND_SPD
uint32_t timer = 0;    // Timer for movement

void loop() {
  // Check white lines (by physical position)
  bool whiteF = robot.isWhiteLine(Front);
  bool whiteR = robot.isWhiteLine(Right);
  bool whiteB = robot.isWhiteLine(Back);
  bool whiteL = robot.isWhiteLine(Left);

  speed = BOUND_SPD; // default to boundary speed
  if (whiteF)      angle = 180;  // Front detected -> move back
  else if (whiteR) angle = 270;  // Right detected -> move left
  else if (whiteB) angle = 0;    // Back detected  -> move forward
  else if (whiteL) angle = 90;   // Left detected  -> move right
  else {    // No white line detected
    speed = MOVE_SPD; // no white line detected, use normal speed
    // Update direction when time is up
    if (millis() - timer >= TIME) {
      angle = (angle + 90) % 360;  // Next direction
      timer = millis(); // Reset timer
    }
  }

  // Move in the determined direction and speed
  robot.move(angle, speed);
}
