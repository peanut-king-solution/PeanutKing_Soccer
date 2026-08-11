// Simple square movement test with white line detection
// Robot moves in square: Forward -> Right -> Back -> Left -> repeat
// If white line detected, move in opposite direction

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

// Parameters
const int SPEED = 80;      // Movement speed
const int BOUND_SPD = 200; // Speed when white line detected
const int TIME  = 2000;    // Time per side (ms)

int angle = 0;  // Current direction: 0, 90, 180, 270
unsigned long timer = 0;  // Timer for movement

void setup() {
  robot.init();

  // You should ensure that the robot's motors are properly configured,
  // the movement coordinate system is set up correctly before running this example,
  // color sensors are properly configured
  // and the compass is calibrated and functioning correctly.
  // You can refer to the void init() in Motor.ino, Movement.ino, ColorSensor.ino, and Compass.ino examples for guidance.

  // Enable compass correction, disable out-of-bounds prevention
  // as this example just demonstrates simple out of bounds prevention and square movement
  robot.compassCorrectEnabled = true;
  robot.outBoundPreventEnabled = false;
  timer = millis(); // start timer
}

void loop() {
  // Check white lines (by physical position)
  bool whiteF = robot.isWhiteLine(Front);
  bool whiteR = robot.isWhiteLine(Right);
  bool whiteB = robot.isWhiteLine(Back);
  bool whiteL = robot.isWhiteLine(Left);

  // If white line detected, move in opposite direction
  if (whiteF || whiteR || whiteB || whiteL) {
    int oppositeDir = 0;
    if (whiteF) oppositeDir = 180;      // Front detected -> move back
    else if (whiteR) oppositeDir = 270; // Right detected -> move left
    else if (whiteB) oppositeDir = 0;   // Back detected  -> move forward
    else if (whiteL) oppositeDir = 90;  // Left detected  -> move right

    // Move in opposite direction
    robot.move(oppositeDir, BOUND_SPD);
  }
  // Update direction when times up
  else if (millis() - timer >= TIME) {
    angle = (angle + 90) % 360;  // Next direction
    timer = millis(); // Reset timer
  }
  // Move in square direction
  else {
    robot.move(angle, SPEED);
  }
}
