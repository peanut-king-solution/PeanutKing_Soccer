// Simple square movement test with white line detection
// Robot moves in square: Forward -> Right -> Back -> Left -> repeat
// If white line detected, move in opposite direction

#include <PeanutKingSoccerV4.h>
PeanutKingSoccerV4 robot;

// Parameters
const int SPEED = 80;      // Movement speed
const int BOUND_SPD = 200; // Speed when white line detected
const int TIME  = 2000;    // Time per side (ms)

int angle = 0;  // Current direction: 0, 90, 180, 270
unsigned long timer = 0;  // Timer for movement

void setup() {
  robot.init();
  timer = millis(); // start timer
}

void loop() {
  // Check white lines
  bool whiteF = robot.colorSensor.isWhiteLine(CL1);
  bool whiteR = robot.colorSensor.isWhiteLine(CL2);
  bool whiteB = robot.colorSensor.isWhiteLine(CL3);
  bool whiteL = robot.colorSensor.isWhiteLine(CL4);

  // If white line detected, move in opposite direction
  if (whiteF || whiteR || whiteB || whiteL) {
    int oppositeDir = 0;
    if (whiteF) oppositeDir = 180;      // Front detected -> move back
    else if (whiteR) oppositeDir = 270; // Right detected -> move left
    else if (whiteB) oppositeDir = 0;   // Back detected  -> move forward
    else if (whiteL) oppositeDir = 90;  // Left detected  -> move right

    // Move in opposite direction
    robot.moveWithCorr(oppositeDir, BOUND_SPD);
  }
  // Update direction when times up
  else if (millis() - timer >= TIME) {
    angle = (angle + 90) % 360;  // Next direction
    timer = millis(); // Reset timer
  }
  // Move in square direction
  else {
    robot.moveWithCorr(angle, SPEED);
  }
}
