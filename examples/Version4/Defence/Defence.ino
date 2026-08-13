/**
 * PeanutKing Soccer V4 - Defence Example
 * This example demonstrates a simple defensive strategy for the PeanutKing Soccer V4 robot.
 * The robot uses its ultrasonic sensors to maintain a defensive position on the field.
 * It will move to the middle of the field if it is too far from its own goal,
 * and will move back to defend if it is already in the middle. 
 * If it is too close to its own goal, it will move forward to avoid scoring an own goal.
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();

  // movement configuration: enable compass correction and disable out-of-bounds prevention
  robot.compassCorrectEnabled = true;
  robot.outBoundPreventEnabled = false;

  // You should configure motor mapping if needed (refer to Motor.ino example)
  // robot.motorConfiguration(M1, M2, M3, M4);

  // You should also configure the movement coordinate system if needed (refer to Movement.ino example)
  // robot.movementCoordinateReset(); // Reset to default coordinate system
  // robot.movementCoordinateRotate(90, CW); // Rotate coordinate system by 90 degrees clockwise
  // robot.movementCoordinateFlip(); // Flip coordinate system direction (CW <-> CCW)
  
  // You can also change the coordinate system of the compass if needed (refer to Compass.ino example)
  // robot.compassCoordinateReset(); // Reset to default coordinate system
  // robot.compassCoordinateRotate(90, CW); // Rotate coordinate system by 90 degrees clockwise
  // robot.compassCoordinateFlip(); // Flip coordinate system direction (CW <-> CCW)

  // You should also configure the ultrasonic sensor mapping if needed (refer to Ultrasound.ino example)
  // robot.ultrasoundConfiguration(U1, U2, U3, U4);
}

int defenseSpeed = 100;  // Speed for defensive movements

int backDistMax  = 300;  // Maximum distance from own goal
int backDistMin  = 100;  // Minimum distance from own goal

/*
You should get the data from the left and right ultrasonic sensors
when the robot is in the middle of the field

For example, if the robot is in the middle of the field, 
and the left and right distances is about 650 mm,
then you can set the leftDistMax and rightDistMax to 600 mm
to give some buffer to avoid the robot moving left and right 
and cannot stay in the middle of the field.
*/
int rightDistMax = 600;  // Maximum distance from right side
int leftDistMax  = 600;  // Maximum distance from left side

void loop() {
  // Read ultrasonic distances
  int frontDist = robot.ultrasoundGetDist(Front);  // Front distance
  int rightDist = robot.ultrasoundGetDist(Right);  // Right distance
  int backDist  = robot.ultrasoundGetDist(Back);    // Back distance
  int leftDist  = robot.ultrasoundGetDist(Left);    // Left distance
  
  // Too far from the own goal, move to middle of the field first and then move back to defend
  if (backDist > backDistMax) {
    // Too close to the left side, move right
    if (leftDist < leftDistMax) {  // left distance need to be larger than leftDistMax to be in the middle of the field
      robot.move(90, defenseSpeed);  // Move right
    }
    // Too close to the right side, move left
    else if (rightDist < rightDistMax) {  // right distance need to be larger than rightDistMax to be in the middle of the field
      robot.move(270, defenseSpeed); // Move left
    }
    // Already in the middle, move back to defend
    else {
      robot.move(180, defenseSpeed); // Move back
    }
  }
  // Too close to the own goal, move forward to avoid own goal
  else if (backDist < backDistMin) {
    robot.move(0, defenseSpeed);   // Move forward
  }
  // Safe distance from the own goal, stay in place and defend
  else {
    robot.motorStopAll();  // Stop all motors
  }
}