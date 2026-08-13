/**
 * A simple example to test the robot's movement.
 * The robot will move forward, right front, rightward, and then repeat the pattern.
 *
 * Before running this example, you should ensure you have checked the following:
 *  - The robot's motors are properly connected and configured. (see Motor.ino example)
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup()
{
  robot.init();

  robot.compassCorrectEnabled  = false;  // Disable compass correction for movement
  robot.outBoundPreventEnabled = false;  // Disable out-of-bounds prevention for movement

  /*
  You should check the movement direction of the robot without any configuration first,
  to check the coordinate system of the robot's movement,
  then configure the motor converter to adjust the robot's move direction.

  For example, if the robot moves in the order of

    leftward (270°) -> left back (225°) -> backward (180°)
    (Correct should be forward (0°) -> right front (45°) -> rightward (90°))

    -> means the robot's movement direction is wrong configured
    (rotated by -90° and becomes counter-clockwise)

  then you should adjust the movement direction,
  configure by uncommenting the below line of code
  */
  // robot.movementCoordinateReset();
  // robot.movementCoordinateRotate(90, CW);
  // robot.movementCoordinateFlip();
}

int speed = 80;       // Movement speed (0-255)
int duration = 1000;  // Duration for each movement in milliseconds

void loop()
{ 
  // if configured correctly, the robot should move in the order of:
  // forward (0°) -> right front (45°) -> rightward (90°)
  robot.move(0, speed);
  delay(duration);
  robot.move(45, speed);
  delay(duration);
  robot.move(90, speed);
  delay(duration);

  robot.motorStopAll(); // Stop all motors
  delay(duration);      // Wait before repeating the loop
}