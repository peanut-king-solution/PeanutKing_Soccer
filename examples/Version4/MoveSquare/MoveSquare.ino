/**
 * A simple example to test the robot's movement in a square pattern.
 * The robot will move forward, right front, rightward, and then repeat the pattern.
 * It will also demonstrate the use of compass correction
 * 
 * Before running this example, you should ensure you have checked the following:
 *  - The robot's motors are properly connected and configured. (see Motor.ino example)
 *  - The robot's movement coordinate system is set up correctly. (see Movement.ino example)
 *  - The robot's compass is calibrated and functioning correctly (see Compass.ino example).
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

/**
 * Test the robot's movement in a square pattern.
 * `speed`    - Movement speed (0-255)
 * `duration` - Duration (in milliseconds) for each side of the square
 */
void testMoveSqure(int speed, int duration) {
  for (int i = 0; i < 4; i++) {
    int timer = millis(); // Record the start time
    while (millis() - timer < duration) {
      robot.move(i * 90, speed);
    }
  }
}

void setup()
{
  robot.init();

  // You should ensure that the robot's motors are properly configured 
  // and the movement coordinate system is set up correctly before running this example.
  // Check the Motor.ino and Movement.ino examples for guidance.

  // disable out-of-bounds prevention
  robot.outBoundPreventEnabled = false;
}

int speed = 80;       // Movement speed (0-255)
int duration = 1500; // 1 second delay for each movement

void loop()
{ 
  // Disable both compass correction
  robot.compassCorrectEnabled  = false;
  // Move the robot in a square pattern with increased speed
  testMoveSqure(speed * 1.5, duration);

  // Enable compass correction for movement
  robot.compassCorrectEnabled  = true;
  // Move the robot in a square pattern with compass correction
  testMoveSqure(speed, duration);

  // Disable compass correction for rotation
  robot.compassCorrectEnabled  = false;
  // Rotate the robot clockwise at speed 100
  robot.move(0, 0, 100);
  delay(duration);
  // Rotate the robot counterclockwise at speed 100
  robot.move(0, 0, -100);
  delay(duration);

  robot.motorStopAll(); // Stop all motors
  delay(duration);      // Wait before repeating the loop 
}