#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

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

  // allocate the motor port to the correct motor position if necessary
  // robot.motorConfiguration(M2, M1, M3, M4); // swap Left Front and Right Front

  // Adjust coordinate system
  // robot.movement.coordinateReset();   // Reset to default coordinate system
  // robot.movement.coordinateShift(0);  // Offset 0 degrees
  // robot.movement.coordinateFlip();    // Flip 180 degrees (CW <-> CCW)
}

int speed = 80;       // Movement speed (0-255)
int duration = 1500; // 1 second delay for each movement

void loop()
{ 
  // Move the robot in a square pattern without compass car
  // Disable both compass correction and out-of-bounds prevention
  robot.compassCorrectEnabled  = false;
  robot.outBoundPreventEnabled = false;
  // Move the robot in a square pattern with increased speed
  testMoveSqure(speed * 1.5, duration);

  // Move the robot in a square pattern with compass correction
  // Enable compass correction only (disable out-of-bounds prevention)
  robot.compassCorrectEnabled  = true;
  robot.outBoundPreventEnabled = false;
  testMoveSqure(speed, duration);

  // Rotate the robot clockwise and counterclockwise
  robot.compassCorrectEnabled  = false;  // Disable compass correction for rotation
  robot.outBoundPreventEnabled = false;  // Disable out-of-bounds prevention for rotation
  // Rotate the robot clockwise at speed 100
  robot.move(0, 0, 100);
  delay(duration);

  // Rotate the robot counterclockwise at speed 100
  robot.move(0, 0, -100);
  delay(duration);

  robot.motorStopAll(); // Stop all motors
  delay(2000);          // Wait for 2 seconds before repeating the loop 
}