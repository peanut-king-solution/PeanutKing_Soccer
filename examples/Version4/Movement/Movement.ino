#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup()
{
  robot.init();

  // allocate the motor port to the correct motor position if necessary
  // robot.motorConfiguration(M2, M1, M3, M4); // swap Left Front and Right Front

  // configure the coordinate system if necessary
  // robot.move.converter.reset().shift(0);  // Reset and shift 0 degrees
  // robot.move.converter.flip();            // Flip 180 degrees (CW <-> CCW)
}

void loop()
{
  // Move the robot in a square pattern without compass car

  robot.moveByAngle(0, 100, 0); // Move forward at speed 100
  delay(2000);  // Wait for 2 seconds

  robot.moveByAngle(90, 100, 0); // Move right at speed 100
  delay(2000);  // Wait for 2 seconds

  robot.moveByAngle(180, 100, 0); // Move backward at speed 100
  delay(2000);  // Wait for 2 seconds

  robot.moveByAngle(270, 100, 0); // Move left at speed 100
  delay(2000);  // Wait for 2 seconds

  // Move the robot in a square pattern with compass correction

  // Move forward at speed 100 with compass correction
  robot.moveWithCorr(0, 100);
  delay(2000);  // Wait for 2 seconds

  // Move right at speed 100 with compass correction
  robot.moveWithCorr(90, 100);
  delay(2000);  // Wait for 2 seconds

  // Move backward at speed 100 with compass correction
  robot.moveWithCorr(180, 100);
  delay(2000);  // Wait for 2 seconds

  // Move left at speed 100 with compass correction
  robot.moveWithCorr(270, 100);
  delay(2000);  // Wait for 2 seconds

  // Rotate the robot clockwise and counterclockwise

  // Rotate the robot clockwise at speed 100
  robot.moveByAngle(0, 0, 100);
  delay(2000);  // Wait for 2 seconds

  // Rotate the robot counterclockwise at speed 100
  robot.moveByAngle(0, 0, -100);
  delay(2000);  // Wait for 2 seconds
}