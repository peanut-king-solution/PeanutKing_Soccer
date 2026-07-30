#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup()
{
  robot.init();

  // allocate the motor port to the correct motor position if necessary
  // robot.motor.configuration(M2, M1, M3, M4); // swap Left Front and Right Front

  // configure the coordinate system if necessary
  // robot.move.converter.reset().shift(0);  // Reset and shift 0 degrees
  // robot.move.converter.flip();            // Flip 180 degrees (CW <-> CCW)
}

void loop()
{
  // Move the robot in a square pattern without compass car

  robot.move.byAngle(0, 100, 0); // Move forward at speed 100
  delay(2000);  // Wait for 2 seconds

  robot.move.byAngle(90, 100, 0); // Move right at speed 100
  delay(2000);  // Wait for 2 seconds

  robot.move.byAngle(180, 100, 0); // Move backward at speed 100
  delay(2000);  // Wait for 2 seconds

  robot.move.byAngle(270, 100, 0); // Move left at speed 100
  delay(2000);  // Wait for 2 seconds

  // Move the robot in a square pattern with compass correction

  // Move forward at speed 100 with compass correction
  robot.move.withCorr(0, 100, robot.compass.read());
  delay(2000);  // Wait for 2 seconds

  // Move right at speed 100 with compass correction
  robot.move.withCorr(90, 100, robot.compass.read());
  delay(2000);  // Wait for 2 seconds

  // Move backward at speed 100 with compass correction
  robot.move.withCorr(180, 100, robot.compass.read());
  delay(2000);  // Wait for 2 seconds

  // Move left at speed 100 with compass correction
  robot.move.withCorr(270, 100, robot.compass.read());
  delay(2000);  // Wait for 2 seconds

  // Rotate the robot clockwise and counterclockwise

  // Rotate the robot clockwise at speed 100
  robot.move.byAngle(0, 0, 100);
  delay(2000);  // Wait for 2 seconds

  // Rotate the robot counterclockwise at speed 100
  robot.move.byAngle(0, 0, -100);
  delay(2000);  // Wait for 2 seconds
}