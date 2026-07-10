#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();
static Motor&             motor = robot.motor;
static Movement&           move = robot.move;

void setup()
{
  robot.init();
}

void loop()
{
  // Move the robot in a square pattern without compass car

  move.byAngle(0, 100, 0); // Move forward at speed 100
  delay(2000);  // Wait for 2 seconds

  move.byAngle(90, 100, 0); // Move right at speed 100
  delay(2000);  // Wait for 2 seconds

  move.byAngle(180, 100, 0); // Move backward at speed 100
  delay(2000);  // Wait for 2 seconds

  move.byAngle(270, 100, 0); // Move left at speed 100
  delay(2000);  // Wait for 2 seconds

  // Move the robot in a square pattern with compass correction

  // Move forward at speed 100 with compass correction
  move.byAnglePID(0, 100, robot.compass.read());
  delay(2000);  // Wait for 2 seconds

  // Move right at speed 100 with compass correction
  move.byAnglePID(90, 100, robot.compass.read());
  delay(2000);  // Wait for 2 seconds

  // Move backward at speed 100 with compass correction
  move.byAnglePID(180, 100, robot.compass.read());
  delay(2000);  // Wait for 2 seconds

  // Move left at speed 100 with compass correction
  move.byAnglePID(270, 100, robot.compass.read());
  delay(2000);  // Wait for 2 seconds

  // Rotate the robot clockwise and counterclockwise

  // Rotate the robot clockwise at speed 100
  move.byAngle(0, 0, 100);
  delay(2000);  // Wait for 2 seconds

  // Rotate the robot counterclockwise at speed 100
  move.byAngle(0, 0, -100);
  delay(2000);  // Wait for 2 seconds
}