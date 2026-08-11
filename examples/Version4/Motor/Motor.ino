/**
 * A simple example to test the robot's motors.
 * The robot will activate each motor in sequence.
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup()
{
  robot.init();

  /*
  You should first run without configuration,
  to check which motor (Left Front / Right Front /Left Back / Right Back)
  is connected to which port (M1 / M2 / M3 / M4),
  then allocate the motor port to the correct motor position.

  For example, if the motors are activated in the order of

    Right Front -> Left Front -> Right Back -> Left Back
    (Right Front is activated first, then Left Front)
    -> means they are wrong configured

  then you should swap these two motors,
  configure by uncommenting the below line of code
  */
  robot.motorConfiguration(M4, M3, M1, M2); // swap Left Front and Right Front
}

int speed = 100;
int duration = 1000;

void loop()
{
  /*
  If configured correctly, the motors should activate in the order of

  Left Front -> Right Front -> Right Back -> Left Back (clockwise),
  */
  robot.motorSetSpeed(LeftFront, speed);
  delay(duration);
  robot.motorStop(LeftFront);
  delay(duration);

  robot.motorSetSpeed(RightFront, speed);
  delay(duration);
  robot.motorStop(RightFront);
  delay(duration);

  robot.motorSetSpeed(RightBack, speed);
  delay(duration);
  robot.motorStop(RightBack);
  delay(duration);

  robot.motorSetSpeed(LeftBack, speed);
  delay(duration);
  robot.motorStop(LeftBack);
  delay(duration);

  // Stop all motors
  robot.motorStopAll();
  delay(duration);
}