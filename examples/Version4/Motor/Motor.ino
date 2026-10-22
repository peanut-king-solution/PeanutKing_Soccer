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
  to check which motor (Right Front / Right Back / Left Back / Left Front)
  is connected to which port (M1 / M2 / M3 / M4),
  then allocate the motor port to the correct motor position.

  Default mapping: RF=M1, RB=M2, LB=M3, LF=M4

  For example, if the motors are activated in the order of

    Left Front -> Right Front -> Right Back -> Left Back
    (Left Front is activated first, then Right Front)
    -> means they are wrong configured

  then you should swap these two motors,
  configure by uncommenting the below line of code
  */
  // robot.motorConfiguration(M2, M3, M4, M1); // e.g. swap RF and LF ports
}

int speed = 100;
int duration = 1000;

void loop()
{
  /*
  If configured correctly, the motors should activate in the order of

  Right Front -> Right Back -> Left Back -> Left Front (clockwise),
  */
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

  robot.motorSetSpeed(LeftFront, speed);
  delay(duration);
  robot.motorStop(LeftFront);
  delay(duration);

  // Stop all motors
  robot.motorStopAll();
  delay(duration);
}