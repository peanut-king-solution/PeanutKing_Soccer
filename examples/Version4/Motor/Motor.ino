#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

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
  // robot.motorConfiguration(M2, M1, M3, M4); // swap Left Front and Right Front

  /*
  You should also check the movement direction of the robot,
  to check the coordinate system of the robot's movement,
  then configure the motor converter to adjust the robot's move direction.

  For example, if the robot moves in the order of

    leftward (270°) -> left back (225°) -> backward (180°)
    (Correct should be forward (0°) -> right front (45°) -> rightward (90°))

    -> means the robot's movement direction is wrong configured
    (shifted by -90° and becomes counter-clockwise)

  then you should adjust the movement direction,
  configure by uncommenting the below line of code
  */
  // robot.movement.coordinateShift(90);
  // robot.movement.coordinateFlip();
}

void loop()
{

  int speed = 100;
  /*
  If configured correctly, the motors should activate in the order of

  Left Front -> Right Front -> Right Back -> Left Back (clockwise),
  */
  for (uint8_t i = 0; i < 4; i++) {
    // convert index to MotorPos enum
    MotorPos pos = (MotorPos)i;
    // activate the motor at the given position with the specified speed
    robot.motorSetSpeed(pos, speed);
    delay(1000);
    // stop the motor at the given position
    robot.motorStop(pos);
    delay(500);
  }

  /*
  if configured correctly, the robot should move

  forward (0°) -> right front (45°) -> rightward (90°)
  */
  robot.compassCorrectEnabled  = false;  // Disable compass correction for movement
  robot.outBoundPreventEnabled = false;  // Disable out-of-bounds prevention for movement
  robot.move(0, speed);
  delay(1000);
  robot.move(45, speed);
  delay(1000);
  robot.move(90, speed);
  delay(1000);
  robot.motorStopAll();
  delay(500);
}