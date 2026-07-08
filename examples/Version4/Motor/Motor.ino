#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();
static Motor&             motor = robot.motor;
static Movement&           move = robot.move;

void setup()
{
  robot.init();

  /* You should first run without config check which motor (LF/RF/LB/RB) is connected to which port (M1/M2/M3/M4),
  then allocate the motor port to the correct motor position in void setup() function. */

  // uncomment the below line to configure the motor ports
  // motor.mapMotors(M4, M2, M3, M1);

  /* You should also check the direction of each motor (LF/RF/LB/RB),
  then configure the motor converter to flip the direction in void setup() function. */

  // uncomment the below line to configure the motor converter to adjust the robot's move direction
  // move.converter.config().shift(90).flip();
}

void loop()
{
  // In this time, you should check which motor (LF/RF/RB/LB) is connected to which port (M1/M2/M3/M4),
  // then allocate the motor port to the correct motor position in void setup() function.
  // The below code will test motor from M1 to M4
  // if configured correctly, the motors should activate in the order of LF, RF, RB, LB (clockwise)  
  motor.SetSpeed(M1, 199);
  delay(1000);

  motor.StopAll();
  motor.SetSpeed(M2, 199);
  delay(1000);

  motor.StopAll();
  motor.SetSpeed(M3, 199);
  delay(1000);

  motor.StopAll();
  motor.SetSpeed(M4, 199);
  delay(1000);

  // Stop all motors
  motor.StopAll();
  delay(1000);

  // In this time, you should check the direction of each motor (LF/RF/LB/RB),
  // then configure the motor converter to flip the direction in void setup() function.
  move.byAngle(0, 100, 0); // Move forward at speed 100
  delay(1000);

  move.byAngle(45, 100, 0); // Move forward-right at speed 100
  delay(1000);

  move.byAngle(90, 100, 0); // Move right at speed 100
  delay(1000);

  // Stop all motors
  motor.StopAll();
  delay(1000);
}