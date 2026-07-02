#include <PeanutKingSoccerV4.h>
#include <PIDController.h>
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
  robot.motorsConfiguration(M2, M3, M4, M1); // configure the motor ports
  robot.motorConverter.config().flip();      // configure the motor converter to flip the direction
}

void loop() {
  // Move at 0 degrees with speed 80 and PID control based on compass reading
  robot.moveByAngleWithSmart(0, 80, robot.motorPID);
}

/* Extra function
 *  
 * robot.moveSmart( a,b,c,d );  // motor move using compass as reference 
 *                              // a is angular direction, normally use compass reading as reference
 *                              // b is speed, motor car moving speed 
 *                              // c is angle, 
 *                              // d is precision,
 */

  
