#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init(); 
}

void loop() {
  robot.motorSet(0,-199); // motor 1, turn clockwise
  robot.motorSet(1,-199); // motor 2, turn clockwise
  robot.motorSet(2,-199); // motor 3, turn clockwise
  robot.motorSet(3,-199); // motor 4, turn clockwise

  delay(200);
  robot.motorStop();       // all motor stop

  robot.motorSet(0,199);  // motor 1, turn anti-clockwise
  robot.motorSet(1,199);  // motor 2, turn anti-clockwise
  robot.motorSet(2,199);  // motor 3, turn anti-clockwise
  robot.motorSet(3,199);  // motor 4, turn anti-clockwise

  delay(200);

  robot.motorStop();       // all motor stop

  delay(1000);
}


/* Extra function
 *  
 * robot.moveSmart( a,b,c,d );  // motor move using compass as reference 
 *                              // a is angular direction, normally use compass reading as reference
 *                              // b is speed, motor car moving speed 
 *                              // c is angle, 
 *                              // d is precision,
 */

  
