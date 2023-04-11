#include <PeanutKingSoccerV3.h>
static PeanutKingSoccerV3 robot = PeanutKingSoccerV3();

void setup() {
  robot.init(); 
}

void loop() {
  robot.motorSet(0,-60); // motor 1, turn clockwise
  robot.motorSet(1,-60); // motor 2, turn clockwise
  robot.motorSet(2,-60); // motor 3, turn clockwise
  robot.motorSet(3,-60); // motor 4, turn clockwise

  delay(1000);

  robot.motorSet(0,60);  // motor 1, turn anti-clockwise
  robot.motorSet(1,60);  // motor 2, turn anti-clockwise
  robot.motorSet(2,60);  // motor 3, turn anti-clockwise
  robot.motorSet(3,60);  // motor 4, turn anti-clockwise

  delay(1000);

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

  
