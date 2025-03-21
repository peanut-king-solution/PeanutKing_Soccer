#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();
int maxEye = 0;
int maxEyeReading = 0;

void motor(int a, int b, int c, int d){
  robot.motorSet(0,a);
  robot.motorSet(1,b);
  robot.motorSet(2,c);
  robot.motorSet(3,d);
}  

void setup() {
  robot.init();
}

void loop() {
  maxEye = robot.compoundMaxEye();             // Maximum Eye         - The infrared senor with highest reading
  maxEyeReading = robot.compoundMaxEyeVal();      // Maximum Eye Reading - The reading from the Maximum Eye
  
  if( maxEye == 0|| maxEye == 1 || maxEye == 2){
    motor(90,90,-90,-90);
  }
  else if( maxEye == 3 || maxEye == 4 || maxEye == 5){
    motor(-90,90,90,-90);
  }
  else if( maxEye == 6 || maxEye == 7 || maxEye == 8){
    motor(90,-90,-90,90);
  }
  else if( maxEye == 9 || maxEye == 10 || maxEye ==11){
    motor(-90,-90,90,90);
  }
}
