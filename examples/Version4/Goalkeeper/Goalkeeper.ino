#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();
int xonicleft = 0;
int xonicback = 0;
int angle = 0;

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
  angle = robot.compassRead();
  xonicleft = robot.ultrasonicRead(U1);
  xonicback = robot.ultrasonicRead(U2);
  
  if (angle>180 && angle<=270){
    motor(-90,-90,-90,-90);  
  }
  else if (angle > 270 && angle < 350){                   // If the car is facing left
    motor(-60,-60,-60,-60);                         // rotate clockwise
  }
  else if (angle > 10 && angle <= 90){              // If the car is facing right
    motor(60,60,60,60);        // rotate anti-clockwise
  }
  else if (angle > 90 && angle <= 180){              // If the car is facing right
    motor(90,90,90,90);
  }    
  else if (xonicleft < 700){                         // Use distance away from left wall as reference to relocate the car
    motor(-90,90,90,-90);                           // move right
  }
  else if (xonicleft > 950){                         // Use distance away from left wall as reference to relocate the car
    motor(90,-90,-90,90);                           // move left 
  }
  else if (xonicback > 400){                         // Use distance away from back wall as reference to relocate the car
    motor(-90,-90,90,90);                           // move back
  }
  else if (xonicback < 350){                         // Use distance away from back wall as reference to relocate the car
    motor(90,90,-90,-90);                           // move forward
  }
  else{
    robot.motorStop();                              // Stop moving
  }
}
