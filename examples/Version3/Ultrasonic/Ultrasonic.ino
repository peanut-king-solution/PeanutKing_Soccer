#include <PeanutKingSoccerV3.h>
static PeanutKingSoccerV3 robot = PeanutKingSoccerV3();

void setup() {
  robot.init();
}

void loop() {
  int xleft = robot.ultrasonicRead(left);     // Read data from left ultrasound sensor 
  int xright = robot.ultrasonicRead(right);   // Read data from right ultrasound sensor 
  int xfront = robot.ultrasonicRead(front);   // Read data from front ultrasound sensor 
  int xback = robot.ultrasonicRead(back);     // Read data from back ultrasound sensor 

  robot.setScreen(0, 0, xleft,   4);          // Show data on LCD monitor (Column, row, number, digit)
  robot.setScreen(7, 0, xright,  4);          // Show data on LCD monitor (Column, row, number, digit)
  robot.setScreen(0, 1, xfront,  4);          // Show data on LCD monitor (Column, row, number, digit)
  robot.setScreen(7, 1, xback,   4);          // Show data on LCD monitor (Column, row, number, digit)

  delay(10);
}
