#include <PeanutKingSoccerV3.h>
static PeanutKingSoccerV3 robot = PeanutKingSoccerV3();

void setup() {
  robot.init();
  robot.whiteLineCal(front, 1000);              // Set threshold value of white line 
  robot.whiteLineCal(back, 1000);               // Robot PILA has 4 colour sensor in 4 direction
  robot.whiteLineCal(left, 1000);               // The colour sensor can send out total rgb value of below surface 
  robot.whiteLineCal(right, 1000);              // It can distinguish the soccer field and white line

  int Front = robot.whiteLineCheck(front);      // It is a boolean function   
  int Back = robot.whiteLineCheck(back);        // If the measured value is higher than threshold value, it will return true(1)
  int Left = robot.whiteLineCheck(left);        // If the measured value is lower than threshold value, it will return false(0)
  int Right = robot.whiteLineCheck(right);      //
}

void loop() {
  int Front = robot.whiteLineCal(front);        // 
  int Back = robot.whiteLineCal(back);          // 
  int Left = robot.whiteLineCal(left);          // 
  int Right = robot.whiteLineCal(right);        // 

  robot.setScreen(0, 0, Front, 4);              // Show data on LCD monitor (Column, row, number, digit)
  robot.setScreen(7, 0, Back,  4);              // Show data on LCD monitor (Column, row, number, digit)
  robot.setScreen(0, 1, Left,  4);              // Show data on LCD monitor (Column, row, number, digit)
  robot.setScreen(7, 1, Right, 4);              // Show data on LCD monitor (Column, row, number, digit)
}
