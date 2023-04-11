#include <PeanutKingSoccerV3.h>
static PeanutKingSoccerV3 robot = PeanutKingSoccerV3();

void setup() {
  robot.init();
}

void loop() {
  if (robot.buttonRead(1))                      // If pressed button 1
    robot.setLED(255, 255, 255, 255, 255);        // Turn on all led on top board (255,r,g,b,w)

  else if(robot.buttonRead(2))                  // If pressed button 2
    robot.setLED(255, 0, 0, 0, 0);                // Turn off all led on top board (255,r,g,b,w)

}
