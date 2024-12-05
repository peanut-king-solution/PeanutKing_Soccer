#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
}

void loop() {
  if (robot.buttonRead(1))                      // If pressed button 1
    robot.setOnBrdLED(LED_YELLOW);        // Turn on all led on top board (255,r,g,b,w)

  else if(robot.buttonRead(2))                  // If pressed button 2
    robot.setOnBrdLED(LED_BLUE);                // Turn off all led on top board (255,r,g,b,w)

  else if(robot.buttonRead(3))                  // If pressed button 2
    robot.setOnBrdLED(LED_RED);                // Turn off all led on top board (255,r,g,b,w)

  else if(robot.buttonRead(4))                  // If pressed button 2
    robot.setOnBrdLED(LED_CYAN);                // Turn off all led on top board (255,r,g,b,w)
  else
     robot.setOnBrdLED(LED_OFF);
}
