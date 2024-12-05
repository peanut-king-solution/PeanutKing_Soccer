#include "PeanutKingSoccerV4.h"
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  // put your setup code here, to run once:
  robot.init();
	robot.tft.setTextSize(1);
	robot.tft.setTextColor(ST7735_WHITE);
  robot.tft.setCursor(0, 0);
	robot.tft.print("Hello World.");

}
uint32_t curr_tick = 0;
void loop() {
  // put your main code here, to run repeatedly:
	robot.tft.setTextColor(ST7735_BLACK);
  robot.tft.setCursor(0, 10);
	robot.tft.print(curr_tick);
  curr_tick = millis();
	robot.tft.setTextColor(ST7735_WHITE);
  robot.tft.setCursor(0, 10);
	robot.tft.print(curr_tick);
  delay(10);

}
