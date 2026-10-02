#include <PeanutKingSoccerV4.h>
#include "PKS_ICON.h"
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();
  robot.tft.fillScreen(ST7735_BLACK);
}

void loop() {
  robot.tft.drawBitmap(0, 0, PKS_ICONPKS_White_LOGO, 121, 64, ST7735_WHITE);
}