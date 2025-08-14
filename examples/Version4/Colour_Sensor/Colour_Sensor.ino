#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();
char* color_name[] = {
  "BLACK",
  "WHITE",
  "GREY",
  "RED",
  "GREEN",
  "BLUE",
  "YELLOW",
  "CYAN"
};

// Replace the following with the printed values
uint16_t WhiteLineThres_1 = 190;
uint16_t WhiteLineThres_2 = 190;
uint16_t WhiteLineThres_3 = 190;
uint16_t WhiteLineThres_4 = 190;
void setup() {
  robot.init();
  for (int CL; CL < 4; CL++) {
    Serial.print("uint16_t WhiteLineThres_");
    Serial.print(CL);
    Serial.print(" = ");
    Serial.print(robot.whiteLineCal(CL));
    Serial.println(";");
  }
}
rgb_t rgb;
hsl_t hsl;
void loop() {

  // get rgb and hsl of 1
  rgb = robot.getColorSensorRGB(CL7);
  hsl = robot.getColorSensorHSL(CL7);
  Serial.print("RGB:");
  Serial.print(rgb.r);
  Serial.print("||");
  Serial.print(rgb.g);
  Serial.print("||");
  Serial.println(rgb.b);
  Serial.print("HSL:");
  Serial.print(hsl.h);
  Serial.print("||");
  Serial.print(hsl.s);
  Serial.print("||");
  Serial.println(hsl.l);
  if(robot.whiteLineCheck(CL1,WhiteLineThres_1)){
    robot.setOnBrdLED(LED_BLUE);
  }else if(robot.whiteLineCheck(CL2,WhiteLineThres_2)){
    robot.setOnBrdLED(LED_GREEN);
  }else if(robot.whiteLineCheck(CL3,WhiteLineThres_3)){
    robot.setOnBrdLED(LED_CYAN);
  }else if(robot.whiteLineCheck(CL4,WhiteLineThres_4)){
    robot.setOnBrdLED(LED_RED);
  }else{
    robot.setOnBrdLED(LED_OFF);
  }
}
