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
  "CYAN"    };
void setup() {
  robot.init();
}

RGB_Struct rgb;
HSL_Struct hsl;
void loop() {
  Serial.print("color:");
  Serial.println(color_name[robot.getColorSensor(CL7)]);
  // get rgb and hsl of 1 
  rgb = robot.getColorSensorRGB(CL7);
  hsl = robot.getColorSensorHSL(CL7);
  Serial.print("RGB:");
  Serial.print(rgb.red);                                                                                                                                                                                                                                                                                                                                                                 
  Serial.print("||");
  Serial.print(rgb.green);
  Serial.print("||");
  Serial.println(rgb.blue);
  Serial.print("HSL:");
  Serial.print(hsl.h);
  Serial.print("||");
  Serial.print(hsl.s);
  Serial.print("||");
  Serial.println(hsl.l);
}
