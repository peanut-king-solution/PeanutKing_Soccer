#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
}

void loop() {
  int xu1 = robot.ultrasonicRead(U1);     // Read data from left ultrasound sensor 
  int xu2 = robot.ultrasonicRead(U2);   // Read data from right ultrasound sensor 
  int xu3 = robot.ultrasonicRead(U3);   // Read data from front ultrasound sensor 
  int xu4 = robot.ultrasonicRead(U4);     // Read data from back ultrasound sensor 

  Serial.print("U1:");
  Serial.print(xu1);
  Serial.print("U2:");
  Serial.print(xu2);
  Serial.print("U3:");
  Serial.print(xu3);
  Serial.print("U4:");
  Serial.println(xu4);
  delay(10);
}
