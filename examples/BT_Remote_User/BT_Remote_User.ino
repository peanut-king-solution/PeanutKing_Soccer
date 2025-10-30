#include <PeanutKingSoccerV3.h>
static PeanutKingSoccerV3 robot = PeanutKingSoccerV3();

void PeanutKingSoccerV3::bluetoothSendStr(){ // declared in the header file but not in the cpp file
  // protocol defined here
  String send2bt = "soccer,";
  send2bt = send2bt + String(robot.compassRead()) + ",";
  
  send2bt = send2bt + String(robot.ultrasonicRead(front)) + "," + String(robot.ultrasonicRead(back)) + "," + String(robot.ultrasonicRead(left)) + "," + String(robot.ultrasonicRead(right)) + ",";

  send2bt = send2bt + String(compoundEyeRead(13)) + "," + String(compoundEyeRead(14));

  Serial1.println(send2bt);
}

void setup() {
  robot.init();
  robot.bluetoothRename("PKS-soccerTest8");
  robot.bluetoothPrintName();
  // put your setup code here, to run once:

}
void loop() {
  // put your main code here, to run repeatedly:
  robot.bluetoothRemote();
  robot.ultrasonicRead(0); // need this  to update LED

  // robot.setScreen(0, 0, robot.compoundEyeRead(13), 4);
  // Serial.println(angle);
}
