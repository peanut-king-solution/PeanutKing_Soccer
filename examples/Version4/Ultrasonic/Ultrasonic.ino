#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();

  /*
  You should first run without configuration,
  to check which ultrasonic sensor (Front / Right / Back / Left)
  is connected to which port (U1 / U2 / U3 / U4),
  then allocate the sensor port to the correct sensor position.

  For example, if the sensors are reading in the order of

    Right -> Front -> Back -> Left
    (Right sensor is reading as Front, Front as Right)
    -> means they are wrong configured

  then you should swap these two sensors,
  configure by uncommenting the below line of code
  */
  // robot.xsound.mapXsounds(U2, U1, U3, U4); // swap Front and Right
}

void loop() {
  // Read distances from ultrasonic sensors
  int xu1 = robot.xsound.read(U1);   // Read distance from front ultrasound sensor
  int xu2 = robot.xsound.read(U2);   // Read distance from right ultrasound sensor
  int xu3 = robot.xsound.read(U3);   // Read distance from back ultrasound sensor
  int xu4 = robot.xsound.read(U4);   // Read distance from left ultrasound sensor

  // Alternatively, you can use the wrapper function for compatibility:
  // int xu1 = robot.ultrasonicRead(U1);
  // int xu2 = robot.ultrasonicRead(U2);
  // int xu3 = robot.ultrasonicRead(U3);
  // int xu4 = robot.ultrasonicRead(U4);

  // Print the readings to the Serial Monitor
  Serial.print("U1: ");
  Serial.print(xu1);
  Serial.print(", U2: ");
  Serial.print(xu2);
  Serial.print(", U3: ");
  Serial.print(xu3);
  Serial.print(", U4: ");
  Serial.println(xu4);
  delay(10);
}