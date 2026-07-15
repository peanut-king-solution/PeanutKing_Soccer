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
  // robot.xsound.configuration(U2, U1, U3, U4); // swap Front and Right

  /*
  If you only have 2 or 3 sensors connected,
  you can disable the unused sensors to save time.
  Disabled sensors will not be triggered and return 0 when read.

  Method 1: Use setEnabled() with 4 booleans
  */
  // robot.xsound.setEnabled(true, false, true, false);  // Enable U1, U3 only

  /*
  Method 2: Use enableSensor() to control individual sensors
  */
  // robot.xsound.enableSensor(U1, true);   // Enable U1
  // robot.xsound.enableSensor(U2, false);  // Disable U2
  // robot.xsound.enableSensor(U3, true);   // Enable U3
  // robot.xsound.enableSensor(U4, false);  // Disable U4

  /*
  Method 3: Use enableAll() to enable/disable all sensors at once
  */
  // robot.xsound.enableAll(false);  // Disable all sensors
  // robot.xsound.enableAll(true);   // Enable all sensors
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

  // Or you can read all ultrasonic sensors at once using the dataFetch() method:
  // robot.dataFetch();
  // int xu1 = robot.ultrasonic[0];
  // int xu2 = robot.ultrasonic[1];
  // int xu3 = robot.ultrasonic[2];
  // int xu4 = robot.ultrasonic[3];

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