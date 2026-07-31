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
  // robot.ultrasoundConfig(U2, U1, U3, U4); // swap Front and Right

  /*
  If you only have 2 or 3 sensors connected,
  you can disable the unused sensors to save time.
  Disabled sensors will not be triggered and return 0 when read.

  Method 1: Use ultrasoundSetEnabled() to enable/disable sensors by position
  Parameters are now by Position: front, right, back, left
  */
  // robot.ultrasoundSetEnabled(true, false, true, false);  // Enable Front, Back only

  /*
  Method 2: Use enable() to enable/disable sensors by port (U1~U4)
  Parameters are now by UltrasoundId: U1, U2, U3, U4
  */
  // robot.ultrasound.enable(U1, true);   // Enable U1
  // robot.ultrasound.enable(U2, false);  // Disable U2
  // robot.ultrasound.enable(U3, true);   // Enable U3
  // robot.ultrasound.enable(U4, false);  // Disable U4

  /*
  Method 3: Use ultrasoundEnableAll() to enable/disable all sensors at once (via V4 wrapper)
  */
  // robot.ultrasoundEnableAll(false);  // Disable all sensors
  // robot.ultrasoundEnableAll(true);   // Enable all sensors
}

void loop() {
  // Read distances from ultrasonic sensors by position (Front, Right, Back, Left)
  int xu1 = robot.ultrasoundGetDist(Position::FRONT);   // Read distance from front
  int xu2 = robot.ultrasoundGetDist(Position::RIGHT);   // Read distance from right
  int xu3 = robot.ultrasoundGetDist(Position::BACK);    // Read distance from back
  int xu4 = robot.ultrasoundGetDist(Position::LEFT);    // Read distance from left

  // Alternatively, you can read directly from the Ultrasound module by port:
  // int xu1 = robot.ultrasound.read(U1);
  // int xu2 = robot.ultrasound.read(U2);
  // int xu3 = robot.ultrasound.read(U3);
  // int xu4 = robot.ultrasound.read(U4);

  // Or you can read all ultrasonic sensors at once using the dataFetch() method:
  // robot.dataFetch();
  // int xu1 = robot.distances[0];
  // int xu2 = robot.distances[1];
  // int xu3 = robot.distances[2];
  // int xu4 = robot.distances[3];

  // Print the readings to the Serial Monitor
  Serial.print("Front: ");
  Serial.print(xu1);
  Serial.print(", Right: ");
  Serial.print(xu2);
  Serial.print(", Back: ");
  Serial.print(xu3);
  Serial.print(", Left: ");
  Serial.println(xu4);
  delay(10);
}