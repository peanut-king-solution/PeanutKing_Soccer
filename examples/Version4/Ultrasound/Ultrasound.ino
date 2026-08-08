/**
 * This example demonstrates how to use the Ultrasound module of the PeanutKingSoccerV4 robot.
 * It reads distances from the four ultrasonic sensors (Front, Right, Back, Left)
 * and prints the readings to the Serial Monitor.
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

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
  // robot.ultrasoundConfiguration(U2, U1, U3, U4); // swap Front and Right
}

void loop() {
  // Read distances from ultrasonic sensors by position (Front, Right, Back, Left)
  int xu1 = robot.ultrasoundGetDist(Front);   // Read distance from front
  int xu2 = robot.ultrasoundGetDist(Right);   // Read distance from right
  int xu3 = robot.ultrasoundGetDist(Back);    // Read distance from back
  int xu4 = robot.ultrasoundGetDist(Left);    // Read distance from left

  // Or you can read all ultrasonic sensors at once using the dataFetch() method:
  // robot.dataFetch();
  // int xu1 = robot.distances[0];    // Read distance from front
  // int xu2 = robot.distances[1];    // Read distance from right
  // int xu3 = robot.distances[2];    // Read distance from back
  // int xu4 = robot.distances[3];    // Read distance from left

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