#include <PeanutKing_Soccer_V2.h>

// create soccer robot object
static PeanutKing_Soccer_V2 robot = PeanutKing_Soccer_V2();

void setup() {
  // initialize soccer robot
  robot.init();

  // Turn off sensors auto scanning
  robot.enableScanning(false);
  
  //robot.EYEBOUNDARY = 20;
}

void loop() {
  // "static" makes the variable stays and keeps values
  // start keeps the state
  static bool start = false;
  // get yellow button reading
  bool ButtonA = robot.buttonRead(1);
  
  if ( ButtonA ) {
    // when the button is pressed, the state flipped
    start = !start;
  }
  
  if ( start ) {
    // Turn on sensors auto scanning and set all sonsers
    robot.enableScanning(true, ALLSENSORS);
    // sample for displaying all sensors
    robot.debug(ALLSENSORS);
  }
  else {
    robot.enableScanning(false);

    // print "Hello World" on serial monitor and start a new line
    Serial.println("Hello World");
    
    // print "Second line!" on serial monitor
    Serial.print("Second line!");

    // get eye 1 reading
    int eye1 = robot.compoundEyeRead(1);   //1-14
    
    // get front ultrasonic reading
    int frontUltrasonic = robot.ultrasonicRead(front);

    // get compass reading
    int compass = robot.compassRead();
    
    // get rgb values from color sensor
    int frontRed = robot.floorColorReadRaw(front, red);
    
    // get the color from color sensor
    color frontColor = robot.floorColorRead(front);

    // motor      no, speed
    robot.motorSet(0, 100);

    // clear LCD
    robot.lcdClear();
    // print on LCD x, y, content

    // first line
    robot.setScreen(0, 0, "compass: ");
    robot.setScreen(12, 0, compass);

    // second line
    robot.setScreen(0, 1, "front ults: ");
    robot.setScreen(12, 1, frontUltrasonic);
    
    // delay 0.5s before looping to the beginning
    delay(500);
  }
}
