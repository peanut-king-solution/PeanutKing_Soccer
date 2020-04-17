#include "PeanutKing_Soccer.h"

#ifndef PeanutKing_Soccer_V2_H
#define PeanutKing_Soccer_V2_H

// 1. userdefinebutton set of movement
// 2. attributes


class PeanutKing_Soccer_V2: public PeanutKing_Soccer {
 public:
  PeanutKing_Soccer_V2(void);

  // functions =======================================================================
  // get sensor reading
  bool
    buttonRead(uint8_t),
    buttTrigRead(uint8_t),
    whiteLineCheck(uint8_t);
  uint8_t 
    floorColorReadRaw(uint8_t, uint8_t = black),
    floorColorRead(uint8_t);
  uint16_t
    compoundEyeRead(uint8_t = 13),
    ultrasonicRead(uint8_t),
    compassRead(void);
  void
    Chase(int&, int&, int&),
    Back(int&, int&, int&),
    bluetoothRemote(void),
    bluetoothAttributes(void);
    
  // testing
  uint8_t
    pressureTest(void),
    motorTest(void);
  void 
    lcdMenu(void),
    testProgram(void),
    ledTest(uint8_t = STATESET),
    btTest(void),
    collisionTest(void),
    
    init(uint8_t = 0),
    debug(uint16_t),
    autoScanning(void),
    motorControl(float,float,float),
    motorSet(uint8_t, int16_t),
    move(int16_t, int16_t),
    moveSmart(uint16_t, int16_t, int16_t = 0, uint8_t = 5),
    motorStop(void),
    
    buttons(void),
    compoundEyes(void);

  // Pin Allocation ==================================================================
  const uint8_t
    tcsblPin,
    ledPin,
    actledPin,
    buttonPin[3],
    pwmPin[4],
    dirPin[4],
    dir2Pin[4],
    diagPin[4],
    trigPin[4],
    echoPin[4],
    tcsSxPin[4],
    tcsRxPin[4],
    irPin[12];
    
  const uint8_t
    numLEDs     = 8;       // Number of RGB LEDs in strip
    
  // Variables =======================================================================
  bool
    button[3],
    buttonPressed[3],
    buttonReleased[3],
    buttonTriggered[3],
    isWhite[4]  = {false},
    onBound[4]  = {false},
    outBound[4] = {false};
  uint8_t 
    maxEye,
    minEye,
    groundColor[4];  //color sensor 
  uint16_t
    compass,
    eyeAngle,
    eye[13];         // 12 ir reading , can be 16, depends on version number
  int16_t
    ultrasonic[4];       //4 ultrasonic reading
  rgb
    colorRGB[4];
  hsv
    colorHSV[4];
    
};

#endif

