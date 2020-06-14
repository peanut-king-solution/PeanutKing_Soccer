#include "PeanutKing_Soccer.h"

#ifndef PeanutKing_Soccer_V3_H
#define PeanutKing_Soccer_V3_H

// 1. userdefinebutton set of movement
// 2. attributes

typedef enum {
  cmdTransmit   = 'T',
  cmdReceive    = 'R',
  cmdParameter  = 'P', 
} Commands;

typedef enum {
  Compass     = 'C',  // 2byte
  Ultrasonic  = 'U',  // 2byte*4
  IRsensor    = 'I',  // 2byte*12 + 1
  ColorSensor = 'K',  // (3+1)byte*4
  Motor       = 'M',  // 2byte*4
  Led         = 'L',  // 4byte*8
  Screen      = 'S',  // ?
} Sensors;

typedef enum {
  onIdle = 0,
  onPressed,
  onHold,
  onReleased,
} ButtonStatus;

class PeanutKing_Soccer_V3: public PeanutKing_Soccer {
 public:
  PeanutKing_Soccer_V3(void);

  // functions =======================================================================
  // get sensor reading
  bool
    buttonRead(uint8_t),
    whiteLineCheck(uint8_t);
  uint8_t 
    floorColorReadRaw(uint8_t, uint8_t = black),
    floorColorRead(uint8_t);
  uint16_t
    compoundEyeRead(uint8_t = 13),
    ultrasonicRead(uint8_t),
    compassRead(void);

  void 
    init(uint8_t = 0),
    autoScanning(void),

    motorControl(float,float,float),
    motorSet(uint8_t, int16_t),
    move(int16_t, int16_t),
    moveSmart(uint16_t, int16_t, int16_t = 0, uint8_t = 5),
    motorStop(void),
    
    dataFetch( ),
    I2CSensorRead(int8_t addr, Sensors sensor, uint8_t length),
    I2CSensorSend(int8_t addr, Sensors sensor, uint8_t *data, uint8_t length),
    I2CSend(int8_t addr, uint8_t *data, uint8_t length),
    I2CRead(int8_t addr, uint8_t *data, uint8_t length),

    buttons(void);

  
  // Pin Allocation ==================================================================
  const uint8_t
    sensorBoardAddr,
    topBoardAddr,
    actledPin,
    buttonPin[3],
    pwmPin[4],
    dirPin[4],
    dir2Pin[4],
    diagPin[4];
    
  const uint8_t
    numLEDs     = 8;  // Number of RGB LEDs in strip
    
  // Variables =======================================================================
  bool
    button[3],
    buttonPressed[3],
    isWhite[4]  = {false},
    onBound[4]  = {false},
    outBound[4] = {false};
  uint8_t 
    maxEye,
    groundColor[4];   //color sensor
  uint16_t
    compass,
    eyeAngle,
    eye[13];          // 12 ir reading
  int16_t
    ultrasonic[4];    //4 ultrasonic reading
  rgb_t
    colorRGB[4];
    
  uint8_t rxBuff[50];
  uint8_t txBuff[50];
};

#endif

