/* 
 *  PEANUT KING SOCCER LIBRARY V2.5
 */
#include "PeanutKing_Soccer.h"

#ifndef PeanutKing_Soccer_V1_H
#define PeanutKing_Soccer_V1_H


class PeanutKing_Soccer_V1: public PeanutKing_Soccer {
 public:
  PeanutKing_Soccer_V1(void);

  // ==================   METHOD DECLARATIONS   ===================

  void init(uint8_t);
  bool buttonPress(uint8_t);
  bool
    buttonRead(uint8_t);//0,1,2 White,Left
    
  uint16_t
    compoundEyeRead(uint8_t),
    ultrasonicRead(uint8_t),
    compassRead(void);
    
  void Testing (void);
  void motorControl(uint8_t, uint8_t speed = 2);
  void motors(int16_t a, int16_t b, int16_t c);
  void moveSmart(int spd, float angle);
    
  color whiteLine(uint8_t);

  int mapSpeed(float); //helper function of motorControl, can be only called in this .h file
  void motors0(int16_t, int16_t, int16_t);
  void motorControl(uint16_t, uint16_t);
  
  // Pin Allocation ==================================================================
  const uint8_t
    ledPin,
    buttonPin[3],
    pwmPin[3],
    dirPin[3],
    dir2Pin[3],
    diagPin[3],
    trigPin[4],
    echoPin[4],
    tcsSxPin[4],
    tcsRxPin[3],
    irPin[9];

  const uint8_t
    numLEDs     = 1;       // Number of RGB LEDs in strip
    
  // ======================     VARIABLES    ======================
  uint8_t ROBOT_VERSION = 2;
  
  bool
    button[3],
    buttonPressed[3],
    buttonReleased[3],
    buttonTriggered[3],
    isWhite[4]  = {false},
    onBound[4]  = {false},
    outBound[4] = {false};
  uint8_t 
    LCD_backlightval,
    systemTime,      //a reference 100Hz clock, 0-100 every second
    maxEye,
    minEye,
    groundColor[4];  //color sensor
  uint16_t
    autoScanSensors = ALLSENSORS,
    EYEBOUNDARY = 20,
    compass,
    eyeAngle,
    eye[13];         // 12 ir reading , can be 16, depends on version number
  int16_t
    ultrasonic[4];       //4 ultrasonic reading
  led_t
    leds[2];
  rgb_t
    colorRGB[4];
  hsv_t
    colorHSV[4];
};

#endif


