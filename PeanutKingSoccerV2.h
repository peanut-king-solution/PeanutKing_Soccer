/*
 * Copyright (c) 2021 PeanutKing Solution
 *
 * @file        PeanutKingSoccerV2.h
 * @summary     Soccer Robot Library using timer interrupt
 * @version     2.0
 * @author      Jack Kwok
 * @date        11 May 2021
 * https://www.arduino.cc/en/Hacking/PinMapping2560
 */

#ifndef PeanutKingSoccerV2_H
#define PeanutKingSoccerV2_H

#include "IICIT.h"

// motor,  + clockwise turn when positive value
// 1 4
// 2 3


// struct pin_map_t {
//   volatile uint8_t* ddr;
//   volatile uint8_t* pin;
//   volatile uint8_t* port;
//   uint8_t bit;
// };

// static const pin_map_t pinMap[] = {
//   {&DDRD, &PIND, &PORTD, 0},  // D0  0
//   {&DDRD, &PIND, &PORTD, 1},  // D1  1
//   {&DDRD, &PIND, &PORTD, 2},  // D2  2
//   {&DDRD, &PIND, &PORTD, 3},  // D3  3
//   {&DDRD, &PIND, &PORTD, 4},  // D4  4
//   {&DDRD, &PIND, &PORTD, 5},  // D5  5
//   {&DDRD, &PIND, &PORTD, 6},  // D6  6
//   {&DDRD, &PIND, &PORTD, 7},  // D7  7
//   {&DDRB, &PINB, &PORTB, 0},  // B0  8
//   {&DDRB, &PINB, &PORTB, 1},  // B1  9
//   {&DDRB, &PINB, &PORTB, 2},  // B2 10
//   {&DDRB, &PINB, &PORTB, 3},  // B3 11
//   {&DDRB, &PINB, &PORTB, 4},  // B4 12
//   {&DDRB, &PINB, &PORTB, 5},  // B5 13
//   {&DDRC, &PINC, &PORTC, 0},  // C0 14
//   {&DDRC, &PINC, &PORTC, 1},  // C1 15
//   {&DDRC, &PINC, &PORTC, 2},  // C2 16
//   {&DDRC, &PINC, &PORTC, 3},  // C3 17
//   {&DDRC, &PINC, &PORTC, 4},  // C4 18
//   {&DDRC, &PINC, &PORTC, 5}   // C5 19
// };
// static const uint8_t pinCount = sizeof(pinMap)/sizeof(pin_map_t);

// static inline uint8_t badPinNumber(void)
//  __attribute__((error("Pin number is too large or not a constant")));

// static inline __attribute__((always_inline))
//   uint8_t digitalReadFast(uint8_t pin) {
//   if (__builtin_constant_p(pin) && pin < pinCount) {
//     return (*pinMap[pin].pin >> pinMap[pin].bit) & 1;
//   } else {
//     return badPinNumber();
//   }
// }

// static inline __attribute__((always_inline))
//   void digitalWriteFast(uint8_t pin, uint8_t value) {
//   if (__builtin_constant_p(pin) && pin < pinCount) {
//     if (value) {
//       *pinMap[pin].port |= 1 << pinMap[pin].bit;
//     } else {
//       *pinMap[pin].port &= ~(1 << pinMap[pin].bit);
//     }
//   } else {
//     badPinNumber();
//   }
// }

// #define MY_PIN    8

// // do this once at setup
// uint8_t myPin_mask = digitalPinToBitMask(MY_PIN);
// volatile uint8_t *myPin_port = portInputRegister(digitalPinToPort(MY_PIN));

// // read the pin
// uint8_t pinValue = (*myPin_port & myPin_mask) != 0;

//   digitalReadFast
//   return (*pinMap[pin].pin >> pinMap[pin].bit) & 1;
//   pin41, PG0: return (PING >> 0) & 1;
  
//   digitalWriteFast
//   if (value) {
//     *pinMap[pin].port |= 1 << pinMap[pin].bit;
//   } else {
//     *pinMap[pin].port &= ~(1 << pinMap[pin].bit);
//   }
//   pin37, PC0: PORTC |= 1 << 0;
//   PORTC |= 1 << 0;
//   PORTC &= ~(1 << 0);

#define LED1   0x01
#define LED2   0x02
#define LED3   0x04
#define LED4   0x08
#define LED5   0x10
#define LED6   0x20
#define LED7   0x40
#define LED8   0x80

#define DEC 10
#define HEX 16
#define OCT 8
#ifdef  BIN // Prevent warnings if BIN is previously defined in "iotnx4.h" or similar
  #undef  BIN
#endif
#define BIN 2

#define LCD_DISPLAYCONTROL 0x08

// flags for backlight control
#define LCD_BACKLIGHT   0x08    // B00001000
#define LCD_NOBACKLIGHT 0x00    // B00000000

#define En B00000100  // Enable bit
//#define Rw B00000010  // Read/Write bit
#define Rs B00000001  // Register select bit

#define LCD_RETURNHOME 0x02
#define LCD_ENTRYLEFT  
#define LCD_ENTRYSHIFTDECREMENT 0x00
#define LCD_ENTRYMODESET 0x04
#define LCD_CLEARDISPLAY 0x01
#define LCD_SETDDRAMADDR 0x80
#define LCD_FUNCTIONSET 0x20    // B00100000

#define DEBUGMODE       1

#define pulseLow      0
#define pulsePullUp   1
#define pulseHigh     2
#define pulsePullDown 3
#define pulseEnd      4

#define BUTTON_IDLE   0
#define ONPRESSED     1
#define HOLD          2
#define ONRELESAED    3

#define PIN           32
#define NUMPIXELS     8

#define GET_READING   0x55
#define compass_address   8

// old library def and const ===========================

#define STATERESET    0
#define STATESET      1

const uint16_t
  ALLIOs       = 0xffff,
  ALLSENSORS   = 0x0fff,
  COMPASS      = 0x0001,    // 0.27ms
  COMPOUNDEYE  = 0x0002,    // 1.45ms
  ULTRASONIC   = 0x00f0,    // ~14ms ~ 32ms for 4
  ULTRASONIC0  = 0x0010,    
  ULTRASONIC1  = 0x0020,    
  ULTRASONIC2  = 0x0040,    
  ULTRASONIC3  = 0x0080,    
  COLORSENSOR  = 0x0f00,    // ~10ms for 4 (2.7 FOR 4), new 7.6ms for 4
  COLORSENSOR0 = 0x0100,    
  COLORSENSOR1 = 0x0200,    
  COLORSENSOR2 = 0x0400,    
  COLORSENSOR3 = 0x0800,    
  ALLOUTPUTs   = 0xf000,
  LED          = 0x1000,    // ~0.5ms for 8 leds
  MOTOR        = 0x2000,    // 0.42ms for motorcontrol (4motors)
  LCDSCREEN    = 0x4000;
  
typedef enum {
  testLED = 1,
  testMotor,
  testCompass,
  testUltrasonic,
  testCompoundeye,
  testColor,
  testBT,
  
  testAll = 99,
} testUnit;

typedef enum {
  chaseball,
  goal,
  gohome,
  gohome2,
  goleft,
  goright,
  gofront
} pressureTestStatus;


/* =========================================================== */


typedef struct {
  uint16_t r;      uint16_t g;      uint16_t b;
} rgb_t;

typedef struct {
  uint16_t h;     uint8_t s;      uint8_t v;
} hsv_t;

typedef struct {
  uint16_t h;     uint8_t s;      uint8_t l;
} hsl_t;

typedef struct {
  volatile uint8_t *port;
  uint8_t mask;
  uint8_t numLEDs;
  uint8_t numBytes;
  uint8_t *pixels;     // Holds LED color values (3 or 4 bytes each)
} led_t;

typedef struct {
  volatile uint16_t pwm;
  volatile uint8_t* port;
  uint8_t bita;
  uint8_t bitb;
} motor_t;

typedef struct {
//  trgPin;
//  echoPin;
  uint32_t t;
  uint8_t  pSt;
  uint8_t  pinRead;
  int32_t  dur;
} pulsein_t;

typedef struct {
  int32_t  dur[3];
  uint32_t t;
  uint32_t f;
  int32_t  fx[3];
  uint8_t  pSt;
  uint8_t  nxt;
  uint8_t  pulse;
  uint8_t  pinRead;
} pulseinColor_t;

typedef struct {
  volatile uint8_t* port;
  uint8_t bit;
} readpin_t;


typedef enum { front = 0, left, right, back } sensorNum;

typedef enum {
  black=0,  white,   grey,
  red,      blue,    green,
  yellow,   cyan,    magenta
} color_t;

typedef enum {
  Idle = 0,
  Joystick = 1,
  PadButton,
  ButtonDef,
  Attributes,
  EndOfData = 26,
  DemoMode = 25,
} btData_t;

// Original function time needed
  // COMPASS      = 0x0001,    // 0.27ms
  // COMPOUNDEYE  = 0x0002,    // 1.45ms
  // ULTRASONIC   = 0x00f0,    // ~14ms ~ 32ms for 4 
  // COLORSENSOR  = 0x0f00,    // ~10ms for 4 (2.7 FOR 4), new 7.6ms for 4
  // LED          = 0x1000,    // ~0.5ms for 8 leds
  // MOTOR        = 0x2000,    // 0.42ms for motorcontrol (4motors)
  
// trig
// 53  PC0 ( A8 )  Digital pin 37
// 55  PC2 ( A10 ) Digital pin 35
// 56  PC3 ( A11 ) Digital pin 34
// 54  PC1 ( A9 )  Digital pin 36

// echo
// 51  PG0 ( WR )  Digital pin 41
// 70  PG2 ( ALE ) Digital pin 39
// 50  PD7 ( T0 )  Digital pin 38
// 52  PG1 ( RD ) Digital pin 40

// color
// 41  PL6 Digital pin 43
// 40  PL5 ( OC5C )  Digital pin 44 (PWM)
// 39  PL4 ( OC5B )  Digital pin 45 (PWM)
// 38  PL3 ( OC5A )  Digital pin 46 (PWM)

// 72  PA6 ( AD6 ) Digital pin 28
// 73  PA5 ( AD5 ) Digital pin 27
// 71  PA7 ( AD7 ) Digital pin 29
// 74  PA4 ( AD4 ) Digital pin 26

// motor
// 5 PE3 ( OC3A/AIN1 ) Digital pin 5 (PWM)
// 1 PG5 ( OC0B )      Digital pin 4 (PWM)
// 7 PE5 ( OC3C/INT5 ) Digital pin 3 (PWM)
// 6 PE4 ( OC3B/INT4 ) Digital pin 2 (PWM)

// 25  PB6 ( OC1B/PCINT6 ) Digital pin 12 (PWM)
// 26  PB7 ( OC0A/OC1C/PCINT7 )  Digital pin 13 (PWM)
// 23  PB4 ( OC2A/PCINT4 ) Digital pin 10 (PWM)
// 24  PB5 ( OC1A/PCINT5 ) Digital pin 11 (PWM)
// 17  PH5 ( OC4C )  Digital pin 8 (PWM)
// 18  PH6 ( OC2B )  Digital pin 9 (PWM)
// 15  PH3 ( OC4A )  Digital pin 6 (PWM)
// 16  PH4 ( OC4B )  Digital pin 7 (PWM)

// button
// 42	PL7	Digital pin 42
// 37	PL2 ( T5 )	Digital pin 47
// 36	PL1 ( ICP5 )	Digital pin 48

const float pi = 3.1415926535897;

extern pulseinColor_t clr[4];

class PeanutKingSoccerV2 {
 public:
  // Constructor 
  PeanutKingSoccerV2(void);
  PeanutKingSoccerV2(uint8_t);

/* =============================================================================
 *                              Functions
 * ============================================================================= */

  uint16_t
    whiteLineThreshold = 600,
    compass,
    eyeAngle;

  bool
    autoScanEnabled   = true,
    motorEnabled      = true,
    motorBrakeEnabled = true,
    ledEnabled        = false,
    ledFlashEnabled   = false;


  bool buttTrigRead(uint8_t y) {
    bool temp = buttonTriggered[y];
    buttonTriggered[y] = false;
    return temp;
  }

  void enableScanning(bool = true, uint16_t = ALLSENSORS, bool = false) {

  }

  // testing
  void
    debug(uint16_t),
    ledTest(uint8_t = STATESET),
    testProgram(void),
    btTest(void),
    collisionTest(void);
  uint8_t
    pressureTest(void),
    motorTest(void);


  // =============================================================

  uint16_t 
    sort(uint16_t a[], uint8_t size);
  hsv_t
    rgb2hsv(rgb_t in);
    /*
  virtual void 
    bluetoothRemote(void),
    bluetoothAttributes(void);
    */
  void
    init(),
    lcdMenu(void),
    bluetoothSend(char[]),
    bluetoothReceive(void),
    
    ledShow(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t = 0),
    ledSetPixels(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t),
    ledClear(void),
    ledUpdate(uint8_t = 0),
    
    printSpace(uint32_t, uint8_t digits = 3),
    setScreen(uint8_t, uint8_t, char[] ),
    setScreen(uint8_t, uint8_t, int16_t, uint8_t digits = 3);
    
  // Advance Control 
  void
    Chase(int&, int&, int&),
    Back(int&, int&, int&),
    bluetoothRemote(void),
    bluetoothAttributes(void);

//protected:
  /* Sensors RAW Data  */
  void 
    ledSetup(uint8_t, uint8_t, uint8_t),
    lcdSetup(void),
    lcdClear(void),
    setCursor(uint8_t col, uint8_t row);

  /* LCD Library */
  void
    send(uint8_t value, uint8_t mode),
    write4bits(uint8_t value),
    expanderWrite(uint8_t _data);
  size_t 
    printNumber(unsigned long, uint8_t),
    print(long, int = DEC),
    print(const char *),
    write(uint8_t);

  void motorStop(void) {
    for(uint8_t i=0; i<4; i++) {
      motorSet(i, 0);
    }
  }

  inline void motorCal(uint8_t pin, float scale) {
    motorScale[pin] = scale;
  }

  // Constant  ===========================================================
  const int8_t 
    PAGEUPPERLIMIT = 6,
    PAGELOWERLIMIT = 0;

  const uint8_t
    LCD_Addr    = 0x38,
    LCD_displayfunction = 0x08,
    // turn the display on with no cursor or blinking default
    LCD_displaycontrol = 0x04,
    // Initialize to default text direction (for roman languages)
    LCD_displaymode = 0x02;

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
    
  const motor_t motor[4] = {
    {&OCR3A, &PORTB, 6, 7},  // B6 7
    {&OCR0B, &PORTB, 4, 5},  // B4 5
    {&OCR3C, &PORTH, 5, 6},  // H5 6
    {&OCR3B, &PORTH, 3, 4}   // H3 4
  };


  // Sensor Reading ======================================================
  uint16_t compoundEyeRead (uint8_t = 13);
  bool buttonRead(uint8_t button_no);

  uint16_t ultrasonicRead(uint8_t ultrasonic_no);
  uint16_t floorColorRead(uint8_t pin_no, uint8_t mono = 0);
  uint16_t whiteLineCal(uint8_t pin_no, uint16_t calVal);
  bool whiteLineCheck(uint8_t pin_no);
  uint16_t compassRead(uint8_t = compass_address, uint8_t = GET_READING, uint8_t = 3);

  void motorSet(uint8_t pin, int16_t speed);
  void motorControl(float mAngle, float mSpeed, float rotate);
  void moveSmart(uint16_t, int16_t, int16_t = 0, uint8_t = 5);

  // Variables ===========================================================
  bool
    button[3],
    buttonPressed[3],
    buttonReleased[3],
    buttonTriggered[3],
    isWhite[4]  = {false},
    onBound[4]  = {false},
    outBound[4] = {false};
    
  uint32_t
    sysTicks = 0;

  uint16_t eye[12] = {0}, maxEye = 1, minEye = 1;
  int16_t  ultrasonic[4];       //4 ultrasonic reading

  uint16_t
    LCD_backlightval,
    systemTime,      //a reference 100Hz clock, 0-100 every second
    EYEBOUNDARY = 20;

  led_t leds[2];
  rgb_t colorRGB[4];
  hsv_t colorHSV[4];
  float motorScale[4] = {1,1,1,1};
  
  // BT Variables ========================================================
  bool
    btButton[10];
  uint8_t
    btButtonIndex,
    btGestureCode,
    btButtonFunction[4],
    btAttributes[5]  = {5,5,5,5,5},
    btTxBuffer[50],
    btRxBuffer[50]; //store at most the most updated 100 values from BT
  int16_t
    btDegree = 0,
    btDistance = 0,
    btRotate = 0;


  IICIT::Handle compassHandle;
  IICIT::Handle lcdHandle;
};

#endif


