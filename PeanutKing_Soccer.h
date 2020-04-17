#ifndef PeanutKing_Soccer_H
#define PeanutKing_Soccer_H

#include <Arduino.h>
#include <Wire.h>

// motor,  + clockwise turn when positive value
// 1 4
// 2 3


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

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} rgb;

typedef struct {
  uint16_t h;
  uint8_t s;
  uint8_t v;
} hsv;

typedef struct {
  volatile uint8_t *port;
  uint8_t mask;
  uint8_t numLEDs;
  uint8_t numBytes;
  uint8_t *pixels;     // Holds LED color values (3 or 4 bytes each)
} ledType;

typedef enum { front = 0, left, right, back } sensorNum;

typedef enum {
  black=0,  white,   grey,
  red,      green,   blue, 
  yellow,   cyan,    magenta
} color;


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
  
const uint8_t
  STATERESET   = 0,
  STATESET     = 1;

const float pi = 3.1415926535897;

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

typedef enum {
  Idle = 0,
  Joystick = 1,
  PadButton,
  ButtonDef,
  Attributes,
  EndOfData = 26,
} btDataType;

class PeanutKing_Soccer {
 public:
  // Constant  ===========================================================
  const int8_t 
    PAGEUPPERLIMIT = 6,
    PAGELOWERLIMIT = 0;
  
  const int8_t  compass_address = 8;

  const uint8_t
    LCD_Addr    = 0x38,
    LCD_displayfunction = 0x08,
    
    // turn the display on with no cursor or blinking default
    LCD_displaycontrol = 0x04,
    // Initialize to default text direction (for roman languages)
    LCD_displaymode = 0x02,
    GET_READING = 0x55,
    SET_HOME    = 0x54;
  
  PeanutKing_Soccer(void);
  PeanutKing_Soccer(uint8_t);
  
  // Variables ===========================================================
  bool
    btButton[10],
    autoScanEnabled   = true,
    motorEnabled      = true,
    motorBrakeEnabled = true,
    ledEnabled        = false,
    ledFlashEnabled   = false;
  uint8_t
    btButtonIndex,
    btButtonCode,
    btButtonFunction[4],
    btAttributes[5]  = {5,5,5,5,5},
    btTxBuffer[50],
    btRxBuffer[50]; //store at most the most updated 100 values from BT
  uint16_t
    EYEBOUNDARY = 20,
    autoScanSensors = ALLSENSORS;
  int16_t
    systemTime,      //a reference 100Hz clock, 0-100 every second
    LCD_backlightval,
    btDegree = 0,
    btDistance = 0,
    btRotate = 0;
  ledType
    leds[2];
  uint32_t
    sysTicks = 0;
  
  // functions ============================================================
  uint16_t sort(uint16_t a[], uint8_t size);
  hsv
    rgb2hsv(rgb in);
  virtual void 
    bluetoothRemote(void),
    bluetoothAttributes(void);
  void
    enableScanning(bool = true, uint16_t = ALLSENSORS, bool = false),
    bluetoothSend(char[]),
    bluetoothReceive(void),
    
    ledShow(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t = 0),
    ledSetPixels(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t),
    ledClear(void),
    ledUpdate(uint8_t = 0),
    
    printSpace(uint32_t, uint8_t digits = 3),
    setScreen(uint8_t, uint8_t, char[] ),
    setScreen(uint8_t, uint8_t, int16_t, uint8_t digits = 3);
    
  //protected:
  /* Sensors RAW Data  */
  bool
    rawButton(uint8_t);
  uint8_t 
    rawMonoColor(uint8_t);
  float 
    rawCompass(int8_t, int8_t);
  uint16_t
    rawCompoundEye(uint8_t),
    rawUltrasonic(uint8_t, uint8_t);
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
};

#endif


