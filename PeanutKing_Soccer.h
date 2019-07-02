
#ifndef PeanutKing_Soccer_H
#define PeanutKing_Soccer_H

#include <Arduino.h>
#include <Wire.h>

//#include <LiquidCrystal_I2C.h>

// motor,  + clockwise
// 14
// 23


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

// flags for display on/off control
#define LCD_DISPLAYON  0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSOROFF  0x00
#define LCD_BLINKOFF   0x00

#define LCD_RETURNHOME 0x02
#define LCD_ENTRYLEFT  0x02
#define LCD_ENTRYSHIFTDECREMENT 0x00
#define LCD_ENTRYMODESET 0x04
#define LCD_CLEARDISPLAY 0x01

#define LCD_SETDDRAMADDR 0x80

#define LCD_FUNCTIONSET 0x20    // B00100000
#define LCD_4BITMODE    0x00    // B000X0000  
#define LCD_2LINE       0x08    // B0000X000
#define LCD_5x8DOTS     0x00    // B00000X00


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
  ALLSENSOR    = 0x0fff,
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
  ALLOUTPUT    = 0xf000,
  LED          = 0x1000,    // ~0.5ms for 8 leds
  MOTOR        = 0x2000,    // 0.42ms for motorcontrol (4motors)
  LCDSCREEN    = 0x4000;
  
const uint8_t
  STATERESET   = 0,
  STATESET     = 1;

const float pi = 3.1415926535897;

class PeanutKing_Soccer {
  const int8_t 
    PAGEUPPERLIMIT = 6,
    PAGELOWERLIMIT = 0;
  
  const int8_t  compass_address = 8;
  const uint8_t
    LCD_Addr    = 0x38,
    LCD_lineNum = 2,
    
    LCD_displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS,
    
    // turn the display on with no cursor or blinking default
    LCD_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF,
    
    // Initialize to default text direction (for roman languages)
    LCD_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT,
    
    numLEDs     = 8,       // Number of RGB LEDs in strip
    GET_READING = 0x55,
    SET_HOME    = 0x54,
    
    tcsblPin    = 32,
    ledPin      = 33,
    
    buttonPin[3] = {42, 47, 48},
    
    pwmPin[4]   = { 5,  4,  3,  2},
    //dirPin[4]   = {22, 23, 24, 25},  // v2.1
    dirPin[4]   = {12, 10,  8,  6},  // v2.2
    dir2Pin[4]  = {13, 11,  9,  7},
    diagPin[4]  = {50, 51, 52, 53},
    // xsound:     xf, xl. xr. xb
    trigPin[4]  = {37, 35, 34, 36},
    echoPin[4]  = {41, 39, 38, 40},
    // rgbs:       s0, s1, s2, s3
    tcsSxPin[4] = {43, 44, 45, 46},
    tcsRxPin[4] = {28, 27, 29, 26},
    
    irPin[12]   = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11};
  
    
 public:
  PeanutKing_Soccer(void);
  PeanutKing_Soccer(uint8_t);
  
  bool
    autoScanEnabled   = true,
    motorEnabled      = true,
    motorBrakeEnabled = true,
    ledEnabled        = false,
    ledFlashEnabled   = false,
    
    Button[3],
    buttonPressed[3],
    buttonReleased[3],
    buttonTriggered[3],
    isWhite[4]  = {false},
    onBound[4]  = {false},
    outBound[4] = {false};
  uint8_t 
    LCD_backlightval,
    systemTime,      //a reference 100Hz clock, 0-100 every second
    MaxEye,
    MinEye,
    GroundColor[4];  //color sensor
  int16_t
    Xsonic[4];       //4 xsonic reading
  uint16_t
    autoScanSensors = ALLSENSOR,
    EYEBOUNDARY = 20,
    eyeAngle,
    Eye[13],         // 12 ir reading , can be 16, depends on version number
    Compass;         // compass angle reading
  //  BT_buffer[100]; //store at most the most updated 100 values from BT
  ledType
    leds[2];
  rgb
    colorRGB[4];
  hsv
    colorHSV[4];
    
  uint32_t
    sysTicks = 0;
  
  // functions ------------------------------------------------------
  void
    ledTest (uint8_t = STATESET),
    btTest(void),
    testProgram(void),
    printSensors(uint16_t);
  uint8_t
    motorTest(void);
  
  hsv
    rgb2hsv(rgb in);
    
  bool
    buttonRead(uint8_t);
  uint16_t
    compassRead(void),
    compoundEyeRead(uint8_t),
    ultrasonicRead(uint8_t);
  uint8_t
    goundColorRead(uint8_t);
    
  int16_t mapSpeed (float);
  
  void
    init(void),
    autoScanning(void),
    bluetoothSend(char[]),
    bluetoothReceive(void),
    enableScanning(bool, uint8_t),
    
    Testing(void),
    debug(uint16_t),
    LCDMenu(void),
    
    motorSet(uint8_t, int16_t),
    move(int16_t, int16_t),
    moveSmart(uint16_t , int16_t),
    motorStop(void),
    
    ledSetup(uint8_t, uint8_t, uint8_t),
    ledShow(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t = 0),
    ledSetPixels(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t),
    ledClear(void),
    ledUpdate(uint8_t = 0),
    
    printSpace(uint32_t, uint8_t),
    LCDPrintSpace(int16_t),
    setScreen(uint8_t, uint8_t, char[] ),
    setScreen(uint8_t, uint8_t, int16_t );
    
  //protected:
  float 
    rawCompass(void),
    rawGyro(void),
    rawAccel(void);
  
  bool rawButton(uint8_t);
  uint16_t
    setHome(void),
    rawCompoundEye(uint8_t),
    rawUltrasonic(uint8_t);
    
  uint8_t rawMonoColor(uint8_t);
  
  void 
    motorControl(float,float,float),
    buttons(void),
    compoundEyes(void);
  
  void
    LCDSetup(void),
    LCDClear(void),
    setCursor(uint8_t col, uint8_t row),
    send(uint8_t value, uint8_t mode),
    write4bits(uint8_t value),
    expanderWrite(uint8_t _data);
  
  size_t 
    printNumber(unsigned long, uint8_t),
    print(long, int = DEC),
    print(const char *),
    write(uint8_t);
};

#endif // ROBOT_H


