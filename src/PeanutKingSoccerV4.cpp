/*
 * Copyright (c) 2024 PeanutKing Solution
 *
 * @file        PeanutKingSoccerV4.cpp
 * @summary     Soccer Robot V4 Library
 * @version     4.0.0
 * @author      Jack Kwok
 * @date        2 January 2024
 *
 * @log         4.0.0 - 9  Jul 2024 - Extract Compass module
 *              3.3.0 - 5  Jun 2023
 *              3.1.0 - 26 Jul 2022
 */

#include "PeanutKingSoccerV4.h"

/* =============================================================================
 *                              Constructor
 * ============================================================================= */

PeanutKingSoccerV4::PeanutKingSoccerV4(void) :
  pwmPin{10, 11, 12, 13},
  move(motor)   // Initialize Movement module with Motor instance
{
}

/* =============================================================================
 *                              Initialization
 * ============================================================================= */

void PeanutKingSoccerV4::init(uint8_t mode) {
  Serial.begin(115200);
  Serial1.begin(115200);

  // Initialize motor pins
  motor.init();
  
  // Initialize I2C Manager (software + hardware I2C)
  I2CManager::getInstance().init();

  // Initialize color sensor module
  colorSensor.init();

  // Initialize compound eye module
  compoundEye.init();

  // Initialize button module
  buttonMgr.init();

  // Initialize LED module
  ledCtrl.init();

  // Initialize ultrasonic module
  xsound.init();
  
  // Initialize Compass module
  compass.init();

  // Initialize TFT
  #if defined(ST7735_RST_PIN)
    FastPin<ST7735_RST_PIN>::setOutput();
    FastPin<ST7735_RST_PIN>::hi();
    FastPin<ST7735_RST_PIN>::lo();
    delay(1);
    FastPin<ST7735_RST_PIN>::hi();
  #endif
  tft.start_TFT();
  tft.fillScreen(ST7735_BLACK);
}

/* =============================================================================
 *                              Data Fetch
 * ============================================================================= */

void PeanutKingSoccerV4::dataFetch(void) {
  // Color sensor - RGB (via SW I2C)
  for (uint8_t i = CL1; i <= CL8; i++) {
    if (colorSensor.isEnabled((CLR_SENSOR_ID)i)) {
      colorRGB[i] = colorSensor.readRGB((CLR_SENSOR_ID)i);
    }
  }

  // Color sensor - HSL (via SW I2C)
  for (uint8_t i = CL1; i <= CL8; i++) {
    if (colorSensor.isEnabled((CLR_SENSOR_ID)i)) {
      colorHSL[i] = colorSensor.readHSL((CLR_SENSOR_ID)i);
    }
  }

  // Compound eye
  uint8_t* eyePtr = compoundEye.readAll();
  for (uint8_t i = 0; i < 12; i++) {
    eye[i] = eyePtr[i];
  }
  maxEye = compoundEye.getMaxEye();
  eyeAngle = compoundEye.getAngle();

  // Ultrasonic (not sure will it have any effect on the performance)
  for (uint8_t i = U1; i <= U4; i++) {
    ultrasonic[i] = xsound.read((ULTR_SENSOR)i);
  }

  // Compass
  heading = compass.read();
}

/* =============================================================================
 *                              Motor (wrapper)
 * ============================================================================= */

  void PeanutKingSoccerV4::setMotorSpeed(MOTOR_ID mi, int16_t speed) {
    motor.setSpeed(mi, speed);
  }
  void PeanutKingSoccerV4::stopAllMotors(void) {
    motor.stopAll();
  }

/* =============================================================================
*                              Movement (wrapper)
* ============================================================================= */

  void PeanutKingSoccerV4::moveByAngle(float mAngle, float mSpeed, float rotate) {
    move.byAngle(mAngle, mSpeed, rotate);
  }
  void PeanutKingSoccerV4::moveByAnglePID(float mAngle, float mSpeed) {
    move.byAnglePID(mAngle, mSpeed, compass.read());
  }

/* =============================================================================
 *                              Color Sensor (soft I2C)
 * ============================================================================= */

uint8_t PeanutKingSoccerV4::getColorSensor(CLR_SENSOR_ID color_sensor_num) {
  return colorSensor.readColor(color_sensor_num);
}

rgb_t PeanutKingSoccerV4::getColorSensorRGB(CLR_SENSOR_ID color_sensor_num) {
  return colorSensor.readRGB(color_sensor_num);
}

hsl_t PeanutKingSoccerV4::getColorSensorHSL(CLR_SENSOR_ID color_sensor_num) {
  return colorSensor.readHSL(color_sensor_num);
}

uint16_t PeanutKingSoccerV4::whiteLineCal(CLR_SENSOR_ID pin_no) {
  whiteLineThreshold[pin_no] = getColorSensorHSL(pin_no).h;
  return whiteLineThreshold[pin_no];
}

bool PeanutKingSoccerV4::whiteLineCheck(CLR_SENSOR_ID i, uint16_t thresh) {
  colorHSL[i] = getColorSensorHSL(i);
  isWhite[i] = (abs((int)colorHSL[i].h - (int)thresh) < 10 && colorHSL[i].l >= 50);
  return isWhite[i];
}

/* =============================================================================
 *                       IR Compound Eye (wrapper)
 * ============================================================================= */

uint8_t* PeanutKingSoccerV4::compoundEyeRead() {
  uint8_t* eyePtr = compoundEye.readAll();
  for (uint8_t i = 0; i < 12; i++) {
    eye[i] = eyePtr[i];
  }
  return eye;
}

uint8_t PeanutKingSoccerV4::compoundMaxEye() {
  return compoundEye.getMaxEye();
}

uint8_t PeanutKingSoccerV4::compoundMaxEyeVal() {
  return compoundEye.getMaxEyeVal();
}

uint8_t PeanutKingSoccerV4::compoundEyeVal(uint8_t n) {
  return compoundEye.getEyeVal(n);
}

uint16_t PeanutKingSoccerV4::compoundEyeAngle(void) {
  return compoundEye.getAngle();
}

/* =============================================================================
 *                       Button (wrapper for compatibility)
 * ============================================================================= */

bool PeanutKingSoccerV4::buttonRead(BUTTON_ID btn) {
  return buttonMgr.read(btn);
}

void PeanutKingSoccerV4::buttonUpdate(void) {
  buttonMgr.update();
}

buttonStatus_t PeanutKingSoccerV4::buttonGetStatus(BUTTON_ID btn) {
  return buttonMgr.getStatus(btn);
}

/* =============================================================================
 *                       LED (wrapper for compatibility)
 * ============================================================================= */

void PeanutKingSoccerV4::setOnBrdLED(obBrdLEDCL color) {
  ledCtrl.setLED(color);
}

void PeanutKingSoccerV4::setOnBrdLED(uint8_t LED, uint8_t status) {
  ledCtrl.setLED(LED, status);
}

/* =============================================================================
 *                       Ultrasonic (wrapper)
 * ============================================================================= */

uint16_t PeanutKingSoccerV4::ultrasonicRead(ULTR_SENSOR n) {
  return xsound.read(n);
}

/* =============================================================================
 *                              Compass (wrapper)
 * ============================================================================= */

uint16_t PeanutKingSoccerV4::compassRead(void) {
  heading = compass.read();
  return heading;
}

int16_t* PeanutKingSoccerV4::getAccelerometerRaw(void) { return compass.getAccelerometerRaw(); }
int16_t* PeanutKingSoccerV4::getGyroscopeRaw(void) { return compass.getGyroscopeRaw(); }
int16_t* PeanutKingSoccerV4::getMagnetometerRaw(void) { return compass.getMagnetometerRaw(); }

/* =============================================================================
 *                              TFT Display
 * ============================================================================= */

void PeanutKingSoccerV4::setTextColor(uint16_t color) {
  tft.setTextColor(color);
}

void PeanutKingSoccerV4::setTextColor(uint16_t fg, uint16_t bg) {
  tft.setTextColor(fg, bg);
}

void PeanutKingSoccerV4::setTextSize(uint8_t size) {
  tft.setTextSize(size);
}

void PeanutKingSoccerV4::setScreen(uint8_t col, uint8_t row, char string[]) {
  tft.setCursor(col*6, row*10);
  tft.print(string);
}

void PeanutKingSoccerV4::setScreen(uint8_t col, uint8_t row, int16_t numbers) {
  tft.setCursor(col*6, row*10);
  tft.print(numbers);
}

void PeanutKingSoccerV4::clearScreen(void) {
  tft.fillScreen(ST7735_BLACK);
}

void PeanutKingSoccerV4::drawAnglePointer(int x, int y, int radius, uint16_t angle, uint16_t arrowColor) {
  // Normalize angle to [0, 360) range
  angle = angle % 360;
  
  // Clear previous indicator circle
  tft.fillCircle(x, y, radius + 2, ST7735_BLACK);

  // Draw compass circle outline
  tft.drawCircle(x, y, radius, ST7735_WHITE);

  // Draw N/S/E/W markers
  tft.setTextColor(ST7735_RED);
  tft.setTextSize(1);
  tft.setCursor(x - 3, y - radius - 10);
  tft.print("N");
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(x - 3, y + radius + 2);
  tft.print("S");
  tft.setCursor(x + radius + 2, y - 3);
  tft.print("E");
  tft.setCursor(x - radius - 6, y - 3);
  tft.print("W");

  // Calculate arrow position based on heading angle
  float angleRad = angle * 3.14159 / 180.0;
  int arrowTipX = x + (int)(sin(angleRad) * radius);
  int arrowTipY = y - (int)(cos(angleRad) * radius);

  // Draw arrow line from center to tip
  tft.drawLine(x, y, arrowTipX, arrowTipY, arrowColor);

  // Draw arrow head (filled triangle at tip)
  int headSize = 5;
  float perpAngle = angleRad + 3.14159 / 2.0;
  int head1X = arrowTipX - (int)(sin(angleRad) * headSize) + (int)(cos(perpAngle) * headSize / 2);
  int head1Y = arrowTipY + (int)(cos(angleRad) * headSize) + (int)(sin(perpAngle) * headSize / 2);
  int head2X = arrowTipX - (int)(sin(angleRad) * headSize) - (int)(cos(perpAngle) * headSize / 2);
  int head2Y = arrowTipY + (int)(cos(angleRad) * headSize) - (int)(sin(perpAngle) * headSize / 2);
  tft.fillTriangle(arrowTipX, arrowTipY, head1X, head1Y, head2X, head2Y, arrowColor);
}

/* =============================================================================
 *                              Bluetooth
 * ============================================================================= */

void PeanutKingSoccerV4::bluetoothAttributes() {}
void PeanutKingSoccerV4::bluetoothRemote(void) {}

/* =============================================================================
 *                              Strategy Functions
 * ============================================================================= */

void PeanutKingSoccerV4::Chase(int& direct, int& speed, int& rotation) {
  (void)direct; (void)speed; (void)rotation;
}

void PeanutKingSoccerV4::Back(int& direct, int& speed, int& rotation) {
  (void)direct; (void)speed; (void)rotation;
}

/* =============================================================================
 *                              Deprecated / Legacy Code (Archived)
 * ============================================================================= */

// //---------------------------------------------- Set PWM frequency for D4 & D27 ------------------------------
// //TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
// //TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
//   // TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz (Default)
// //TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
// //TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz


// //---------------------------------------------- Set PWM frequency for D11 & D12 -----------------------------

// //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 32772.55 Hz
// TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
//   // TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
// // TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
// //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

// //---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------

// //TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 32772.55 Hz
// // TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
// TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
//   // TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
// //TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
// // TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
// //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz


// //---------------------------------------------- Set PWM frequency for D2, D3 & D5 ---------------------------

// //TCCR3B = TCCR3B & B11111000 | B00000001;    // set timer 3 divisor to     1 for PWM frequency of 32772.55 Hz
// TCCR3B = TCCR3B & B11111000 | B00000010;    // set timer 3 divisor to     8 for PWM frequency of  3921.16 Hz
//   // TCCR3B = TCCR3B & B11111000 | B00000011;    // set timer 3 divisor to    64 for PWM frequency of   490.20 Hz
// // TCCR3B = TCCR3B & B11111000 | B00000100;    // set timer 3 divisor to   256 for PWM frequency of   122.55 Hz
// //TCCR3B = TCCR3B & B11111000 | B00000101;    // set timer 3 divisor to  1024 for PWM frequency of    30.64 Hz


// //---------------------------------------------- Set PWM frequency for D6, D7 & D8 ---------------------------

// //TCCR4B = TCCR4B & B11111000 | B00000001;    // set timer 4 divisor to     1 for PWM frequency of 32772.55 Hz
// TCCR4B = TCCR4B & B11111000 | B00000010;    // set timer 4 divisor to     8 for PWM frequency of  3921.16 Hz
//   // TCCR4B = TCCR4B & B11111000 | B00000011;    // set timer 4 divisor to    64 for PWM frequency of   490.20 Hz
// // TCCR4B = TCCR4B & B11111000 | B00000100;    // set timer 4 divisor to   256 for PWM frequency of   122.55 Hz
// //TCCR4B = TCCR4B & B11111000 | B00000101;    // set timer 4 divisor to  1024 for PWM frequency of    30.64 Hz


// //---------------------------------------------- Set PWM frequency for D44, D45 & D46 ------------------------

// //TCCR5B = TCCR5B & B11111000 | B00000001;    // set timer 5 divisor to     1 for PWM frequency of 32772.55 Hz
// //TCCR5B = TCCR5B & B11111000 | B00000010;    // set timer 5 divisor to     8 for PWM frequency of  3921.16 Hz
//   // TCCR5B = TCCR5B & B11111000 | B00000011;    // set timer 5 divisor to    64 for PWM frequency of   490.20 Hz
// // TCCR5B = TCCR5B & B11111000 | B00000100;    // set timer 5 divisor to   256 for PWM frequency of   122.55 Hz
// //TCCR5B = TCCR5B & B11111000 | B00000101;    // set timer 5 divisor to  1024 for PWM frequency of    30.64 Hz


//   /*
//   //   Timer 1
//   TCCR1A  = 0x00;           // Normal mode, just as a Timer
//   TCNT1   = 0;
//   OCR1A   = 624;             // 8 * 4 / 16 = 2us
//   // OCR1A = 1250;       // =(16*10^6) / (125*256) -1 (must be <65536)
//   OCR1A   = 100;            // Hz = ?

//   // TIMSK1 |= (1 << OCIE1B);  // enable timer compare interrupt

//   */
//   // TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
//   // TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(WGM31) | _BV(WGM30);
//   // TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(WGM31) | _BV(WGM30);
//   // TCCR4C = _BV(COM4A1) | _BV(COM4B1) | _BV(WGM41) | _BV(WGM40);

//   TCNT2   = 0;
//   TCCR2B |= (1 << WGM22);    // CTC mode; Clear Timer on Compare (OCR2A)
//   // TCCR2A |= (1 << WGM21);    // CTC mode; Clear Timer on Compare (OCR2A)
//   TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt
//   OCR2A = 63;
//   // OCR2B = 63;
//   sei();    //allow interrupts

//   //while ( compassRead() == 400 );

//   // uint8_t l[12];
//   // for (uint8_t i=0; i<4; i++) {
//   //   if ( n & (1<<i) ) {
//   //     uint8_t *p = &l[i * 3];  // 4 bytes per pixel
//   //     p[0] = 200;                   // R
//   //     p[1] = 200;                   // G
//   //     p[2] = 200;                   // B
//   //   }
//   // }

//   delay(1500);

// void PeanutKingSoccerV4::I2CSend(int8_t addr, uint8_t *data, uint8_t length) {
//   Wire.beginTransmission(addr);
//   Wire.write(data, length);
//   Wire.endTransmission();
// }

// void PeanutKingSoccerV4::I2CRead(int8_t addr, uint8_t *data, uint8_t length) {
//   uint8_t i=0;
//   Wire.requestFrom((int)addr, (int)length);
//   while (Wire.available()) {
//     data[i++] = Wire.read();
//   }
// }

// void PeanutKingSoccerV4::sendLED(void) {
// }

// uint16_t PeanutKingSoccerV4::sort(uint16_t a[], uint8_t size) {
//   for(uint8_t i=0; i<(size-1); i++) {
//     for(uint8_t o=0; o<(size-(i+1)); o++) {
//       if(a[o] > a[o+1]) {
//         uint16_t t = a[o];
//         a[o] = a[o+1];
//         a[o+1] = t;
//       }
//     }
//   }
//   return a[(size-1)/2];
// }

// hsv_t PeanutKingSoccerV4::rgb2hsv(rgb_t in) {
//   hsv_t      out;
//   int16_t  min, max, delta;

//   min = in.r < in.g ? in.r : in.g;
//   min = min  < in.b ? min  : in.b;

//   max = in.r > in.g ? in.r : in.g;
//   max = max  > in.b ? max  : in.b;

//   out.v = max;                                // v
//   delta = max - min;
//   if ( max == 0 ) { // if max is 0, then r = g = b = 0
//     out.s = 0;                                // s = 0
//     out.h = NAN;                              // h is now undefined
//   }
//   else if ( delta < 1 ) { // grey color
//     out.s = 0;
//     out.h = 0;                                // undefined
//   }
//   else {
//   // NOTE: if Max is == 0, this divide would cause a crash
//     out.s = 255 * delta / max;                // s

//     if ( in.g >= max )                    // > is bogus, just keeps compilor happy
//       out.h = 120 + int16_t( in.b - in.r ) * 60 / delta;  // between cyan & yellow
//     else
//     if ( in.b >= max )
//       out.h = 240 + int16_t( in.r - in.g ) * 60 / delta;  // between magenta & cyan
//     else {
//       out.h = 360 + int16_t( in.g - in.b ) * 60 / delta;  // between yellow & magenta
//       if ( out.h > 360 )
//         out.h -= 360;
//     }
//   }
//   return out;
// }

// void PeanutKingSoccerV4::bluetoothSend(char string[]) {
// // send char
//   Serial1.write(string, sizeof(string));
// }

// void PeanutKingSoccerV4::bluetoothReceive(void) {
// // send char
//   btRxBuffer[0] = Serial1.read();
// }

// Strategy functions (archived implementations)
/*
void PeanutKingSoccerV4::Chase(int& direct, int& speed, int& rotation) {
  static bool outside[4];
  //  attack
  //  Defend
  //  MovingSpeed
  //  BallPossession
  //  Precision

  int16_t
    attackSpeed = 80 + btAttributes[2]*16,                // 150
    defendSpeed = 50 + btAttributes[2]*12,
    BallPossessionSpeed = attackSpeed-btAttributes[3]*4,  // 100
    reading = eye[maxEye];

  uint8_t quadrant;

    speed = 120;//reading > 500 ? BallPossessionSpeed : attackSpeed;
    //rotation = (ultrasonic[right] - ultrasonic[left])/6;

    if (eyeAngle<135)
      direct = eyeAngle*1.5;
    else if (eyeAngle>225)
      direct = eyeAngle * 1.5 - 180; //359 - (359-eyeAngle)*1.5;
    else {
      if (ultrasonic[left] > ultrasonic[right])
        direct = eyeAngle*1.5;
      else
        direct = eyeAngle * 1.5 - 180; //359 - (359-eyeAngle)*1.5;
    }

    uint16_t moveAngle = direct;
    if (moveAngle < 45 || moveAngle > 315)
      quadrant = front;
    else if (moveAngle < 135)
      quadrant = right;
    else if (moveAngle < 225)
      quadrant = back;
    else
      quadrant = left;

    if ( ultrasonic[quadrant]>31 )
      outside[quadrant] = false;
    else {
      whiteLineCheck(quadrant);
      if ( isWhite[quadrant] )
        outside[quadrant] = true;
    }
    if ( outside[quadrant] )
      speed = 0;
    if ( outside[front] && (eyeAngle<100 || eyeAngle>260) )
      speed = 0;
    else if ( outside[back]  && (eyeAngle>270 && eyeAngle>90) )
      speed = 0;
    else
    if ( outside[left] ) {
      if (eyeAngle<120)
        direct = eyeAngle*1.5;
      else if (eyeAngle>300)
        direct = 0;
      else if (eyeAngle>265)
        speed = 0;
      else if (eyeAngle>195)
        direct = 180;
      else
        direct = 360 - (360-eyeAngle)*1.5;
    }
    else if ( outside[right] ) {
      if (eyeAngle>240)
        direct = 360 - (360-eyeAngle)*1.5;
      else if (eyeAngle<60)
        direct = 0;
      else if (eyeAngle<95)
        speed = 0;
      else if (eyeAngle<165)
        direct = 180;
      else
        direct = eyeAngle*1.5;
    }
}

void PeanutKingSoccerV4::Back(int& direct, int& speed, int& rotation) {
  int16_t
    defendSpeed = 50 + btAttributes[2]*12,          // 80
    y = ultrasonic[back] - (11 - btAttributes[2]) * 8,
    x = (ultrasonic[left] - ultrasonic[right])/2;

  speed = defendSpeed;
  if ( abs(x) > 50 || ultrasonic[left]+ultrasonic[right]<130 )
    y -= 25;
  if ( y > 30 )
    direct = atan( (float)x/y)*180+180;
  else if ( x > 4 )
    direct = 270;
  else if ( x < -4 )
    direct = 90;
  else if ( y > 4 )
    direct = 180;
  else if ( y < -4 )
    direct = 0;
  else {
    direct = 0;
    speed = 0;
  }
}
*/

// Bluetooth remote (archived)
/*
typedef enum
{
  FORWARD= 'F',
  BACKWARD ='B',
  LEFT ='L',
  RIGHT= 'R',
  CIRCLE= 'C',
  CROSS= 'X',
  TRIANGLE= 'T',
  SQUARE= 'S',
  START= 'A',
  PAUSE ='P'
}BluetoothCmd;

void PeanutKingSoccerV4::bluetoothRemote(void) {
  static uint32_t last_ticks = 0;
  static BluetoothCmd v = PAUSE;
  static char msg[2] = {0};
  if (Serial1.available()) {
    Serial1.readBytes(msg, 2);
    Serial.print("2char:");
    Serial.print(msg[0]);
    Serial.print(" ");
    Serial.println(msg[1]);
    if(msg[1] == '0'){
      v = msg[0];
    }else if(msg[1] == '1'){
      v = PAUSE;
    }

    switch (v) {
        case FORWARD:
          Serial.println("front");
          motorSet(0,120);
          motorSet(1,120);
          motorSet(2,-120);
          motorSet(3,-120);
          break;
        case BACKWARD:
          Serial.println("back");
          motorSet(0,-120);
          motorSet(1,-120);
          motorSet(2,120);
          motorSet(3,120);
          break;
        case RIGHT:
          Serial.println("left");
          motorSet(0,-120);
          motorSet(1,120);
          motorSet(2,120);
          motorSet(3,-120);
          break;
        case LEFT:
          Serial.println("right");
          motorSet(0,120);
          motorSet(1,-120);
          motorSet(2,-120);
          motorSet(3,120);
          break;
        case CIRCLE:
          Serial.println("rot ccw");
          motorSet(0,120);
          motorSet(1,120);
          motorSet(2,120);
          motorSet(3,120);
          break;
        case SQUARE:
          Serial.println("rot cw");
          motorSet(0,-120);
          motorSet(1,-120);
          motorSet(2,-120);
          motorSet(3,-120);
          break;
        case PAUSE:
          Serial.println("Stop");
          motorSet(0,0);
          motorSet(1,0);
          motorSet(2,0);
          motorSet(3,0);
          break;
        case START:
          break;
      }
  }
}
*/

// BT setup test function
/*
void btSetupnTest(){
  char setBaud[] = "AT+BAUD8";

  Serial1.begin(9600);
  Serial1.write(setBaud, sizeof(setBaud));
  Serial1.end();
  Serial1.begin(115200);
}
*/