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
 *                              Static ISR Wrappers (Ultrasonic)
 * ============================================================================= */

static PeanutKingSoccerV4* V4bot = NULL;

static void PeanutKingSoccerV4::ULT_Echo_dect_0() { V4bot->ULT_Echo_dect(0); }
static void PeanutKingSoccerV4::ULT_Echo_dect_1() { V4bot->ULT_Echo_dect(1); }
static void PeanutKingSoccerV4::ULT_Echo_dect_2() { V4bot->ULT_Echo_dect(2); }
static void PeanutKingSoccerV4::ULT_Echo_dect_3() { V4bot->ULT_Echo_dect(3); }

void (*PeanutKingSoccerV4::ULT_Echo_dect_ptr[4])() = {
  PeanutKingSoccerV4::ULT_Echo_dect_0,
  PeanutKingSoccerV4::ULT_Echo_dect_1,
  PeanutKingSoccerV4::ULT_Echo_dect_2,
  PeanutKingSoccerV4::ULT_Echo_dect_3
};

/* =============================================================================
 *                              Constructor
 * ============================================================================= */

PeanutKingSoccerV4::PeanutKingSoccerV4(void) :
  swiic{
    SlowSoftI2CMaster(29, 30, 1),
    SlowSoftI2CMaster(31, 32, 1),
    SlowSoftI2CMaster(33, 34, 1),
    SlowSoftI2CMaster(35, 36, 1),
    SlowSoftI2CMaster(37, 38, 1),
    SlowSoftI2CMaster(39, 40, 1),
    SlowSoftI2CMaster(41, 42, 1),
    SlowSoftI2CMaster(43, 44, 1)},
  buttonPin{22, 23, 24, 25},
  ledPin{26, 28, 27},
  ULTPin_trig{49, 48, 47, 46},
  ULTPin_echo{A15, A14, A13, A12},
  pwmPin{10, 11, 12, 13},
  move(motor)   // Initialize Movement module with Motor instance
{
  if (V4bot == NULL) {
    V4bot = this;
  }
}

/* =============================================================================
 *                              Initialization
 * ============================================================================= */

void PeanutKingSoccerV4::init(uint8_t mode) {
  Serial.begin(115200);
  Serial1.begin(115200);

  // Initialize motor pins
  motor.init();

  // Initialize Compass module (I2CManager is initialized inside Compass::init())
  compass.init();

  // Initialize soft I2C instances for color sensors
  for (uint8_t i=0; i<8; i++){
    swiic[i].i2c_init();
  }

  // Initialize button pins
  for (uint8_t i=0; i<4; i++)
    pinMode(buttonPin[i], INPUT_PULLUP);

  // Initialize LED pins
  for (uint8_t i=0; i<3; i++)
    pinMode(ledPin[i], OUTPUT);

  // Initialize ultrasonic pins and interrupts
  for (uint8_t i=0; i<4; i++) {
    pinMode(ULTPin_trig[i], OUTPUT);
    pinMode(ULTPin_echo[i], INPUT);
    PcInt::attachInterrupt(ULTPin_echo[i], ULT_Echo_dect_ptr[i], CHANGE);
  }
  delay(10);

  // Register I2C devices via legacy IICIT
  senbrdHandle = gIIC->RegisterDevice(sensorBoardAddr, 1, IICIT::Speed::SLOW);

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
  // Compass
  heading = compass.read();

  // Compound eye
  compoundEyeRead();

  // Color sensor - RGB
  for (uint8_t i=0; i<28; i++)    rxBuff[i] = 0;
  I2CSensorRead(senbrdHandle, COLOR_RAW, 28);
  for (uint8_t i=0; i<4; i++) {
    colorRGB[i].r  = rxBuff[6*i]   | rxBuff[6*i+1]<<8;
    colorRGB[i].g  = rxBuff[6*i+2] | rxBuff[6*i+3]<<8;
    colorRGB[i].b  = rxBuff[6*i+4] | rxBuff[6*i+5]<<8;
    groundColor[i] = rxBuff[24+i];
  }

  // Color sensor - HSL
  for (uint8_t i=0; i<16; i++)    rxBuff[i] = 0;
  I2CSensorRead(senbrdHandle, COLOR_HSL, 16);
  for (uint8_t i=0; i<4; i++) {
    colorHSL[i].h  = rxBuff[4*i] | rxBuff[4*i+1]<<8;
    colorHSL[i].s  = rxBuff[4*i+2];
    colorHSL[i].l  = rxBuff[4*i+3];
  }

  // Color sensor - HSV
  for (uint8_t i=0; i<16; i++)    rxBuff[i] = 0;
  I2CSensorRead(senbrdHandle, COLOR_HSV, 16);
  for (uint8_t i=0; i<4; i++) {
    colorHSV[i].h  = rxBuff[4*i] | rxBuff[4*i+1]<<8;
    colorHSV[i].s  = rxBuff[4*i+2];
    colorHSV[i].v  = rxBuff[4*i+3];
  }
}

/* =============================================================================
 *                              I2C Low-Level (Legacy IICIT)
 * ============================================================================= */

void PeanutKingSoccerV4::I2CSensorRead(IICIT::Handle handle, uint8_t sensor, uint8_t length) {
  uint8_t _status;
  txBuff[0] = sensor;
  _status = gIIC->Write(handle, txBuff, 1);
  _status = gIIC->Read(handle, rxBuff, length);
}

void PeanutKingSoccerV4::I2CSensorSend(IICIT::Handle handle, uint8_t sensor, uint8_t *data, uint8_t length) {
  uint8_t _status;
  txBuff[0] = sensor;
  for (uint8_t i=0; i<length; i++) {
    txBuff[i+1] = data[i];
  }
  _status = gIIC->Write(handle, txBuff, length+1);
}

IICIT::status_t PeanutKingSoccerV4::rxCpltCallback(const IICIT::status_t status) {
  return status;
}

/* =============================================================================
 *                              Color Sensor (soft I2C)
 * ============================================================================= */

uint8_t PeanutKingSoccerV4::getColorSensor(uint8_t color_sensor_num) {
  uint8_t val = 0;
  if (!swiic[color_sensor_num].i2c_start((0x11<<1)|I2C_WRITE)) { return val; }
  swiic[color_sensor_num].i2c_write(0x01);
  swiic[color_sensor_num].i2c_rep_start((0x11<<1)|I2C_READ);
  val = swiic[color_sensor_num].i2c_read(true);
  swiic[color_sensor_num].i2c_stop();
  return val;
}

rgb_t PeanutKingSoccerV4::getColorSensorRGB(uint8_t color_sensor_num) {
  rgb_t temp;
  if (!swiic[color_sensor_num].i2c_start((0x11<<1)|I2C_WRITE)) { return temp; }
  swiic[color_sensor_num].i2c_write(0x08);
  swiic[color_sensor_num].i2c_rep_start((0x11<<1)|I2C_READ);
  temp.r = swiic[color_sensor_num].i2c_read(false);
  temp.g = swiic[color_sensor_num].i2c_read(false);
  temp.b = swiic[color_sensor_num].i2c_read(true);
  swiic[color_sensor_num].i2c_stop();
  return temp;
}

hsl_t PeanutKingSoccerV4::getColorSensorHSL(uint8_t color_sensor_num) {
  hsl_t temp;
  if (!swiic[color_sensor_num].i2c_start((0x11<<1)|I2C_WRITE)) { return temp; }
  swiic[color_sensor_num].i2c_write(0x03);
  swiic[color_sensor_num].i2c_rep_start((0x11<<1)|I2C_READ);
  temp.h = (uint16_t)(swiic[color_sensor_num].i2c_read(0) | swiic[color_sensor_num].i2c_read(0)<<8);
  temp.s = swiic[color_sensor_num].i2c_read(0);
  temp.l = swiic[color_sensor_num].i2c_read(1);
  swiic[color_sensor_num].i2c_stop();
  return temp;
}

/* =============================================================================
 *                              Button
 * ============================================================================= */

bool PeanutKingSoccerV4::buttonRead(uint8_t button_no) {
  if (button_no == 1 || button_no == 2 || button_no == 3 || button_no == 4)
    return !digitalRead(buttonPin[button_no-1]);
  return 0;
}

bool PeanutKingSoccerV4::buttTrigRead(uint8_t pin) {
  return digitalRead(buttonPin[pin]);
}

void PeanutKingSoccerV4::buttons(void) {
  static uint32_t holdTimer[4] = {0};
  uint32_t currentTime = millis();

  for (uint8_t i=0; i<4; i++) {
    bool b = !digitalRead(buttonPin[i]);
    if (b) {
      switch(button[i]) {
        case NONE:  button[i] = TAP; holdTimer[i] = currentTime; break;
        case TAP:   button[i] = PRESS; break;
        case TAP2:  if (currentTime - holdTimer[i] > HOLD_DURATION) button[i] = HOLD2; break;
        case TAP3:  if (currentTime - holdTimer[i] > HOLD_DURATION) button[i] = RELEASE; break;
        case PRESS: if (currentTime - holdTimer[i] > HOLD_DURATION) button[i] = HOLD; break;
        case TAP1_W:  holdTimer[i] = currentTime; button[i] = TAP2; break;
        case TAP2_W:  holdTimer[i] = currentTime; button[i] = TAP3; break;
        case TAP3_W:  if (currentTime - holdTimer[i] > HOLD_DURATION) button[i] = RELEASE; break;
        case RELEASE:
        case RELEASE_S:
        case RELEASE_L: button[i] = TAP; break;
        case HOLD: break;
        default: break;
      }
    }
    else {
      switch(button[i]) {
        case TAP:   button[i] = TAP1_W; holdTimer[i] = currentTime; break;
        case TAP2:  button[i] = TAP2_W; holdTimer[i] = currentTime; break;
        case TAP3:  button[i] = RELEASE; holdTimer[i] = currentTime; break;
        case PRESS: button[i] = RELEASE_S; break;
        case TAP1_W: if (currentTime - holdTimer[i] > WAIT_DURATION) button[i] = RELEASE_S; break;
        case HOLD:  button[i] = RELEASE_L; break;
        case TAP2_W: if (currentTime - holdTimer[i] > WAIT_DURATION) button[i] = TAP2_R; break;
        case TAP3_W: if (currentTime - holdTimer[i] > WAIT_DURATION) button[i] = TAP3_R; break;
        case RELEASE:
        case RELEASE_S:
        case RELEASE_L:
        case TAP2_R:
        case TAP3_R: button[i] = NONE; break;
        default: button[i] = NONE; break;
      }
    }
  }
}

/* =============================================================================
 *                              IR Compound Eye
 * ============================================================================= */

uint8_t PeanutKingSoccerV4::compoundMaxEye() {
  rxBuff[0] = 0;
  I2CSensorRead(senbrdHandle, 13, 1);
  return rxBuff[0];
}

uint8_t PeanutKingSoccerV4::compoundMaxEyeVal() {
  rxBuff[0] = 0;
  I2CSensorRead(senbrdHandle, 12, 1);
  return rxBuff[0];
}

uint8_t PeanutKingSoccerV4::compoundEyeVal(uint8_t n) {
  rxBuff[0] = 0;
  I2CSensorRead(senbrdHandle, n, 1);
  return rxBuff[0];
}

uint8_t* PeanutKingSoccerV4::compoundEyeRead() {
  for (uint8_t i=0; i<12; i++) rxBuff[i] = 0;
  I2CSensorRead(senbrdHandle, IR_RAW, 12);
  for (uint8_t i=0; i<12; i++) { eye[i] = rxBuff[i]; }
  return eye;
}

void PeanutKingSoccerV4::compoundEyeCal(float* calData) {
  uint8_t msg[25] = {0};
  uint16_t eyeCal[12] = {0};
  msg[0] = IR_CAL;
  for (uint8_t i=0; i<12; i++) eyeCal[i] = 4096.0 / calData[i];
  for (uint8_t i=0; i<12; i++) {
    msg[2*i+1] = eyeCal[i] & 0xff;
    msg[2*i+2] = eyeCal[i] >> 8;
  }
  gIIC->Write(senbrdHandle, msg, 25);
}

/* =============================================================================
 *                              Ultrasonic
 * ============================================================================= */

uint16_t PeanutKingSoccerV4::ultrasonicRead(uint8_t n) {
  if (millis() - ULT_get_interval < 30) return ultrasonic[n];
  ultra_send_seq = (ultra_send_seq >= 3) ? 0 : ultra_send_seq + 1;
  digitalWrite(ULTPin_trig[ultra_send_seq], LOW);
  delayMicroseconds(2);
  digitalWrite(ULTPin_trig[ultra_send_seq], HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTPin_trig[ultra_send_seq], LOW);
  ULT_get_interval = millis();
  return ultrasonic[n];
}

void PeanutKingSoccerV4::ULT_Echo_dect(uint8_t n) {
  if (ultra_send_seq != n) return;
  if (digitalRead(ULTPin_echo[n])) {
    ULT_dt[n] = micros();
  } else {
    ULT_dt[n] = micros() - ULT_dt[n];
  }
  float dist = (float)ULT_dt[n] * 0.17f;
  if (dist < 4500) ultrasonic[n] = (uint16_t)round(dist);
}

/* =============================================================================
 *                              Color Sensor (via Sensor Board)
 * ============================================================================= */

void PeanutKingSoccerV4::setColorBL(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  txBuff[0] = COLOR_BL;
  for (uint8_t i=0; i<4; i++) {
    uint8_t *p = &txBuff[1 + i*4];
    p[0] = r; p[1] = g; p[2] = b; p[3] = w;
  }
  gIIC->Write(senbrdHandle, txBuff, 17);
}

uint16_t PeanutKingSoccerV4::getRedColor(uint8_t i) {
  const uint8_t ci[4] = {0, 3, 1, 2};
  for (uint8_t j=0; j<2; j++) rxBuff[j] = 0;
  I2CSensorRead(senbrdHandle, COLOR_RAW+ci[i]*8, 2);
  colorRGB[i].r = rxBuff[0] | rxBuff[1]<<8;
  return colorRGB[i].r;
}

uint16_t PeanutKingSoccerV4::floorColorRead(uint8_t i) {
  const uint8_t ci[4] = {0, 3, 1, 2};
  for (uint8_t j=0; j<6; j++) rxBuff[j] = 0;
  I2CSensorRead(senbrdHandle, COLOR_RAW+ci[i]*8, 6);
  colorRGB[i].r = rxBuff[0] | rxBuff[1]<<8;
  colorRGB[i].g = rxBuff[2] | rxBuff[3]<<8;
  colorRGB[i].b = rxBuff[4] | rxBuff[5]<<8;
  return colorRGB[i].r + colorRGB[i].g + colorRGB[i].b;
}

uint16_t PeanutKingSoccerV4::whiteLineCal(uint8_t pin_no) {
  whiteLineThreshold[pin_no] = getColorSensorHSL(pin_no).h;
  return whiteLineThreshold[pin_no];
}

bool PeanutKingSoccerV4::whiteLineCheck(uint8_t i, uint16_t thresh) {
  colorHSL[i] = getColorSensorHSL(i);
  isWhite[i] = (abs((int)colorHSL[i].h - (int)thresh) < 10 && colorHSL[i].l >= 50);
  return isWhite[i];
}

/* =============================================================================
 *                              LED
 * ============================================================================= */

void PeanutKingSoccerV4::setOnBrdLED(uint8_t color) {
  digitalWrite(ledPin[0], color & 1);
  digitalWrite(ledPin[1], color & 2);
  digitalWrite(ledPin[2], color & 4);
}

void PeanutKingSoccerV4::setOnBrdLED(uint8_t LED, uint8_t status) {
  digitalWrite(ledPin[LED], status);
}

/* =============================================================================
 *                              TFT Display
 * ============================================================================= */

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

/* =============================================================================
 *                              Bluetooth
 * ============================================================================= */

void PeanutKingSoccerV4::enableScanning(bool enable, uint16_t sensorType, bool enableLED) {
  (void)enable; (void)sensorType; (void)enableLED;
}

void PeanutKingSoccerV4::bluetoothAttributes() {}
void PeanutKingSoccerV4::bluetoothRemote(void) {}

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
//   // I2CSensorSend(senbrdHandle,LED_RGB,l,12);

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

// uint8_t PeanutKingSoccerV4::colorReadAll(void) {
//   const uint8_t ci[4] = {0, 3, 1, 2};

//   for (uint8_t i=0; i<32; i++)    rxBuff[i] = 0;
//   I2CSensorRead(senbrdHandle, COLOR_RAW, 32);
//   for (uint8_t i=0; i<4; i++) {
//     colorRGB[ci[i]].r  = rxBuff[8*i]   | rxBuff[8*i+1]<<8;
//     colorRGB[ci[i]].g  = rxBuff[8*i+2] | rxBuff[8*i+3]<<8;
//     colorRGB[ci[i]].b  = rxBuff[8*i+4] | rxBuff[8*i+5]<<8;
//   }
//     groundColor[i] = rxBuff[24+i];

//   for (uint8_t i=0; i<16; i++)    rxBuff[i] = 0;
//   I2CSensorRead(senbrdHandle, COLOR_HSL, 16);
//   for (uint8_t i=0; i<4; i++) {
//     colorHSL[ci[i]].h  = rxBuff[4*i] | rxBuff[4*i+1]<<8;
//     colorHSL[ci[i]].s  = rxBuff[4*i+2];
//     colorHSL[ci[i]].l  = rxBuff[4*i+3];
//   }

//   for (uint8_t i=0; i<16; i++)    rxBuff[i] = 0;
//   I2CSensorRead(senbrdHandle, COLOR_HSV, 16);
//   for (uint8_t i=0; i<4; i++) {
//     colorHSV[ci[i]].h  = rxBuff[4*i] | rxBuff[4*i+1]<<8;
//     colorHSV[ci[i]].s  = rxBuff[4*i+2];
//     colorHSV[ci[i]].v  = rxBuff[4*i+3];
//   }
//   for (uint8_t i=0; i<28; i++)    rxBuff[i] = 0;
//   I2CSensorRead(senbrdHandle, COLOR_RAW, 28);
//   for (uint8_t i=0; i<4; i++) {
//     colorRGB[i].r  = rxBuff[6*i]   | rxBuff[6*i+1]<<8;
//     colorRGB[i].g  = rxBuff[6*i+2] | rxBuff[6*i+3]<<8;
//     colorRGB[i].b  = rxBuff[6*i+4] | rxBuff[6*i+5]<<8;
//     groundColor[i] = rxBuff[24+i];
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