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
  maxEye    = compoundEye.getMaxEye();
  maxEyeVal = compoundEye.getMaxEyeVal();
  eyeAngle  = compoundEye.getAngle();

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


bool PeanutKingSoccerV4::whiteLineCheck(CLR_SENSOR_ID i) {
  return colorSensor.isWhiteLine(i);
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
int16_t* PeanutKingSoccerV4::getGyroscopeRaw(void)     { return compass.getGyroscopeRaw(); }
int16_t* PeanutKingSoccerV4::getMagnetometerRaw(void)  { return compass.getMagnetometerRaw(); }

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

bool PeanutKingSoccerV4::_sendPILAData(void) {
  char buf[64];
  // Format: "soccer,<compass>,<ultrasoundFront>,<ultrasoundBack>,<ultrasoundLeft>,<ultrasoundRight>,<maxEye>,<maxEyeValue>"
  snprintf(buf, sizeof(buf), "soccer,%d,%d,%d,%d,%d,%d,%d",
    heading, 
    ultrasonic[0], ultrasonic[2], ultrasonic[3], ultrasonic[1], 
    maxEye, maxEyeVal
  );
  return ble.sendData(String(buf));
}
void PeanutKingSoccerV4::_PILAUpdate(void)
{
  dataFetch();  // Fetch all sensor data
  // Send data to Peanut Queen every 100ms
  if (millis() - _lastSendTime > 100) {
    _sendPILAData();
    _lastSendTime = millis();
  }
  // Not yet implemented (parsing of incoming data from Peanut Queen)
}

void PeanutKingSoccerV4::_DASHBOARDUpdate(void)
{
  dataFetch();  // Fetch all sensor data
  // Send data to Peanut Dashboard every 100ms
  if (millis() - _lastSendTime > 100) {
    _lastSendTime = millis();
  }
  // Not yet implemented (parsing of incoming data from Peanut Dashboard)
}

void PeanutKingSoccerV4::bluetoothRemote(void) {
  // Check for connection status changes
  if (ble.checkConnection()) {
    // Handle the connection status change
    ble.handleConnection();
  }

  // If disconnected, skip parsing data
  if (!ble.isConnected()) { return; }

  // If connected, parse incoming data based on the current remote mode
  if (ble.getMode() == RemoteMode::PILA) {
    _PILAUpdate();
  } else if (ble.getMode() == RemoteMode::DASHBOARD) {
    _DASHBOARDUpdate();
  }
}

/* =============================================================================
 *                              Strategy Functions
 * ============================================================================= */

void PeanutKingSoccerV4::Chase(int& direct, int& speed, int& rotation) {
  (void)direct; (void)speed; (void)rotation;
}

void PeanutKingSoccerV4::Back(int& direct, int& speed, int& rotation) {
  (void)direct; (void)speed; (void)rotation;
}