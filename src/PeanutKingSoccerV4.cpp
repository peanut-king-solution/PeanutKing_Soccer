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
  pwmPin{10, 11, 12, 13}
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
  button.init();

  // Initialize LED module
  led.init();

  // Initialize ultrasonic module
  ultrasound.init();
  
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
  for (uint8_t pos = Front; pos < PositionCount; pos++) {
    // Color sensor - RGBC/RGB/HSL/white line by physical position
    colorRGBC[pos] = colorSensorReadRGBC(pos);
    colorRGB[pos]  = colorSensorReadRGB(pos);
    colorHSL[pos]  = colorSensorReadHSL(pos);
    isWhite[pos]   = isWhiteLine(pos);

    // Ultrasound - distance by physical position
    distances[pos] = ultrasoundGetDist(pos);
  }

  // Compound eye
  compoundEyeReadAll();  // Read all 12 IR sensor values
  maxEye    = compoundMaxEyeRead();       // Read the index of the IR sensor with maximum reading
  maxEyeVal = compoundMaxEyeValueRead();  // Read the maximum IR sensor value
  irAngle   = compoundEyeAngleRead();     // Read the angle of the detected object
  
  // Compass
  heading = compassRead();
}

/* =============================================================================
 *                              Motor (wrapper)
 * ============================================================================= */

  void PeanutKingSoccerV4::motorConfiguration(MotorId RightFront, MotorId RightBack, MotorId LeftBack, MotorId LeftFront) {
    motor.mapPort(RightFront, RightBack, LeftBack, LeftFront);
  }

  void PeanutKingSoccerV4::motorFlipDirection(MotorPos pos, bool flip) {
    MotorId mi = motor.getPortFromPos(pos);
    motor.flipDirection(mi, flip);
  }

  void PeanutKingSoccerV4::motorSetSpeed(MotorPos pos, int16_t speed) {
    MotorId mi = motor.getPortFromPos(pos);
    motor.setSpeed(mi, speed);
  }

  void PeanutKingSoccerV4::motorStop(MotorPos pos) {
    MotorId mi = motor.getPortFromPos(pos);
    motor.stop(mi);
  }

  void PeanutKingSoccerV4::motorStopAll(void) {
    for (uint8_t i = 0; i < 4; i++) {
      MotorId mi = static_cast<MotorId>(i);
      motor.stop(mi);
    }
  }

  void PeanutKingSoccerV4::motorTestAll(int16_t speed, int duration) {
    for (uint8_t i = 0; i < 4; i++) {
      MotorPos pos = static_cast<MotorPos>(i);
      motorSetSpeed(pos, speed);
      delay(duration);
      motorStop(pos);
      delay(duration);
    }
    motorStopAll();
    delay(duration);
  }

/* =============================================================================
*                              Movement (wrapper)
* ============================================================================= */
  
  void PeanutKingSoccerV4::move(float mAngle, float mSpeed, float rotate) {
    uint16_t compassReading = 65535; // Default to invalid value
    // If compass correction is enabled, read the compass heading
    if (compassCorrectEnabled) { compassReading = compassRead(); }
    // Check out-of-bounds status using color sensors (only when prevention is enabled)
    bool isOutBound[4] = {false, false, false, false};
    if (outBoundPreventEnabled) {
      isOutBound[0] = this->isWhiteLine(Front); // Front
      isOutBound[1] = this->isWhiteLine(Right); // Right
      isOutBound[2] = this->isWhiteLine(Back); // Back
      isOutBound[3] = this->isWhiteLine(Left); // Left
    }

    // Determine movement method based on enabled features
    WheelSpeeds ws = {0, 0, 0, 0}; // Initialize wheel speeds (RF, RB, LB, LF)
    // If both compass correction and out-of-bounds prevention are enabled, use correctedMove
    if (compassCorrectEnabled && outBoundPreventEnabled) {
      ws = movement.correctedMove(mAngle, mSpeed, compassReading, isOutBound);
    }
    // If only compass correction is enabled, use withCorr
    else if (compassCorrectEnabled) {
      ws = movement.withCorr(mAngle, mSpeed, compassReading);
    }
    // If only out-of-bounds prevention is enabled, use outBoundPrevent
    else if (outBoundPreventEnabled) {
      ws = movement.outBoundPrevent(mAngle, mSpeed, isOutBound);
    }
    // If neither feature is enabled, move without any corrections
    else {
      ws = movement.byAngle(mAngle, mSpeed, rotate);
    }
    // Apply the computed speeds by mapping physical position to motor port
    motorSetSpeed(RightFront, ws.rightFront);
    motorSetSpeed(RightBack,  ws.rightBack);
    motorSetSpeed(LeftBack,   ws.leftBack);
    motorSetSpeed(LeftFront,  ws.leftFront);
  }

  void PeanutKingSoccerV4::moveTest(float speed) {
    move(0, speed, 0);
    delay(1000);
    move(45, speed, 0);
    delay(1000);
    move(90, speed, 0);
    delay(1000);
    this->motorStopAll();
    delay(500);
  }

/* =============================================================================
 *                              Color Sensor (soft I2C)
 * ============================================================================= */

void PeanutKingSoccerV4::colorSensorConfiguration(ColorSensorId Front, ColorSensorId Right, ColorSensorId Back, ColorSensorId Left) {
  colorSensor.mapPort(Front, Right, Back, Left);
}

RGBC PeanutKingSoccerV4::colorSensorReadRGBC(SensorPos pos) {
  return colorSensor.readRGBRaw(colorSensor.getPortFromPos(pos));
}

RGB PeanutKingSoccerV4::colorSensorReadRGB(SensorPos pos) {
  return colorSensor.readRGB(colorSensor.getPortFromPos(pos));
}

HSL PeanutKingSoccerV4::colorSensorReadHSL(SensorPos pos) {
  return colorSensor.readHSL(colorSensor.getPortFromPos(pos));
}

bool PeanutKingSoccerV4::isWhiteLine(SensorPos pos) {
  return colorSensor.isWhiteLine(colorSensor.getPortFromPos(pos));
}

void PeanutKingSoccerV4::colorSensorCalBaseline(SensorPos pos, uint8_t samples) {
  colorSensor.calBaseline(colorSensor.getPortFromPos(pos), samples);
}
GreenBaseline PeanutKingSoccerV4::colorSensorGetBaseline(SensorPos pos) {
  return colorSensor.getBaseline(colorSensor.getPortFromPos(pos));
}

/* =============================================================================
 *                       IR Compound Eye (wrapper)
 * ============================================================================= */

uint8_t* PeanutKingSoccerV4::compoundEyeReadAll() {
  uint8_t* eyePtr = compoundEye.readAll();
  for (uint8_t i = 0; i < 12; i++) {
    eyes[i] = eyePtr[i];
  }
  return eyes;
}

EyeId PeanutKingSoccerV4::compoundMaxEyeRead() {
  return compoundEye.readMaxEye();
}

uint8_t PeanutKingSoccerV4::compoundMaxEyeValueRead() {
  return compoundEye.readMaxEyeVal();
}

uint8_t PeanutKingSoccerV4::compoundEyeValueRead(EyeId eyeIndex) {
  return compoundEye.readEyeVal(eyeIndex);
}

uint16_t PeanutKingSoccerV4::compoundEyeAngleRead(void) {
  return compoundEye.readAngle();
}

uint8_t PeanutKingSoccerV4::compoundEyeModeRead(void) {
  return compoundEye.readMode();
}

/* =============================================================================
 *                       Button (wrapper for compatibility)
 * ============================================================================= */

void PeanutKingSoccerV4::buttonUpdate(void) {
  button.update();
}

ButtonState PeanutKingSoccerV4::buttonStateRead(ButtonId btn) {
  return button.readState(btn);
}

/* =============================================================================
 *                       LED (wrapper for compatibility)
 * ============================================================================= */

void PeanutKingSoccerV4::onBoardLedSet(LEDColor color) {
  led.setLED(color);
}

void PeanutKingSoccerV4::onBoardLedSet(uint8_t LED, uint8_t status) {
  led.setLED(LED, status);
}

/* =============================================================================
 *                       Ultrasound (wrapper)
 * ============================================================================= */

uint16_t PeanutKingSoccerV4::ultrasoundGetDist(SensorPos pos) {
  UltrasoundId port = ultrasound.getPortFromPos(pos);
  return ultrasound.read(port);
}

void PeanutKingSoccerV4::ultrasoundConfiguration(UltrasoundId Front, UltrasoundId Right, UltrasoundId Back, UltrasoundId Left) {
  ultrasound.mapPort(Front, Right, Back, Left);
}

void PeanutKingSoccerV4::ultrasoundSetEnabled(bool front, bool right, bool back, bool left) {
  ultrasound.enable(ultrasound.getPortFromPos(Front), front);
  ultrasound.enable(ultrasound.getPortFromPos(Right), right);
  ultrasound.enable(ultrasound.getPortFromPos(Back),  back);
  ultrasound.enable(ultrasound.getPortFromPos(Left),  left);
}

void PeanutKingSoccerV4::ultrasoundEnableAll(bool enabled) {
  ultrasound.setEnableMask(enabled ? 0x0F : 0x00);
}

/* =============================================================================
 *                              Compass (wrapper)
 * ============================================================================= */

uint16_t PeanutKingSoccerV4::compassRead(void)         { return compass.read(); }
int16_t* PeanutKingSoccerV4::compassReadRawAccel(void) { return compass.readRawAccel(); }
int16_t* PeanutKingSoccerV4::compassReadRawGyro(void)  { return compass.readRawGyro(); }
int16_t* PeanutKingSoccerV4::compassReadRawMag(void)   { return compass.readRawMag(); }

void PeanutKingSoccerV4::compassUpdateNorth(void) { compass.updateNorthOffset(); }

/* =============================================================================
 *                              TFT Display
 * ============================================================================= */

void PeanutKingSoccerV4::screenSetTextColor(uint16_t color) {
  tft.setTextColor(color);
}

void PeanutKingSoccerV4::screenSetTextColor(uint16_t fg, uint16_t bg) {
  tft.setTextColor(fg, bg);
}

void PeanutKingSoccerV4::screenSetTextSize(uint8_t size) {
  tft.setTextSize(size);
}

void PeanutKingSoccerV4::screenPrintText(uint8_t col, uint8_t row, const char* string) {
  tft.setCursor(col*6, row*10);
  tft.print(string);
}

void PeanutKingSoccerV4::screenPrintNumber(uint8_t col, uint8_t row, int16_t number) {
  tft.setCursor(col*6, row*10);
  tft.print(number);
}

void PeanutKingSoccerV4::screenClear(void) {
  tft.fillScreen(ST7735_BLACK);
}

void PeanutKingSoccerV4::screenDrawAnglePointer(int x, int y, int radius, uint16_t angle, uint16_t arrowColor) {
  // Normalize angle to [0, 360) range
  angle = angle % 360;
  
  // Clear previous indicator circle
  tft.fillCircle(x, y, radius + 2, ST7735_BLACK);

  // Draw compass circle outline
  tft.drawCircle(x, y, radius, ST7735_WHITE);

  // Draw N/S/E/W markers
  tft.setTextSize(1);
  tft.setTextColor(ST7735_RED);
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

bool PeanutKingSoccerV4::bluetoothInit(HardwareSerial* port, RemoteMode mode) { return bluetooth.init(port, mode); }
bool PeanutKingSoccerV4::bluetoothRename(const char* name) { return bluetooth.rename(name); }
void PeanutKingSoccerV4::bluetoothSetConfig(const String& configMessage) { bluetooth.setConfig(configMessage); }
bool PeanutKingSoccerV4::bluetoothReset(void) { return bluetooth.reset(); }

bool PeanutKingSoccerV4::bluetoothIsConnected(void) {
  return bluetooth.isConnected();
}
void PeanutKingSoccerV4::bluetoothRemote(void) {
  // process Bluetooth serial data and commands
  bluetooth.processSerial();

  // check connection status to app and handle disconnection
  // if (!bluetooth.isConnected()) {
    
  // }

  // Process all pending commands from the queue
  RemoteMode mode = bluetooth.getMode();
  // TODO:
  // while (bluetooth.hasCommand()) {
  //   RxCommand cmd = bluetooth.getCommand();
  //   // Handle commands of PILA mode
  //   if (mode == PILA_LEGACY || mode == PILA_CONFIG) {
  //     // _handlePILACommand(cmd);
  //   } 
  //   // Handle commands of DASHBOARD mode
  //   else {
  //     // _handleDashboardCommand(cmd);
  //   }
  // }

  // Sending telemetry data of legacy mode
  // format: Soccer,<compass>,<ultrasound_front>,<ultrasound_back>,<ultrasound_back>,<ultrasound_left>,<ultrasound_right>, <max_eye>,<max_eye_value>>
}

void PeanutKingSoccerV4::bluetoothSetOutput(const String& name, int value) {}
void PeanutKingSoccerV4::bluetoothSetOutput(const String& name, float value) {}
void PeanutKingSoccerV4::bluetoothSetOutput(const String& name, bool value) {}

bool PeanutKingSoccerV4::bluetoothGetToggle(const String& name) {
  return bluetooth.getToggleState(name);
}
int PeanutKingSoccerV4::bluetoothGetSlider(const String& name) {
  return bluetooth.getSliderValue(name);
}
String PeanutKingSoccerV4::bluetoothGetTextField(const String& name) {
  return bluetooth.getTextFieldValue(name);
}
JoystickState PeanutKingSoccerV4::bluetoothGetJoystick(const String& name) {
  return bluetooth.getJoystick(name);
}

void PeanutKingSoccerV4::bluetoothOnButton(const String& name, ButtonCallback callback) {
  bluetooth.onButton(name, callback);
}

/* =============================================================================
 *                              PS2 Controller
 * ============================================================================= */

byte PeanutKingSoccerV4::ps2Init(DigitalPinId CLK, DigitalPinId DAT, bool pressure, bool vibration) {
  uint8_t PS2_CLK_PIN = static_cast<uint8_t>(CLK);
  uint8_t PS2_DAT_PIN = static_cast<uint8_t>(DAT);

  // there must be 2 pins between CLK and DAT
  if (abs(PS2_CLK_PIN - PS2_DAT_PIN) != 3) {
    // Invalid PS2_CLK_PIN or PS2_DAT_PIN configuration, return error code
    return 0xFF; // Error code for invalid pin configuration
  }

  uint8_t PS2_CMD_PIN = 0, PS2_ATT_PIN = 0;
  // Determine CMD and ATT pins based on CLK and DAT pins
  if (PS2_CLK_PIN > PS2_DAT_PIN) {  // like clk is D1_P and dat is D4_P
    PS2_ATT_PIN = PS2_CLK_PIN - 1; // ATT pin is in CLK right
    PS2_CMD_PIN = PS2_DAT_PIN + 1; // CMD pin is in DAT left
  } else if (PS2_CLK_PIN < PS2_DAT_PIN) {  // like clk is D6_P and dat is D3_P
    PS2_ATT_PIN = PS2_CLK_PIN + 1; // ATT pin is in CLK left
    PS2_CMD_PIN = PS2_DAT_PIN - 1; // CMD pin is in DAT right
  }
  byte error = ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_ATT_PIN, PS2_DAT_PIN, pressure, vibration);
  return error;
}
void PeanutKingSoccerV4::ps2SetVibration(byte strength) {
  vibrationStr = strength;
}
void PeanutKingSoccerV4::ps2Update(void) {
  ps2x.read_gamepad(false, vibrationStr);
}
PS2ButtonState PeanutKingSoccerV4::ps2ButtonStateRead(PS2Button button) {
  uint16_t btn = static_cast<uint16_t>(button);
  if (ps2x.ButtonPressed(btn))      { return PS2Pressed; }
  if (ps2x.Button(btn))             { return PS2Holding; }
  if (ps2x.ButtonReleased(btn))     { return PS2Released; }
  return PS2Idle;
}
PS2JoystickData PeanutKingSoccerV4::ps2JoystickRead(PS2Joystick joystick) {
  // Prepare the data structure to hold the results
  PS2JoystickData data;
  byte x = (joystick == PS2LeftJoystick) ? ps2x.Analog(PSS_LX) : ps2x.Analog(PSS_RX);
  byte y = (joystick == PS2LeftJoystick) ? ps2x.Analog(PSS_LY) : ps2x.Analog(PSS_RY);

  const float CX = 128.0;   // X-axis center point (for LX and RX)
  const float CY = 127.0;   // Y-axis center point (for LY and RY)
  float dx = x - CX;
  float dy = y - CY;

  // Calculate angle in degrees (0-360)
  data.angle = atan2(dx, -dy) * (180.0 / PI);
  if (data.angle < 0) {
    data.angle += 360;   // Normalize to [0, 360)
  }

  // Calculate strength (0-255) based on distance from center
  float raw_strength = sqrt(dx * dx + dy * dy);
  // The following commented-out code was an initial attempt to scale the strength based on a circular boundary, 
  // but it has been replaced with a square boundary scaling method for better control and consistency.
  // const float MAX_R = sqrt(CX * CX + CY * CY); // = sqrt(128^2 + 127^2) ≈ 180.31
  // data.strength = (raw_strength / MAX_R) * 255.0;
  // if (data.strength > 255.0) {
  //   data.strength = 255.0; // Cap at 255
  // }
  // Deadzone for very small movements
  if (raw_strength < 0.5) { data.strength = 0; }
  // Scale strength to fit within a square boundary instead of a circular one
  else {
    float norm_x = dx / 128.0; // Normalize to [-1, 1]
    float norm_y = dy / 127.0; // Normalize to [-1, 1]
    // Find the maximum absolute value to determine the scaling factor
    float max_abs = max(fabs(norm_x), fabs(norm_y));
    if (max_abs > 1.0) { max_abs = 1.0; } // Cap at 1.0
    // Scale to the boundary of the square
    float boundary_r = raw_strength / max_abs;
    data.strength = raw_strength / boundary_r * 255.0;    // Scale to [0, 255]
    if (data.strength > 255.0) { data.strength = 255.0; } // Cap at 255
  }
  return data;
}