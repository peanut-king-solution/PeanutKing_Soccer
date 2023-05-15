/*
 * Copyright (c) 2022 PeanutKing Solution
 *
 * @file        PeanutKingSoccerV3.cpp
 * @summary     Soccer Robot V3 Library
 * @version     3.1
 * @author      Jack Kwok
 * @data        26 July 2022
 */

#include "PeanutKingSoccerV3.h"

static PeanutKingSoccerV3* V3bot = NULL;

PeanutKingSoccerV3::PeanutKingSoccerV3(void) :
  actledPin(37),
  topBoardAddr(10),
  sensorBoardAddr(12),

  buttonPin{36, 35, 34},        // mainboard V3.3-3.4
  inhPin  { 4,  7, 10, 13},     // long high  timer 0 (controls pin 13, 4);
  in1Pin  { 2,  5,  8, 11},     // timer 1 (controls pin 12, 11);
  in2Pin  { 3,  6,  9, 12},     // timer 2 (controls pin 10, 9);
  diagPin {50, 51, 53, 53} {    // timer 3 (controls pin 5, 3, 2);
  if (V3bot == NULL)  {         // timer 4 (controls pin 8, 7, 6);
    V3bot = this;
  }
}

ISR(TIMER1_COMPB_vect) {
  if (V3bot != NULL) {
    V3bot->timerLoop();
  }
}

void PeanutKingSoccerV3::timerLoop(void) {
  // dataFetch();
  tim1Count++;
  
  if (tim1Count%10 == 1) {
    buttons();
    if (button[0]==RELEASE || button[0]==RELEASE_S || button[0]==RELEASE_L) {
      motorEnabled = !motorEnabled;
      // digitalWrite(actledPin, LOW);
    }
  }
  
  if (tim1Count%2 == 1) {
    for (uint8_t i=0; i<4; i++) {
      motorUpdate(i);
    }
  }
}

// initialize all IOs, Serial.begin, I2C, timer interrupt, 
// External interrupt different settings depends on version number 
void PeanutKingSoccerV3::init(uint8_t mode) {
  Serial.begin(115200);
  Serial1.begin(9600);

  for (uint8_t i=0; i<4; i++) {
    pinMode(inhPin[i],  OUTPUT);
    pinMode(in1Pin[i],  OUTPUT);
    pinMode(in2Pin[i],  OUTPUT);
    pinMode(diagPin[i], OUTPUT);
    digitalWrite(inhPin[i], HIGH);
    digitalWrite(diagPin[i], HIGH);
  }
  for (uint8_t i=0; i<3; i++)
    pinMode(buttonPin[i], INPUT);
  
  pinMode(actledPin, OUTPUT);
  digitalWrite(actledPin, HIGH);
  
  delay(10);

  compssHandle = gIIC->RegisterDevice(compass_address, 1, IICIT::Speed::SLOW);
  senbrdHandle = gIIC->RegisterDevice(sensorBoardAddr, 1, IICIT::Speed::SLOW);
  topbrdHandle = gIIC->RegisterDevice(topBoardAddr, 1, IICIT::Speed::SLOW);
  lcdScrHandle = gIIC->RegisterDevice(LCD_Addr, 1, IICIT::Speed::SLOW);

  // uint8_t msg[1] = {LCD_backlightval};// reset expander and turn backlight off (Bit 8 =1)
  // uint8_t _status = gIIC->Write(lcdScrHandle, msg, 1);
  // delay(10);
  // if ( gIIC->getStatus() != 0 ) {
  //   lcdScrHandle = gIIC->RegisterDevice(0x38, 1, IICIT::Speed::SLOW);
  // }
  delay(200);

  lcdSetup();
  delay(200);
  cli();    //disable interrupts
  /*
  //   Timer 1
  TCCR1A  = 0x00;           // Normal mode, just as a Timer
  TCNT1   = 0;
  OCR1A   = 624;             // 8 * 4 / 16 = 2us
  // OCR1A = 1250;       // =(16*10^6) / (125*256) -1 (must be <65536)
  OCR1A   = 100;            // Hz = ?
  
  TCCR1B = (1 << WGM12);    // CTC mode; Clear Timer on Compare
  // TCCR1B |= (1 << CS11); // Set CS#1 bit for 8 prescaler for timer 1
  TCCR1B |= (1 << CS10) | (1 << CS11);    // CLK i/o /64 (From Prescaler)  (must be <65536)
  // TCCR1B |= (1 << CS12);    // prescaler = 256
  // TCCR1B |= (1 << CS10) | (1 << CS12);    // prescaler = 1024
  */
  TCNT1   = 0;
  // TCCR1B  = (1 << WGM12);   // CTC mode; Clear Timer on Compare
  // TCCR1B &= ~(1 << CS10);
  // TCCR1B |= (1 << CS11);    // Set CS#1 bit for 8 prescaler for timer 1
  TIMSK1 |= (1 << OCIE1B);  // enable timer compare interrupt

  sei();    //allow interrupts

  //while ( compassRead() == 400 );

  // uint8_t l[12];
  // for (uint8_t i=0; i<4; i++) {
  //   if ( n & (1<<i) ) {
  //     uint8_t *p = &l[i * 3];  // 4 bytes per pixel
  //     p[0] = 200;                   // R
  //     p[1] = 200;                   // G
  //     p[2] = 200;                   // B
  //   }
  // }
  // I2CSensorSend(senbrdHandle,LED_RGB,l,12);

  delay(1500);
}


/* =============================================================================
 *                                  Data fetch
 * ============================================================================= */
void PeanutKingSoccerV3::dataFetch(void) {
  // int16_t temp;
  // Serial.print("A: ");
  // for (uint8_t i=0; i<6; i++)     rxBuff[i] = 0;
  // I2CSensorRead(8, ACC_RAW, 6);
  // for (uint8_t i=0; i<3; i++) {
  //   temp  = rxBuff[2*i] & 0xff;
  //   temp |= rxBuff[2*i+1] << 8;
  //   Serial.print(temp);   Serial.print(' ');
  // }
  // Serial.print(" G: ");
  // for (uint8_t i=0; i<6; i++)     rxBuff[i] = 0;
  // I2CSensorRead(8, GYR_RAW, 6);
  // for (uint8_t i=0; i<3; i++) {
  //   temp  = rxBuff[2*i] & 0xff;
  //   temp |= rxBuff[2*i+1] << 8;
  //   Serial.print(temp);   Serial.print(' ');
  // }
  // Serial.print(" M: ");
  // for (uint8_t i=0; i<6; i++)     rxBuff[i] = 0;
  // I2CSensorRead(8, MAG_BUF, 6);
  // for (uint8_t i=0; i<3; i++) {
  //   temp  = rxBuff[2*i] & 0xff;
  //   temp |= rxBuff[2*i+1] << 8;
  //   Serial.print(temp);   Serial.print(' ');
  // }
  // Serial.println(' ');

  for (uint8_t i=0; i<2; i++)     rxBuff[i] = 0;
  I2CSensorRead(compssHandle, GET_YAW, 2);
  compass  = rxBuff[0] & 0xff;
  compass |= rxBuff[1] << 8;
  compass = compass/100;

  for (uint8_t i=0; i<8; i++)     rxBuff[i] = 0;
  I2CSensorRead(topbrdHandle, ULT_DATA, 8);
  for (uint8_t i=0; i<4; i++) {
    ultrasonic[i]  = rxBuff[2*i] & 0xff;
    ultrasonic[i] |= rxBuff[2*i+1] << 8;
  }

  //sensorBoardAddr
  for (uint8_t i=0; i<27; i++)    rxBuff[i] = 0;
  I2CSensorRead(senbrdHandle, IR_RAW, 27);
  for (uint8_t i=0; i<12; i++) {
    eye[i]  = rxBuff[2*i] & 0xff;
    eye[i] |= rxBuff[2*i+1] << 8;
  }
  maxEye = rxBuff[24];

  for (uint8_t i=0; i<28; i++)    rxBuff[i] = 0;
  I2CSensorRead(senbrdHandle, COLOR_RAW, 28);
  for (uint8_t i=0; i<4; i++) {
    colorRGB[i].r  = rxBuff[6*i]   | rxBuff[6*i+1]<<8;
    colorRGB[i].g  = rxBuff[6*i+2] | rxBuff[6*i+3]<<8;
    colorRGB[i].b  = rxBuff[6*i+4] | rxBuff[6*i+5]<<8;
    groundColor[i] = rxBuff[24+i];
  }

  for (uint8_t i=0; i<16; i++)    rxBuff[i] = 0;
  I2CSensorRead(senbrdHandle, COLOR_HSL, 16);
  for (uint8_t i=0; i<4; i++) {
    colorHSL[i].h  = rxBuff[4*i] | rxBuff[4*i+1]<<8;
    colorHSL[i].s  = rxBuff[4*i+2];
    colorHSL[i].l  = rxBuff[4*i+3];
  }

  for (uint8_t i=0; i<16; i++)    rxBuff[i] = 0;
  I2CSensorRead(senbrdHandle, COLOR_HSV, 16);
  for (uint8_t i=0; i<4; i++) {
    colorHSV[i].h  = rxBuff[4*i] | rxBuff[4*i+1]<<8;
    colorHSV[i].s  = rxBuff[4*i+2];
    colorHSV[i].v  = rxBuff[4*i+3];
  }
}

void PeanutKingSoccerV3::I2CSensorRead(IICIT::Handle handle, uint8_t sensor, uint8_t length) {
  uint8_t _status;
  txBuff[0] = sensor;
  _status = gIIC->Write(handle, txBuff, 1);
  _status = gIIC->Read(handle, rxBuff, length);
  // I2CSend(addr, txBuff, 1);
  // I2CRead(addr, rxBuff, length);
}

void PeanutKingSoccerV3::I2CSensorSend(IICIT::Handle handle, uint8_t sensor, uint8_t *data, uint8_t length) {
  uint8_t _status;
  txBuff[0] = sensor;
  // if (rxBuff[0]!=2) return;

  for (uint8_t i=0; i<length; i++) {
    txBuff[i+1] = data[i];
  }
  _status = gIIC->Write(handle, txBuff, length+1);
  // I2CSend(addr, txBuff, length+1);
} 

IICIT::status_t PeanutKingSoccerV3::LCDCallback(const IICIT::status_t status) {
  return status;
}

IICIT::status_t PeanutKingSoccerV3::rxCpltCallback(const IICIT::status_t status) {
  return status;
}

// void PeanutKingSoccerV3::I2CSend(int8_t addr, uint8_t *data, uint8_t length) {
//   Wire.beginTransmission(addr);
//   Wire.write(data, length);
//   Wire.endTransmission();
// }

// void PeanutKingSoccerV3::I2CRead(int8_t addr, uint8_t *data, uint8_t length) {
//   uint8_t i=0;
//   Wire.requestFrom((int)addr, (int)length);
//   while (Wire.available()) {
//     data[i++] = Wire.read();
//   }
// }


/* =============================================================================
 *                                  Sensor Read
 * ============================================================================= */
bool PeanutKingSoccerV3::buttonRead(uint8_t button_no) {
  if ( button_no == 1 || button_no == 2 ) 
    return !digitalRead(buttonPin[button_no]);
  else
    return 0;
}

uint16_t PeanutKingSoccerV3::compassRead(void) {
  for (uint8_t i=0; i<2; i++)     rxBuff[i] = 0;
  I2CSensorRead(compssHandle, GET_YAW, 2);
  compass  = rxBuff[0] & 0xff;
  compass |= rxBuff[1] << 8;
  compass = compass/100;
  return compass;
}

uint16_t PeanutKingSoccerV3::compoundEyeRead(uint8_t eye_no) {
  for (uint8_t i=0; i<27; i++)    rxBuff[i] = 0;
  I2CSensorRead(senbrdHandle, IR_RAW, 28);
  for (uint8_t i=0; i<12; i++) {
    eye[i+1]  = rxBuff[2*i] & 0xff;
    eye[i+1] |= rxBuff[2*i+1] << 8;
  }
  maxEye = rxBuff[24]+1;
  eyeAngle  = rxBuff[26] & 0xff;
  eyeAngle |= rxBuff[27] << 8;
  eyeAngle += 30;
  if (eyeAngle>=360) eyeAngle -= 360;
  
  if ( eye_no >0 && eye_no <= 12 )
    return eye[eye_no];
  else if ( eye_no == 13 ) 
    return maxEye;
  else if ( eye_no == 14 ) 
    return eye[maxEye];
  else 
    return 0;
}

uint16_t PeanutKingSoccerV3::eyeReadShort(void) {
  for (uint8_t i=0; i<4; i++)    rxBuff[i] = 0;
  I2CSensorRead(senbrdHandle, IR_MAX, 4);

  maxEye = rxBuff[0];
  eyeAngle  = rxBuff[2] & 0xff;
  eyeAngle |= rxBuff[3] << 8;
  eyeAngle += 30;
  if (eyeAngle>=360) eyeAngle -= 360;
  
  I2CSensorRead(senbrdHandle, IR_RAW+2*maxEye, 2);
  maxEye += 1;  // change from start=0 to start=1;
  eye[maxEye]  = rxBuff[0] & 0xff;
  eye[maxEye] |= rxBuff[1] << 8;

  return eye[maxEye];
}

void PeanutKingSoccerV3::compoundEyeCal(float* calData) {
  uint8_t _status;
  uint8_t msg[25] = {0};
  uint16_t eyeCal[12] = {0};
  msg[0] = IR_CAL;
  for (uint8_t i=0; i<12; i++) {
    eyeCal[i] = 4096.0 / calData[i];
  }
  for (uint8_t i=0; i<12; i++) {
    msg[2*i+1] = eyeCal[i] & 0xff;
    msg[2*i+2] = eyeCal[i] >> 8;
  }
  _status = gIIC->Write(senbrdHandle, msg, 25);
}

uint16_t PeanutKingSoccerV3::ultrasonicRead(uint8_t i) {
  const uint8_t ui[4] = {0, 3, 1, 2};
  //    0     3     1     2
  //  front left  right back
  if ( ui[i]<0 || ui[i]>=4 ) return 999;
  for (uint8_t i=0; i<2; i++)     rxBuff[i] = 0;
  I2CSensorRead(topbrdHandle, ULT_DATA+ui[i]*2, 2);
  ultrasonic[i]  = rxBuff[0] & 0xff;
  ultrasonic[i] |= rxBuff[1] << 8;
  return ultrasonic[i];
}

uint16_t PeanutKingSoccerV3::ultrasonicRead3(void) {
  const uint8_t ui[4] = {0, 3, 1, 2};
  //    0     3     1     2
  //  front left  right back
  for (uint8_t i=0; i<8; i++)     rxBuff[i] = 0;
  I2CSensorRead(topbrdHandle, ULT_DATA, 8);
  
  for (uint8_t i=0; i<4; i++) { 
    ultrasonic[i]  = rxBuff[ui[i]*2] & 0xff;
    ultrasonic[i] |= rxBuff[ui[i]*2+1] << 8;
  }
  return 1;
}

void PeanutKingSoccerV3::setLED(uint8_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  uint8_t _status;
  led[0] = LED_RGB;
  for (uint8_t i=0; i<8; i++) {
    if ( ((n>>i) & 0x01) == 1 ) {
      Serial.println(i);
      uint8_t *p = &led[1 + i*4];  // 4 bytes per pixel
      p[0] = r;                   // R
      p[1] = g;                   // G
      p[2] = b;                   // B
      p[3] = w;                   // W
    }
  }
  _status = gIIC->Write(topbrdHandle, led, 33);
}

void PeanutKingSoccerV3::setColorBL(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  uint8_t _status;
  txBuff[0] = COLOR_BL;
  for (uint8_t i=0; i<4; i++) {
    uint8_t *p = &txBuff[1 + i*4];  // 4 bytes per pixel
    p[0] = r;                   // R
    p[1] = g;                   // G
    p[2] = b;                   // B
    p[3] = w;                   // W
  }
  _status = gIIC->Write(senbrdHandle, txBuff, 17);
}


// void PeanutKingSoccerV3::sendLED(void) {
// }
// f l r b
// f b l r{


uint16_t PeanutKingSoccerV3::getRedColor(uint8_t i) {
  const uint8_t ci[4] = {0, 3, 1, 2};
  for (uint8_t i=0; i<2; i++)    rxBuff[i] = 0;
  I2CSensorRead(senbrdHandle, COLOR_RAW+ci[i]*8, 2);
  colorRGB[i].r  = rxBuff[0] | rxBuff[1]<<8;
  return colorRGB[i].r;
}

uint16_t PeanutKingSoccerV3::floorColorRead(uint8_t i) {
  const uint8_t ci[4] = {0, 3, 1, 2};
  for (uint8_t i=0; i<6; i++)    rxBuff[i] = 0;
  I2CSensorRead(senbrdHandle, COLOR_RAW+ci[i]*8, 6);
  colorRGB[i].r  = rxBuff[0] | rxBuff[1]<<8;
  colorRGB[i].g  = rxBuff[2] | rxBuff[3]<<8;
  colorRGB[i].b  = rxBuff[4] | rxBuff[5]<<8;
  return colorRGB[i].r + colorRGB[i].g + colorRGB[i].b;
}

uint8_t PeanutKingSoccerV3::colorReadAll(void) {
  const uint8_t ci[4] = {0, 3, 1, 2};
  
  for (uint8_t i=0; i<32; i++)    rxBuff[i] = 0;
  I2CSensorRead(senbrdHandle, COLOR_RAW, 32);
  for (uint8_t i=0; i<4; i++) {
    colorRGB[ci[i]].r  = rxBuff[8*i]   | rxBuff[8*i+1]<<8;
    colorRGB[ci[i]].g  = rxBuff[8*i+2] | rxBuff[8*i+3]<<8;
    colorRGB[ci[i]].b  = rxBuff[8*i+4] | rxBuff[8*i+5]<<8;
  }
    // groundColor[i] = rxBuff[24+i];

  // for (uint8_t i=0; i<16; i++)    rxBuff[i] = 0;
  // I2CSensorRead(senbrdHandle, COLOR_HSL, 16);
  // for (uint8_t i=0; i<4; i++) {
  //   colorHSL[ci[i]].h  = rxBuff[4*i] | rxBuff[4*i+1]<<8;
  //   colorHSL[ci[i]].s  = rxBuff[4*i+2];
  //   colorHSL[ci[i]].l  = rxBuff[4*i+3];
  // }

  // for (uint8_t i=0; i<16; i++)    rxBuff[i] = 0;
  // I2CSensorRead(senbrdHandle, COLOR_HSV, 16);
  // for (uint8_t i=0; i<4; i++) {
  //   colorHSV[ci[i]].h  = rxBuff[4*i] | rxBuff[4*i+1]<<8;
  //   colorHSV[ci[i]].s  = rxBuff[4*i+2];
  //   colorHSV[ci[i]].v  = rxBuff[4*i+3];
  // }
  // for (uint8_t i=0; i<28; i++)    rxBuff[i] = 0;
  // I2CSensorRead(senbrdHandle, COLOR_RAW, 28);
  // for (uint8_t i=0; i<4; i++) {
  //   colorRGB[i].r  = rxBuff[6*i]   | rxBuff[6*i+1]<<8;
  //   colorRGB[i].g  = rxBuff[6*i+2] | rxBuff[6*i+3]<<8;
  //   colorRGB[i].b  = rxBuff[6*i+4] | rxBuff[6*i+5]<<8;
  //   groundColor[i] = rxBuff[24+i];
  // }
}

uint16_t PeanutKingSoccerV3::whiteLineCal(uint8_t pin_no, uint16_t calVal) {
  whiteLineThreshold[pin_no] = calVal;
  floorColorRead(pin_no);
  return (colorRGB[pin_no].r + colorRGB[pin_no].g + colorRGB[pin_no].b);
}

bool PeanutKingSoccerV3::whiteLineCheck(uint8_t i) {
  floorColorRead(i);
  isWhite[i] = ((colorRGB[i].r + colorRGB[i].g + colorRGB[i].b) > whiteLineThreshold[i]);
  return isWhite[i];
}

void PeanutKingSoccerV3::actLED(bool state) {
  digitalWrite(actledPin, state);
}

/* =============================================================================
 *                                  Motors
 * ============================================================================= */
// simple motor turn, [mi] cannot add, one by one 
void PeanutKingSoccerV3::motorSet(uint8_t mi, int16_t speed) {
  targetSpeed[mi] = speed;
  //digitalWrite(dirPin[mi], motorBrakeEnabled ?  HIGH : LOW);

  if ( targetSpeed[mi]>0 ) {
    if      ( targetSpeed[mi]<  9 ) targetSpeed[mi] = 0;
    else if ( targetSpeed[mi]< 20 ) targetSpeed[mi] = 20;
    else if ( targetSpeed[mi]>255 ) targetSpeed[mi] = 255;
  }
  else if ( targetSpeed[mi]<0 ) {
    if      ( targetSpeed[mi]> -9 ) targetSpeed[mi] = 0;
    else if ( targetSpeed[mi]>-20 ) targetSpeed[mi] = -20;
    else if ( targetSpeed[mi]<-256) targetSpeed[mi] = -256;
  }
}

void PeanutKingSoccerV3::motorUpdate(uint8_t mi) {
  if ( !motorEnabled ) {
    digitalWrite(in1Pin[mi], HIGH);
    digitalWrite(in2Pin[mi], HIGH);
    targetSpeed[mi] = 0;
    currentSpeed[mi] = 0;
  } 
  else if (targetSpeed[mi] == 0) {
    digitalWrite(in1Pin[mi], HIGH);
    digitalWrite(in2Pin[mi], HIGH);
    currentSpeed[mi] = 0;
  }
  else {
    if      (targetSpeed[mi] > currentSpeed[mi]) currentSpeed[mi] += 1;
    else if (targetSpeed[mi] < currentSpeed[mi]) currentSpeed[mi] -= 1;
    if (currentSpeed[mi] > 0) {
      digitalWrite(in1Pin[mi], LOW);
      analogWrite(in2Pin[mi], currentSpeed[mi] );
    }
    else {
      // digitalWrite(in1Pin[mi], HIGH);
      // analogWrite(in2Pin[mi], 255 + currentSpeed[mi] );
      analogWrite(in1Pin[mi], -currentSpeed[mi] );
      digitalWrite(in2Pin[mi], LOW);
    }
  }
}

void PeanutKingSoccerV3::motorControl(float mAngle, float mSpeed, float rotate) {
  int16_t mc[4];

  mc[0] = -mSpeed*sin( (mAngle+45.0)*pi/180.0 );
  mc[1] = -mSpeed*cos( (mAngle+45.0)*pi/180.0 );
  mc[2] = -mc[0];
  mc[3] = -mc[1];

  for(int8_t i=3; i>=0; i--) {
    motorSet(i, mc[i] - rotate);
  }
}

void PeanutKingSoccerV3::move(int16_t speed_X, int16_t speed_Y) {
  double mAngle = atan((double)speed_Y/(double)speed_X) * pi;
  if ( speed_X<0 ) mAngle += 180;
  if ( mAngle<0 )  mAngle += 360;
  
  uint16_t mSpeed = sqrt( speed_X*speed_X + speed_Y*speed_Y );
  
  moveSmart(mAngle, mSpeed);
}

// motor move + compass as reference
void PeanutKingSoccerV3::moveSmart(uint16_t angular_direction, int16_t speed, int16_t angle, uint8_t precision) {
  int16_t c = compassRead() - angle;
  int16_t rotation = c < 180 ? -c : 360 - c;
  
  //speed - 50
  //rotation = abs(speed) < 120 ? rotation : rotation * 1.5;
  rotation = rotation * (precision+3)/12;
  // if ( speed==0 && abs(rotation)>10 ) rotation = rotation < 35 ? 35 : rotation;
  if ( speed==0 && abs(rotation)<12 ) rotation = 0;
  motorControl(angular_direction, speed, rotation);
}

void PeanutKingSoccerV3::motorStop(void) {
  for(uint8_t i=0; i<4; i++) {
    motorSet(i, 0);
  }
}




/* =============================================================================
 *                              Advance Control
 * ============================================================================= */

// motor test ------------------------------------------------------
uint8_t PeanutKingSoccerV3::motorTest (void) {
  static uint32_t motorTimer = 0;
  static uint8_t i = 0;
  uint32_t timeNow = millis();
  
  if ( timeNow - motorTimer > 1000) {
    motorTimer = timeNow;
    for (uint8_t j=0; j<4; j++) {
      if ( i<4 )
        motorSet( j, i==j ? 100 : 0 );
      else
        motorSet( j, (i-4)==j ? -100 : 0 );
    }
    i++;
    if ( i==9 )
      i=0;
  }
}


//                                  strategy
// =================================================================================
void PeanutKingSoccerV3::Chase(int& direct, int& speed, int& rotation) {
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
    
    /*
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
    */
}

void PeanutKingSoccerV3::Back(int& direct, int& speed, int& rotation) {
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



// uint16_t PeanutKingSoccerV3::sort(uint16_t a[], uint8_t size) {
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

// hsv_t PeanutKingSoccerV3::rgb2hsv(rgb_t in) {
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


// // Bluetooth ------------------------------------------------------
// void PeanutKingSoccerV3::bluetoothSend(char string[]) {
// // send char
//   Serial1.write(string, sizeof(string));
// }

// void PeanutKingSoccerV3::bluetoothReceive(void) {
// // send char
//   btRxBuffer[0] = Serial1.read();
// }


// old function
void PeanutKingSoccerV3::enableScanning(bool enable, uint16_t sensorType, bool enableLED) {
  // autoScanEnabled = enable;
  // autoScanSensors = sensorType;
  // ledEnabled = enableLED;
}

bool PeanutKingSoccerV3::buttTrigRead(uint8_t pin) {
  bool b = !digitalRead(buttonPin[pin]);

}

void PeanutKingSoccerV3::buttons(void) {
  static uint32_t holdTimer[3] = {0};
  uint32_t currentTime = millis();

  for (uint8_t i=0; i<3; i++) {
    bool b = !digitalRead(buttonPin[i]);
    if ( b ) {                  // Pressed
      switch(button[i]) {
        case NONE:
          button[i] = TAP;
          holdTimer[i] = currentTime;
        break;
        case TAP:
          if ( currentTime - holdTimer[i] > TAP_DURATION ) {
            button[i] = PRESS;
          }
        break;
        case TAP2:
          if ( currentTime - holdTimer[i] > HOLD_DURATION ) {
            button[i] = HOLD2;
          }
        break;
        case TAP3:
          if ( currentTime - holdTimer[i] > HOLD_DURATION )
            button[i] = RELEASE;
        break;
        case PRESS:
          if ( currentTime - holdTimer[i] > HOLD_DURATION ) {
            button[i] = HOLD;
          }
        break;
        case TAP1_W:
          holdTimer[i] = currentTime;
          button[i] = TAP2;
        break;
        case TAP2_W:
          holdTimer[i] = currentTime;
          button[i] = TAP3;
        break;
        case TAP3_W:
          if ( currentTime - holdTimer[i] > HOLD_DURATION )
            button[i] = RELEASE;
        break;
        case RELEASE:
        case RELEASE_S:
        case RELEASE_L:
          button[i] = TAP;
        break;
        case HOLD:
        break;
        default:
        break;
      }
    }
    else {                                                // Release
      switch(button[i]) {
        case TAP:
          button[i] = TAP1_W;
          holdTimer[i] = currentTime;
        break;
        case TAP2:
          button[i] = TAP2_W;
          holdTimer[i] = currentTime;
        break;
        case TAP3:
          button[i] = RELEASE;
          holdTimer[i] = currentTime;
        break;
        case PRESS:
          button[i] = RELEASE_S;
        break;
        case TAP1_W:
          if ( currentTime - holdTimer[i] > WAIT_DURATION )
            button[i] = RELEASE_S;
        break;
        case HOLD:
          button[i] = RELEASE_L;
        break;
        case TAP2_W:
          if ( currentTime - holdTimer[i] > WAIT_DURATION )
            button[i] = TAP2_R;
        break;
        case TAP3_W:
          if ( currentTime - holdTimer[i] > WAIT_DURATION )
            button[i] = TAP3_R;
        break;
        case RELEASE:
        case RELEASE_S:
        case RELEASE_L:
        case TAP2_R:
        case TAP3_R:
          button[i] = NONE;
        break;
        default:
        break;
      }
    }
  }
  // return button[i];
}


void PeanutKingSoccerV3::lcdMenu(void) {

}

void PeanutKingSoccerV3::bluetoothAttributes() {

}



void PeanutKingSoccerV3::bluetoothRemote(void) {
  static btDataType btDataHeader = Idle;
  static uint32_t btSendTimer = 0;
  static uint8_t btState = 0, len = 0;
  static int btAngle = 0;
  static String deg = "", dis = "", buttonVal = "";
  static float speed = 1.0;
  
  if (Serial1.available()) {
    char v = Serial1.read();
    // Serial.print(v);
    
    switch (btDataHeader) {
      case Idle:
      case EndOfData:
      case DemoMode:
        switch (v) {
          case 'A':
            btDataHeader = Joystick;
            break;
          case 'B':
            btDataHeader = PadButton;
            break;
          case 'C':
            btDataHeader = ButtonDef;
            break;
          case 'D':
            //btDataHeader = Attributes;
            break;
          case 'Z':
            btDataHeader = EndOfData;
            break;
          case 'Y':
            btDataHeader = DemoMode;
            break;
        }
        btState = 1;
        break;
      case Joystick:
        switch(btState) {
          case 1:
            if (v != 'D')
              deg += v;
            else {
              int temp = deg.toInt();
              if (temp<360)
                btDegree = temp;
                
              deg = "";
              btState++;
              // Serial.print(btDegree); Serial.print(' ');
            }
          break;
          case 2:
            if (v != '.')
              dis += v;
            else {
              int temp = dis.toInt();
              if (temp<=100)
                btDistance = temp;
                
              //Serial.print(btDistance); Serial.println(' ');
              dis = "";
              btState=0;
              btDataHeader = Idle;
            }
          break;
        }
        break;
      case PadButton:
        if (len==0) {
          btButtonIndex = v-'0';
          len ++;
        }
        else if (len==1) {
          btGestureCode = v-'0';
          len = 0;
          btDataHeader = Idle;
          //Serial.print("buttun pressed ");
          //Serial.print(btButtonIndex); Serial.print(btGestureCode); Serial.println(' ');
          //List<String> functionList = ['Accel', 'Back', 'Chase', 'Auto', 'L-Trun', 'R-Trun', 'Front', 'Left', 'Right', 'Back'];
        }
        break;
      case ButtonDef:
        if (len<4) {
          //Serial.print("buttun code ");
          //Serial.print(v);
          //Serial.println(' ');
          //String code = v;
          btButtonFunction[len] = v-'0';//code.toInt();
          len ++;
        }
        else {
          len = 0;
          //buttonVal = "";
          //btState++;
          //btState = 0;
          btDataHeader = Idle;
        }
        break;
      case Attributes:
        if (len<5) {
          btAttributes[len] = v-'0';//code.toInt();
          len ++;
        }
        else {
          EYEBOUNDARY = 10 + (10-btAttributes[0]) * 20;
          len = 0;
          btDataHeader = Idle;
        }
        break;
    }
  }
  if (btDataHeader == EndOfData) {
    btDataHeader = Idle;
    //Serial.print("buttun pressed ");
    //Serial.print(btButtonIndex); Serial.print(btGestureCode); Serial.println(' ');

    // setScreen(0, 0, "Deg ");
    // setScreen(4, 0, btDegree);
    // setScreen(0, 1, "Dist");
    // setScreen(4, 1, btDistance);
    
    // setScreen(12, 0, maxEye);

    // setScreen(12, 1, eye[maxEye] );
    if ( btGestureCode==0 || btGestureCode==4 ) {
      switch (btButtonFunction[btButtonIndex]) {
        case 0:   // Accel
          speed = 2.0;
          break;
        case 1:   // Back
          Back(btDegree, btDistance, btRotate);
          break;
        case 2:   // Chase
          Chase(btDegree, btDistance, btRotate);
          break;
        case 3:   // Auto
          if ( eye[maxEye] > 30 ) {
            Chase(btDegree, btDistance, btRotate);
          }
          else
          {
            btDistance = 0;
            btDegree = 0;
          }
          
          //Back(btDegree, btDistance, btRotate);
          break;
        case 4:
          btAngle = compass;
          btRotate = -40;
          break;
        case 5:
          btAngle = compass;
          btRotate = 40;
          break;
        case 6:   // Front
          break;
      }
    }
    else { // if ( btGestureCode==3 || btGestureCode==6 )
      if (btRotate ==-40 || btRotate ==40) {
          btAngle = compass;
      }
      speed = 1.0;
      btRotate = 0;
    }
    // Execute
    
      Serial.print(btDegree); Serial.print(' ');
      Serial.print(btDistance); Serial.print(' ');
      Serial.print(btRotate); Serial.println(' ');
    
   
    // if ( btRotate==0 ) {
    //   moveSmart(btDegree, btDistance*speed*0.7, btAngle);
    // }
    // else
    //   motorControl(0, 0, btRotate);
  }
  motorControl(btDegree, btDistance, btRotate);

  /*
  else if (btDataHeader == DemoMode) {
    if ( eye[maxEye] > EYEBOUNDARY ) {
      Chase(btDegree, btDistance, btRotate);
    }
    //Back(btDegree, btDistance, btRotate);
    moveSmart(btDegree, btDistance*speed, btAngle);
  }*/

  // Send Data
  if (millis() - btSendTimer > 100) {
    if (Serial1.availableForWrite() > 50) {
      btTxBuffer[0] = 'C';
      btTxBuffer[1] = compass & 0xff;
      btTxBuffer[2] = compass >> 8;
      btTxBuffer[3] = 'U';
      btTxBuffer[4] = ultrasonic[0] > 255 ? 255 : ultrasonic[0];
      btTxBuffer[8] = 'E';
      btTxBuffer[9] = maxEye;
      btTxBuffer[11] = eye[maxEye] & 0xff;
      btTxBuffer[12] = eye[maxEye] >> 8;
      btTxBuffer[13] = 'Z';
      Serial1.write(btTxBuffer, 14);
    }
    btSendTimer = millis();
  }
}


























// col(0-15), row(0-1) --------------------------------------------
void PeanutKingSoccerV3::setScreen(uint8_t col, uint8_t row, char string[]) {
  // if ((millis() - screenTicks) > 40) {
    screenTicks = millis();
    setCursor(col, row);
    print(string);
  // }
}

void PeanutKingSoccerV3::setScreen(uint8_t col, uint8_t row, int16_t numbers, uint8_t digits) {
  // volatile bool temp = autoScanEnabled;
  // autoScanEnabled = false;
  // autoScanEnabled = temp;

  // if ((millis() - screenTicks) > 40) {
    screenTicks = millis();
    setCursor(col, row);
    if (numbers>=0) {
      for (uint8_t i=1; i<digits; i++ ) {
        if ( numbers < pow(10, i) )   print(" ");
      }
    }
    print(numbers);
  // }
}

void PeanutKingSoccerV3::printSpace(uint32_t data, uint8_t digit) {
  for ( int i=1; i<digit; i++ ) {
    if ( data < pow(10, i) )   Serial.print(" ");
  }
  Serial.print(data);
}







//                                  LCD LIBRARY
// ================================================================

void PeanutKingSoccerV3::lcdSetup (void) {
  // According to datasheet PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // we need at least 40ms after power rises above 2.7V before sending commands.
  // Arduino can turn on way befer 4.5V so we'll wait 50
  delay(5);
  
  // Now we pull both RS and R/W low to begin commands
  uint8_t msg[1] = {LCD_backlightval};// reset expander and turn backlight off (Bit 8 =1)
  uint8_t _status = gIIC->Write(lcdScrHandle, msg, 1);
  delay(1000);

  // put the LCD into 4 bit mode, according to the hitachi HD44780 datasheet figure 24, pg 46
  write4bits(0x03 << 4);  delayMicroseconds(4200);  // Start in 8bit mode, try 4 bit mode, wait > 4.1ms
  write4bits(0x03 << 4);  delayMicroseconds(150);   // second try, wait > 4.1ms
  write4bits(0x03 << 4);  delayMicroseconds(37);    // third go!
  write4bits(0x02 << 4);  delayMicroseconds(37);

  send(0b00101000, 0);    delayMicroseconds(37);    // Function Set - 4 bits(Still), 2 lines, 5x8 font
  send(0x08 | 0x04, 0);   delayMicroseconds(37);    // Initialize to default text direction

  lcdClear();               // clear it off

  send(0x04 | 0x02, 0);     // set the entry mode, left to right

  //send(LCD_RETURNHOME, 0);    delay(2);             // set cursor position to zero, takes a long time!
  LCD_backlightval=LCD_BACKLIGHT;
}


/********** high level commands, for the user! */
void PeanutKingSoccerV3::lcdClear(void) {
  send(LCD_CLEARDISPLAY, 0);// Clear display, set cursor position to zero
  delayMicroseconds(1600);  // Takes a long time!
}

void PeanutKingSoccerV3::setCursor(uint8_t col, uint8_t row) {
  static const uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if ( row > 2 )  row = 1;
  send(LCD_SETDDRAMADDR | (col + row_offsets[row]), 0);   // **********send**********
}

size_t PeanutKingSoccerV3::printNumber(unsigned long n, uint8_t base) {
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];

  *str = '\0';

  // prevent crash if called with base == 1
  if (base < 2) base = 10;
  
  do {
    char c = n % base;
    n /= base;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);
  return print(str);
}

size_t PeanutKingSoccerV3::print(long n, int base) {
  if (base == 0) {
    return write(n);
  } else
  if (base == 10) {
    if (n < 0) {
      char v[1] = "-";
      
      int t = print(v);
      n = -n;
      return printNumber(n, 10) + t;
    }
    return printNumber(n, 10);
  }
  else {
    return printNumber(n, base);
  }
}

size_t PeanutKingSoccerV3::print(const char str[]) {
  size_t n = 0;
  size_t l = strlen(str);
  while (1) {
    if (n==l)           break;
    if (write(*str++))  n++;
    else                break;
  }
  return n;
}
//send(c, Rs)

inline size_t PeanutKingSoccerV3::write(uint8_t value) {
  send(value, Rs);
  return 1;
}

//command(uint8_t value)   send(value, 0);
/************ low level data pushing commands **********/

// write either command or data
void PeanutKingSoccerV3::send(uint8_t value, uint8_t mode) {
  uint8_t highnib =  value    & 0xf0;     // HHHH0000
  uint8_t lownib  = (value<<4)& 0xf0;     // LLLL0000
  
  write4bits((highnib)|mode);
  write4bits((lownib)|mode);
}

void PeanutKingSoccerV3::write4bits(uint8_t value) {
  uint8_t _status;
  uint8_t msg[2] = {
    // ((int)(value) | LCD_backlightval),
    ((int)(value | En) | LCD_backlightval),    //pulseEnable
    ((int)(value & ~En) | LCD_backlightval)
  };
  _status = gIIC->Write(lcdScrHandle, msg, 1);
  delayMicroseconds(1);        // enable pulse must be >450ns
  _status = gIIC->Write(lcdScrHandle, msg+1, 1);
  delayMicroseconds(40);       // commands need > 37us to settle
}





void btSetupnTest(){
  char setBaud[] = "AT+BAUD8";
  
  Serial1.begin(9600);

  Serial1.write(setBaud, sizeof(setBaud));

  Serial1.end();
  Serial1.begin(115200);
}

