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
  //actLEC(37)
  pwmPin  { 4,  7, 10, 13},
  dirPin  { 2,  5,  8, 11},
  dir2Pin { 3,  6,  9, 12},
  diagPin {50, 51, 53, 53} {
  if (V3bot == NULL)  {
    V3bot = this;
  }
}

// ISR (TIMER1_COMPA_vect) {
//   if (V3bot != NULL ) {
//     V3bot->dataFetch();
//   }
// }

// initialize all IOs, Serial.begin, I2C, timer interrupt, 
// External interrupt different settings depends on version number 
void PeanutKingSoccerV3::init(uint8_t mode) {
  Serial.begin(115200);
  Serial1.begin(9600);

  for (uint8_t i=0; i<4; i++) {
    pinMode(pwmPin[i],  OUTPUT);
    pinMode(dirPin[i],  OUTPUT);
    pinMode(dir2Pin[i], OUTPUT);
    pinMode(diagPin[i], OUTPUT);
    digitalWrite(diagPin[i], HIGH);
  }
  for (uint8_t i=0; i<3; i++)
    pinMode(buttonPin[i], INPUT);
  
  pinMode(actledPin, INPUT);
  digitalWrite(actledPin, HIGH);
  
  delay(10);

  compssHandle = gIIC->RegisterDevice(compass_address, 1, IICIT::Speed::SLOW);
  senbrdHandle = gIIC->RegisterDevice(sensorBoardAddr, 1, IICIT::Speed::SLOW);
  lcdScrHandle = gIIC->RegisterDevice(LCD_Addr, 1, IICIT::Speed::SLOW);

  // uint8_t msg[1] = {LCD_backlightval};// reset expander and turn backlight off (Bit 8 =1)
  // uint8_t _status = gIIC->Write(lcdScrHandle, msg, 1);
  // delay(10);
  // if ( gIIC->getStatus() != 0 ) {
  //   lcdScrHandle = gIIC->RegisterDevice(0x38, 1, IICIT::Speed::SLOW);
  // }
  delay(2500);

  lcdSetup();
  
  // cli();    //disable interrupts
  // // Timer 1
  // TCCR1A = 0x00;            // Normal mode, just as a Timer
  // TCCR1B = 0;               // same for TCCR0B
  // TCNT1 = 0;
  
  // TCCR1B |= (1 << WGM12);   // CTC mode; Clear Timer on Compare
  // TCCR1B |= (1 << CS12);    // prescaler = 256
  // // TCCR1B |= (1 << CS10) | (1 << CS12);    // prescaler = 1024

  // // 50Hz
  // OCR1A = 1250;       // =(16*10^6) / (125*256) -1 (must be <65536)
  
  // TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  // sei();    //allow interrupts
  
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

  delay(100);
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

void PeanutKingSoccerV3::setLED(uint8_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  uint8_t _status;
  led[0] = LED_RGB;
  for (uint8_t i=0; i<8; i++) {
    if ( n & (1<<i) ) {
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
uint8_t PeanutKingSoccerV3::floorColorRead(uint8_t i) {
  const uint8_t ci[4] = {0, 3, 1, 2};
  for (uint8_t i=0; i<6; i++)    rxBuff[i] = 0;
  I2CSensorRead(senbrdHandle, COLOR_RAW+ci[i]*8, 6);
  colorRGB[i].r  = rxBuff[0] | rxBuff[1]<<8;
  colorRGB[i].g  = rxBuff[2] | rxBuff[3]<<8;
  colorRGB[i].b  = rxBuff[4] | rxBuff[5]<<8;
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

uint16_t PeanutKingSoccerV3::whiteLineCal(uint16_t calVal, uint8_t pin_no) {
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
// simple motor turn, motor_no cannot add, one by one 
void PeanutKingSoccerV3::motorSet(uint8_t motor_no, int16_t speed) {
  //static int16_t previousSpeed[4] = {0,0,0,0};
  if ( !motorEnabled ) speed = 0;
  if      ( speed>0 && speed<256 ) {
    digitalWrite(dirPin[motor_no], LOW);
    digitalWrite(dir2Pin[motor_no], HIGH);
    analogWrite(pwmPin[motor_no], speed);
    digitalWrite(diagPin[motor_no], HIGH);
  }
  else if ( speed<0 && speed>-256 ) {
    digitalWrite(dirPin[motor_no], HIGH);
    digitalWrite(dir2Pin[motor_no], LOW);
    analogWrite(pwmPin[motor_no], -speed);
    digitalWrite(diagPin[motor_no], HIGH);
  }
  else{
    //digitalWrite(dirPin[motor_no], motorBrakeEnabled ?  HIGH : LOW);
    digitalWrite(dirPin[motor_no], HIGH);
    digitalWrite(dir2Pin[motor_no], HIGH);
    digitalWrite(pwmPin[motor_no], HIGH);
    //digitalWrite(diagPin[motor_no], LOW);
  }
  //previousSpeed[motor_no] = speed;
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


void lcdMenu(void) {

}

void bluetoothAttributes() {

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
  // if ((millis() - screenTicks) > 40) {
    screenTicks = millis();
    setCursor(col, row);
    if (numbers>=0) {
      for (uint8_t i=1; i<digits; i++ ) {
        if ( numbers < pow(10, i) )   print(" ");
      }
    }
    print(numbers);
    // print(" ");
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
  // volatile bool temp = autoScanEnabled;
  // autoScanEnabled = false;

  uint8_t msg[2] = {
    // ((int)(value) | LCD_backlightval),
    ((int)(value | En) | LCD_backlightval),    //pulseEnable
    ((int)(value & ~En) | LCD_backlightval)
  };
  uint8_t _status = gIIC->Write(lcdScrHandle, msg, 1);
  delayMicroseconds(1);        // enable pulse must be >450ns
  _status = gIIC->Write(lcdScrHandle, msg+1, 1);

  // autoScanEnabled = temp;
  delayMicroseconds(40);       // commands need > 37us to settle
}



