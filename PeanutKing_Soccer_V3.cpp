/*
 * Copyright (c) 2020 PeanutKing Solution
 *
 * @file        PeanutKing_Soccer_V3.cpp
 * @summary     Soccer Robot Library
 * @version     1.0
 * @author      Jack Kwok
 * @data        1 August 2020
 */

#include "PeanutKing_Soccer_V3.h"

static PeanutKing_Soccer_V3* V3bot = NULL;

PeanutKing_Soccer_V3::PeanutKing_Soccer_V3(void) :
  PeanutKing_Soccer(),
  actledPin(30),
  topBoardAddr(10),
  sensorBoardAddr(12),
  buttonPin{42, 47, 48},

  pwmPin  { 4,  7, 10, 13},
  dirPin  { 2,  5,  8, 11},
  dir2Pin { 3,  6,  9, 12},
  diagPin {50, 51, 53, 53}
  {
  if (V3bot == NULL)  {
    V3bot = this;
  }
}

/*
ISR (TIMER1_COMPA_vect) {
  if (V3bot != NULL ) {
    V3bot->autoScanning();
  }
}
*/

// initialize all IOs, Serial.begin, I2C, timer interrupt, 
// External interrupt different settings depends on version number 
void PeanutKing_Soccer_V3::init(uint8_t mode) {
  Wire.begin();
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
  
  lcdSetup();
  
  delay(10);
  
  cli();    //disable interrupts
  // Timer 1
  TCCR1A = 0x00;            // Normal mode, just as a Timer
  TCCR1B = 0;               // same for TCCR0B
  TCNT1 = 0;
  
  OCR1A = 624;       // =(16*10^6) / (125*256) -1 (must be <65536)
  
  TCCR1B |= (1 << WGM12);   // CTC mode; Clear Timer on Compare
  TCCR1B |= (1 << CS12);    // prescaler = 256
  
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  sei();    //allow interrupts
  
  //while ( compassRead() == 400 );
  delay(10);
}


/* =============================================================================
 *                                  Data fetch
 * ============================================================================= */
void PeanutKing_Soccer_V3::dataFetch(void) {
  int16_t temp;

  // Serial.print("A: ");
  // for (uint8_t i=0; i<6; i++)     rxBuff[i] = 0;
  // I2CSensorRead(8, ACC_RAW, 6);
  // for (uint8_t i=0; i<3; i++) {
  //   temp  = rxBuff[2*i] & 0xff;
  //   temp |= rxBuff[2*i+1] << 8;
  //   Serial.print(temp);
  //   Serial.print(' ');
  // }
  // Serial.print(" G: ");
  // for (uint8_t i=0; i<6; i++)     rxBuff[i] = 0;
  // I2CSensorRead(8, GYR_RAW, 6);
  // for (uint8_t i=0; i<3; i++) {
  //   temp  = rxBuff[2*i] & 0xff;
  //   temp |= rxBuff[2*i+1] << 8;
  //   Serial.print(temp);
  //   Serial.print(' ');
  // }
  // Serial.print(" M: ");
  // for (uint8_t i=0; i<6; i++)     rxBuff[i] = 0;
  // I2CSensorRead(8, MAG_BUF, 6);
  // for (uint8_t i=0; i<3; i++) {
  //   temp  = rxBuff[2*i] & 0xff;
  //   temp |= rxBuff[2*i+1] << 8;
  //   Serial.print(temp);
  //   Serial.print(' ');
  // }
  // Serial.println(' ');

  for (uint8_t i=0; i<2; i++)     rxBuff[i] = 0;
  I2CSensorRead(8, GET_YAW, 2);
  compass  = rxBuff[0] & 0xff;
  compass |= rxBuff[1] << 8;
  compass = compass/100;

  for (uint8_t i=0; i<8; i++)     rxBuff[i] = 0;
  I2CSensorRead(topBoardAddr, ULT_DATA, 8);
  for (uint8_t i=0; i<4; i++) {
    ultrasonic[i]  = rxBuff[2*i] & 0xff;
    ultrasonic[i] |= rxBuff[2*i+1] << 8;
  }

  //sensorBoardAddr
  for (uint8_t i=0; i<27; i++)    rxBuff[i] = 0;
  I2CSensorRead(sensorBoardAddr, IR_RAW, 27);
  for (uint8_t i=0; i<12; i++) {
    eye[i]  = rxBuff[2*i] & 0xff;
    eye[i] |= rxBuff[2*i+1] << 8;
  }
  maxEye = rxBuff[24];

  for (uint8_t i=0; i<28; i++)    rxBuff[i] = 0;
  I2CSensorRead(sensorBoardAddr, COLOR_RAW, 28);
  for (uint8_t i=0; i<4; i++) {
    colorRGB[i].r  = rxBuff[6*i]   | rxBuff[6*i+1]<<8;
    colorRGB[i].g  = rxBuff[6*i+2] | rxBuff[6*i+3]<<8;
    colorRGB[i].b  = rxBuff[6*i+4] | rxBuff[6*i+5]<<8;
    groundColor[i] = rxBuff[24+i];
  }

  for (uint8_t i=0; i<16; i++)    rxBuff[i] = 0;
  I2CSensorRead(sensorBoardAddr, COLOR_HSL, 16);
  for (uint8_t i=0; i<4; i++) {
    colorHSL[i].h  = rxBuff[4*i] | rxBuff[4*i+1]<<8;
    colorHSL[i].s  = rxBuff[4*i+2];
    colorHSL[i].l  = rxBuff[4*i+3];
  }

  for (uint8_t i=0; i<16; i++)    rxBuff[i] = 0;
  I2CSensorRead(sensorBoardAddr, COLOR_HSV, 16);
  for (uint8_t i=0; i<4; i++) {
    colorHSV[i].h  = rxBuff[4*i] | rxBuff[4*i+1]<<8;
    colorHSV[i].s  = rxBuff[4*i+2];
    colorHSV[i].v  = rxBuff[4*i+3];
  }
}

void PeanutKing_Soccer_V3::I2CSensorRead(int8_t addr, soccerRegAddr sensor, uint8_t length) {
  txBuff[0] = sensor;
  I2CSend(addr, txBuff, 1);
  I2CRead(addr, rxBuff, length);
}

void PeanutKing_Soccer_V3::I2CSensorSend(int8_t addr, soccerRegAddr sensor, uint8_t *data, uint8_t length) {
  txBuff[0] = sensor;
  //I2CSend(addr, txBuff, 1);
  //I2CRead(addr, rxBuff, 1);
//  if (rxBuff[0]!=2) return;

  for (uint8_t i=0;i<length;i++) {
    txBuff[i+1] = data[i];
  }
  I2CSend(addr, txBuff, length+1);
}


void PeanutKing_Soccer_V3::I2CSend(int8_t addr, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(addr);
  Wire.write(data, length);
  Wire.endTransmission();
}

void PeanutKing_Soccer_V3::I2CRead(int8_t addr, uint8_t *data, uint8_t length) {
  uint8_t i=0;
  Wire.requestFrom((int)addr, (int)length);
  while (Wire.available()) {
    data[i++] = Wire.read();
  }
}


/* =============================================================================
 *                                  Sensor Read
 * ============================================================================= */
bool PeanutKing_Soccer_V3::buttonRead(uint8_t button_no) {
  if ( button_no == 1 || button_no == 2 ) 
    return rawButton(buttonPin[button_no]);
  else
    return 0;
}

uint16_t PeanutKing_Soccer_V3::compassRead(void) {
  return compass;
}

uint16_t PeanutKing_Soccer_V3::compoundEyeRead (uint8_t eye_no) {
  if ( eye_no >0 && eye_no <= 12 )
    return eye[eye_no];
  else if ( eye_no == 13 ) 
    return maxEye;
  else if ( eye_no == 14 ) 
    return eye[maxEye];
  else 
    return 0;
}

uint16_t PeanutKing_Soccer_V3::ultrasonicRead(uint8_t ultrasonic_no) {
  if ( ultrasonic_no<0 || ultrasonic_no>=4 ) return 999;
  return ultrasonic[ultrasonic_no];
}

void PeanutKing_Soccer_V3::setLED(uint8_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  for (uint8_t i=0; i<8; i++) {
    if ( n & (1<<i) ) {
      uint8_t *p = &led[i * 4];  // 4 bytes per pixel
      p[0] = r;                   // R
      p[1] = g;                   // G
      p[2] = b;                   // B
      p[3] = w;                   // W
    }
  }
  I2CSensorSend(topBoardAddr,LED_RGB,led,32);
}

// void PeanutKing_Soccer_V3::sendLED(void) {
// }

uint8_t PeanutKing_Soccer_V3::floorColorRead(uint8_t pin_no) {
  for (uint8_t i=0; i<28; i++)    rxBuff[i] = 0;
  I2CSensorRead(sensorBoardAddr, COLOR_RAW, 28);
  for (uint8_t i=0; i<4; i++) {
    colorRGB[i].r  = rxBuff[6*i]   | rxBuff[6*i+1]<<8;
    colorRGB[i].g  = rxBuff[6*i+2] | rxBuff[6*i+3]<<8;
    colorRGB[i].b  = rxBuff[6*i+4] | rxBuff[6*i+5]<<8;
    groundColor[i] = rxBuff[24+i];
  }
}

uint16_t PeanutKing_Soccer_V3::whiteLineCal(uint16_t calVal, uint8_t pin_no) {
  whiteLineThreshold = calVal;
  floorColorRead(pin_no);
  return (colorRGB[pin_no].r + colorRGB[pin_no].g + colorRGB[pin_no].b);
}

bool PeanutKing_Soccer_V3::whiteLineCheck(uint8_t pin_no) {
  floorColorRead(pin_no);
  return ((colorRGB[pin_no].r + colorRGB[pin_no].g + colorRGB[pin_no].b) < whiteLineThreshold);
}

/* =============================================================================
 *                                  Motors
 * ============================================================================= */
// simple motor turn, motor_no cannot add, one by one 
void PeanutKing_Soccer_V3::motorSet(uint8_t motor_no, int16_t speed) {
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

void PeanutKing_Soccer_V3::motorControl(float mAngle, float mSpeed, float rotate) {
  int16_t mc[4];

  mc[0] = mSpeed*sin( (mAngle+45.0)*pi/180.0 );
  mc[1] = mSpeed*cos( (mAngle+45.0)*pi/180.0 );
  mc[2] = -mc[0];
  mc[3] = -mc[1];

  for(int8_t i=3; i>=0; i--) {
    motorSet(i, mc[i] + rotate);
  }
}

void PeanutKing_Soccer_V3::move(int16_t speed_X, int16_t speed_Y) {
  double mAngle = atan((double)speed_Y/(double)speed_X) * pi;
  if ( speed_X<0 ) mAngle += 180;
  if ( mAngle<0 )  mAngle += 360;
  
  uint16_t mSpeed = sqrt( speed_X*speed_X + speed_Y*speed_Y );
  
  moveSmart(mAngle, mSpeed);
}

// motor move + compass as reference
void PeanutKing_Soccer_V3::moveSmart(uint16_t angular_direction, int16_t speed, int16_t angle, uint8_t precision) {
  int16_t c = compassRead() - angle;
  int16_t rotation = c < 180 ? -c : 360 - c;
  
  //speed - 50
  //rotation = abs(speed) < 120 ? rotation : rotation * 1.5;
  rotation = rotation * (precision+3)/12;
  if ( speed==0 && abs(rotation)>10 ) rotation = rotation < 35 ? 35 : rotation;
  motorControl(angular_direction, speed, rotation);
}

void PeanutKing_Soccer_V3::motorStop(void) {
  for(uint8_t i=0; i<4; i++) {
    motorSet(i, 0);
  }
}





/* =============================================================================
 *                              Advance Control
 * ============================================================================= */

// motor test ------------------------------------------------------
uint8_t PeanutKing_Soccer_V3::motorTest (void) {
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
void PeanutKing_Soccer_V3::Chase(int& direct, int& speed, int& rotation) {
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

void PeanutKing_Soccer_V3::Back(int& direct, int& speed, int& rotation) {
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












