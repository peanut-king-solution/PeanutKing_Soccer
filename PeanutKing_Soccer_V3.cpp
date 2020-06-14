#include "PeanutKing_Soccer_V3.h"

static PeanutKing_Soccer_V3* V3bot = NULL;

PeanutKing_Soccer_V3::PeanutKing_Soccer_V3(void) :
  PeanutKing_Soccer(),
  actledPin(30),
  topBoardAddr(10),
  sensorBoardAddr(12),
  buttonPin{42, 47, 48},

  pwmPin   { 5,  4,  3,  2},
  dirPin   {12, 10,  8,  6},
  dir2Pin  {13, 11,  9,  7},
  diagPin  {50, 51, 52, 53}
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
  Serial.begin(9600);
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
 *                              Advance Control
 * ============================================================================= */






/* =============================================================================
 *                                  Data fetch
 * ============================================================================= */
void PeanutKing_Soccer_V3::dataFetch( ) {
  I2CSensorRead(8,Compass,2);
  compass  = rxBuff[0] & 0xff;
  compass |= rxBuff[1] << 8;

  I2CSensorRead(topBoardAddr,Ultrasonic,8);
  for (uint8_t i=0; i<4; i++) {
    ultrasonic[i]  = rxBuff[2*i] & 0xff;
    ultrasonic[i] |= rxBuff[2*i+1] << 8;
  }
  //sensorBoardAddr
  I2CSensorRead(sensorBoardAddr,IRsensor,25);
  for (uint8_t i=0; i<12; i++) {
    eye[i]  = rxBuff[2*i] & 0xff;
    eye[i] |= rxBuff[2*i+1] << 8;
  }
  maxEye = txBuff[24];

  I2CSensorRead(sensorBoardAddr,ColorSensor,16);
  for (uint8_t i=0; i<4; i++) {
    colorRGB[i].r  = rxBuff[4*i];
    colorRGB[i].g  = rxBuff[4*i+1];
    colorRGB[i].b  = rxBuff[4*i+2];
    groundColor[i] = rxBuff[4*i+3];
  }
}

void PeanutKing_Soccer_V3::I2CSensorRead(int8_t addr, Sensors sensor, uint8_t length) {
  txBuff[0] = sensor;
  txBuff[1] = cmdReceive;
  I2CSend(addr, txBuff, 2);
  I2CRead(addr, rxBuff, length);
}

void PeanutKing_Soccer_V3::I2CSensorSend(int8_t addr, Sensors sensor, uint8_t *data, uint8_t length) {
  txBuff[0] = sensor;
  txBuff[1] = cmdTransmit;
  
  for (uint8_t i=0;i<length;i++) {
    txBuff[i+2] = data[i];
  }
  I2CSend(addr, txBuff, length+2);
}

void PeanutKing_Soccer_V3::I2CSend(int8_t addr, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(addr);
  Wire.write(data, length);
  Wire.endTransmission();
}

void PeanutKing_Soccer_V3::I2CRead(int8_t addr, uint8_t *data, uint8_t length) {
  uint8_t i=0;
  Wire.requestFrom((int)addr, (int)length+1);
  if (Wire.available()) {
    data[0] = Wire.read();
  }
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



