#include "PeanutKing_Soccer_V1.h"

PeanutKing_Soccer_V1* V1bot = NULL;

PeanutKing_Soccer_V1::PeanutKing_Soccer_V1(void) :
  ledPin  (33),

  buttonPin{42, 47, 48},

  pwmPin   { 2,  3,  4},
  dirPin   {22, 23, 24},
  // xsound:     xf, xl. xr. xb
  trigPin  {34, 35, 37, 36},
  echoPin  {38, 39, 41, 40},
  // rgbs:       s0, s1, s2, s3
  tcsSxPin {43, 44, 45, 46},
  tcsRxPin {26, 28, 27},

  irPin    {A0, A1, A2, A6, A3, A4, A5, A7}
  {
  if (V1bot == NULL)  {
    V1bot = this;
  }
}


// initialize all IOs, Serial.begin, I2C, timer interrupt, 
// External interrupt different settings depends on version number 
void PeanutKing_Soccer_V1::init(uint8_t mode) {
  Wire.begin();
  Serial.begin(9600);
  Serial1.begin(9600);
    for (int i=0; i<8; i++) {
    pinMode(irPin[i], INPUT);
  }
  for (int i=0; i<4; i++) {
    pinMode(trigPin[i], OUTPUT);
    pinMode(echoPin[i], INPUT);
    pinMode(tcsSxPin[i], OUTPUT);
  }
  for (int i=0; i<3; i++) {
    pinMode(pwmPin[i], OUTPUT);
    pinMode(dirPin[i], OUTPUT);
    pinMode(tcsRxPin[i], INPUT);
    pinMode(buttonPin[i],INPUT_PULLUP);
  }
  // s0 high, s1 low : freq scaling to 20% 
  digitalWrite(tcsSxPin[0], HIGH);
  digitalWrite(tcsSxPin[1], HIGH);
  
  lcdSetup();
  
  ledSetup(0, ledPin, numLEDs);
  if ( !ledEnabled ) {
    ledShow(255, 0, 0, 0, 0);
    ledUpdate();
  }
  delay(5);
  
  if ( mode == 0 ) 
    ledShow(1, 255, 255, 255, 255, 1);
  else
    ledShow(1, 0, 0, 0, 0, 1);
    
  ledUpdate(1);
  
  while ( compassRead() == 400 );
}

color PeanutKing_Soccer_V1::whiteLine(uint8_t pin) {
  const uint8_t& out = tcsRxPin[pin];
  const uint8_t TCSARRAYCOUNT = 1;
  uint16_t 
    r[TCSARRAYCOUNT],
    g[TCSARRAYCOUNT],
    b[TCSARRAYCOUNT];
    
  rgb_t rgbData;
  
  for (uint8_t i=0; i<TCSARRAYCOUNT; i++) {
    digitalWrite(tcsSxPin[2], LOW);
    digitalWrite(tcsSxPin[3], LOW);
    
    delayMicroseconds(100);
    //count OUT, pRed, RED  
    r[i] = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH, 10000);
    digitalWrite(tcsSxPin[3], HIGH);
    delayMicroseconds(100);
    //count OUT, pBLUE, BLUE  
    b[i] = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH, 10000);
    digitalWrite(tcsSxPin[2], HIGH);
    delayMicroseconds(100);
    //count OUT, pGreen, GREEN  
    g[i] = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH, 10000);
  }
  
  rgbData.r = sort(r, TCSARRAYCOUNT);
  rgbData.g = sort(g, TCSARRAYCOUNT);
  rgbData.b = sort(b, TCSARRAYCOUNT);
  
  rgbData.r *= 1.15;
  /*
  Serial.print("rgb ");
  Serial.print(rgbData.r, 0); 
  Serial.print(", ");
  Serial.print(rgbData.g, 0);
  Serial.print(", ");  
  Serial.print(rgbData.b, 0);
  Serial.print("   ");*/
  rgbData.r = map(rgbData.r, 400, 140, 0, 1000)/1000.0;
  rgbData.g = map(rgbData.g, 400, 140, 0, 1000)/1000.0;
  rgbData.b = map(rgbData.b, 400, 140, 0, 1000)/1000.0;
  rgbData.r = constrain(rgbData.r,  0, 1);
  rgbData.g = constrain(rgbData.g,  0, 1);
  rgbData.b = constrain(rgbData.b,  0, 1);
  
  hsv_t hsvData = rgb2hsv(rgbData);
  /*
  Serial.print("hsv ");
  Serial.print(hsvData.h, 0); 
  Serial.print(", ");
  Serial.print(hsvData.s, 2);
  Serial.print(", ");
  Serial.print(hsvData.v, 2);
  Serial.print("    ");*/
  
  if ( hsvData.s < 0.25 ) {
	if ( hsvData.v > 0.7 )
      return white;
    else
	  return grey;
  }
  else 
    return green;
}

void PeanutKing_Soccer_V1::motorControl(uint8_t dir, uint8_t speed) {
  if (speed == 3) {
    switch(dir) {
      case 0:  motors0( -130,    0,  150 );   break;
      case 1:  motors0( - 85, - 85,  150 );   break;
      case 2:  motors0(    0, -130,  150 );   break;
      case 3:  motors0(   65, -120,   70 );   break;
      case 4:  motors0(  150, -130,    0 );   break;
      case 5:  motors0(  120, - 65, - 70 );   break;
      case 6:  motors0(  150,    0, -165 );   break;
      case 7:  motors0(   80,   70, -150 );   break;
      case 8:  motors0(    0,  130, -150 );   break;
      case 9:  motors0( - 70,  120, - 80 );   break;
      case 10: motors0( -140,  130,    0 );   break;
      case 11: motors0( -120,   60,   70 );   break;
      case 88: motors0(   0,     0,    0 );   break;
    }
  }
  else if (speed == 2) {
    switch(dir) {
      case 0:  motors0( -80,   0,  95 );   break;
      case 1:  motors0( -45, -45,  95 );   break;
      case 2:  motors0(   0, -80,  95 );   break;
      case 3:  motors0(  40, -80,  55 );   break;
      case 4:  motors0(  90, -80,   0 );   break;
      case 5:  motors0(  80, -40, -55 );   break;
      case 6:  motors0(  90,   0,-105 );   break;
      case 7:  motors0(  45,  40, -90 );   break;
      case 8:  motors0(   0,  80, -95 );   break;
      case 9:  motors0( -45,  80, -55 );   break;
      case 10: motors0( -85,  80,   0 );   break;
      case 11: motors0( -80,  40,  55 );   break;
      case 88: motors0(   0,   0,   0 );   break;
    }
  }
  else {
    switch(dir) {
      case 0:  motors0( -50,   0,  60 );   break;
      case 1:  motors0( -45, -45,  95 );   break;
      case 2:  motors0(   0, -80,  95 );   break;
      case 3:  motors0(  35, -60,  40 );   break;
      case 4:  motors0(  90, -80,   0 );   break;
      case 5:  motors0(  80, -40, -55 );   break;
      case 6:  motors0(  50,   0, -60 );   break;
      case 7:  motors0(  45,  40, -90 );   break;
      case 8:  motors0(   0,  80, -95 );   break;
      case 9:  motors0( -35,  60, -40 );   break;
      case 10: motors0( -85,  80,   0 );   break;
      case 11: motors0( -80,  40,  55 );   break;
      case 88: motors0(   0,   0,   0 );   break;
    }
  }
}


void PeanutKing_Soccer_V1::motors0(int16_t a, int16_t b, int16_t c) {
  int16_t r = compassRead();
  int16_t t = r<180 ? -r : 360 - r;
    motors( a+t, b+t, c+t );
}

void PeanutKing_Soccer_V1::motorControl(uint16_t mSpeed, uint16_t mAngle) {
  float a = (float)mAngle*pi/180;
  int16_t spd[3];
  
  int16_t c = compassRead();
  int16_t tAngle = c<180 ? -c : 360 - c;
  
  spd[0] = -(float)mSpeed * cos(a+pi/6) ;
  spd[1] = -(float)mSpeed * sin(a);
  spd[2] =  (float)mSpeed * cos(a-pi/6)*1.15;
  
  for ( int i=0; i<3; i++ ) {
    if (mSpeed!=0) {
      spd[i] = mapSpeed(spd[i]+tAngle);
      spd[i] = constrain(spd[i], -255, 255);
    }
  }
  motors( spd[0], spd[1], spd[2] );
}

int PeanutKing_Soccer_V1::mapSpeed (float oSpeed) {
  const int a1 = 3, a2 = 70, a3 = 220;
  const int b1 = 25, b2 = 60, b3 = 255;
  int nSpeed = 0;
  if ( abs(oSpeed) <= a1 ) nSpeed = 0;
  else if ( oSpeed >  a2 ) nSpeed = map(oSpeed, a2, a3, b2, b3);
  else if ( oSpeed < -a2 ) nSpeed = map(oSpeed, -a2, -a3, -b2, -b3);
  else if ( oSpeed >  a1 ) nSpeed = map(oSpeed, a1, a2, b1, b2);
  else if ( oSpeed < -a1 ) nSpeed = map(oSpeed, -a1, -a2, -b1, -b2);
  return nSpeed;
}

void PeanutKing_Soccer_V1::motors(int16_t a, int16_t b, int16_t c) {
  int16_t mc[]={a,b,c};
  for (int i=0; i<3; i++) {
    if (mc[i]>0&&mc[i]<=255){
      digitalWrite(dirPin[i], LOW);
      analogWrite(pwmPin[i], mc[i]);
    }
    else if (mc[i]<0&&mc[i]>=-255){
      digitalWrite(dirPin[i], HIGH);
      analogWrite(pwmPin[i], 0-mc[i]);
    }
    else{
      digitalWrite(dirPin[i], LOW);
      digitalWrite(pwmPin[i], LOW);
    }
  }
}

void PeanutKing_Soccer_V1::moveSmart(int spd, float angle){
  int spd1, spd2, spd3;
  spd1 = -spd*cos(angle*pi/180+pi/6);
  spd2 = -spd*sin(angle*pi/180);
  spd3 = (spd-15)*cos(angle*pi/180-pi/6);
  motors(spd1,spd2,spd3);
}

//                                  Sensor Read
// ================================================================

// compassRead ----------------------------------------------------
uint16_t PeanutKing_Soccer_V1::compassRead(void) {
  if ( !autoScanEnabled )
    compass = rawCompass(compass_address, GET_READING);
  //setHome  compass_address, SET_HOME
  return compass;
}

uint16_t PeanutKing_Soccer_V1::compoundEyeRead(uint8_t cmd) {
  static uint32_t lastFuntionTime = 0;
  static uint16_t eye[9], maxNum, maxReading;
  if ( millis() - lastFuntionTime > 50 ) {
    maxNum = 1, maxReading = 0;
    for (int i=1; i<9; i++) {
      eye[i] = rawCompoundEye(irPin[i-1]);
      //debugging use
      //Serial.print("eye[");Serial.print(i+1);Serial.print("]: ");Serial.println(eye[i]);
      if( eye[i]>maxReading ) {
        maxNum = i;
        maxReading = eye[i];
      }
    }
    if ( maxNum == 1 || maxNum == 8 )
      eye[0] = ( (eye[1]+eye[8])*2 -eye[7]-eye[2])/3;
    else
      eye[0] = (eye[1]+eye[8]) /3;
    
    if ( eye[0]>maxReading ) {
      maxNum = 0;
      maxReading = eye[0]<1023 ? eye[0] : 1023;
    }
    int16_t
      upper = (maxEye==12) ? eye[1] : eye[maxEye+1],
      lower = (maxEye==1) ? eye[12] : eye[maxEye-1],
      add = 15.0 * (upper-lower) / (eye[maxEye] - ((upper<lower) ? upper : lower) );
    
    eyeAngle = 30 * maxEye + add;
    
    eyeAngle += ( eyeAngle>=30 ) ? -30 : 330;
    
    lastFuntionTime = millis();
  }
  if ( cmd<=8 )
    return eye[cmd];
  else if ( cmd==9 )
    return maxNum;
  else if ( cmd==10 )
    return maxReading;
}

// buttonRead -----------------------------------------------------
bool PeanutKing_Soccer_V1::buttonRead(uint8_t button_no) {
  /*
  if ( ROBOT_VERSION == 1 )
    return !digitalRead(buttonPin[0]);
  else {
    if(y==0)
      return digitalRead(buttonPin[y]);
    else if(y==1||y==2)
      return !digitalRead(buttonPin[y]);
  }*/
  
  if ( button_no == 1 || button_no == 2 ) 
    return rawButton(buttonPin[button_no]);
  else
    return 0;
}

bool PeanutKing_Soccer_V1::buttonPress(uint8_t y) {
  static bool lastButton[3] = {false};
  bool 
    buttonPressed,
    thisButton = buttonRead(y);
    buttonPressed = ( thisButton && !lastButton[y] );
    lastButton[y] = thisButton;
  return buttonPressed;
}

// ultrasonicRead -------------------------------------------------
uint16_t PeanutKing_Soccer_V1::ultrasonicRead(uint8_t ultrasonic_no) {
  if ( !autoScanEnabled )
    ultrasonic[ultrasonic_no] = rawUltrasonic(trigPin[ultrasonic_no], echoPin[ultrasonic_no]);
  if ( ultrasonic_no<0 || ultrasonic_no>=4 ) return 999;
  return ultrasonic[ultrasonic_no];
}


// ==============================================================
// ======================      Testing     ======================


void PeanutKing_Soccer_V1::Testing (void) {
  uint16_t MaxReading;
  uint16_t ults[4], angle;
  uint8_t  maxNum, goundColor[3];
    /*
  static int ms = 0;// ms[3] = {0};
  static bool accel = true;
  motors(ms, ms, ms);
  if (ms==255) accel = false;
  else if (ms==-255) accel = true;
  if (accel)
    ms++;
  else 
    ms--;
  delay(10);
  */
/*
  static byte ledStatus = 0;
  switch (ledStatus) {
    case 0:
    ledShow(0, 100, 0, 0, 0);
    break;
    case 1:
    ledShow(0, 0, 100, 0, 0);
    break;
    case 2:
    ledShow(0, 0, 0, 100, 0);
    break;
    case 3:
    ledShow(0, 100, 100, 100, 0);
    break;
    case 4:
    ledShow(0, 0, 0, 0, 0);
  }
  ledStatus = ledStatus == 4 ? 0 : ledStatus+1; 
  */
  
  angle = compassRead();
  Serial.print("Compass: ");
  Serial.print(angle);
  
  MaxReading = compoundEyeRead(10);
  maxNum = compoundEyeRead(9);
  Serial.print("   maxNum: ");
  Serial.print(maxNum);
  Serial.print("   MaxReading: ");
  Serial.print(MaxReading);
  Serial.println("");
  for (int i=0; i<=8; i++) {
    Serial.print(compoundEyeRead(i));
    Serial.print("  ");
  }
  Serial.println("");
  
  Serial.print("xsonic:  ");
  for (int i=0; i<4; i++ ) {
    ults[i] = ultrasonicRead(i);
    Serial.print(ults[i]);
    Serial.print("  ");
  }
  Serial.println("");
  
  for (int i=0; i<3; i++ ) {
	goundColor[i] = whiteLine(i);
    switch( goundColor[i] ) {
      case black:   Serial.println("black  ");  break;
      case white:   Serial.println("white  ");  break;
      case grey:    Serial.println("grey   ");  break;
      case red:     Serial.println("red    ");  break;
      case green:   Serial.println("green  ");  break;
      case blue:    Serial.println("blue   ");  break;
      case yellow:  Serial.println("yellow ");  break;
      case cyan:    Serial.println("cyan   ");  break;
      case magenta: Serial.println("magenta");  break;
    }
  }
  Serial.println("");
  if ( goundColor[right]==white || goundColor[left]==white )
    ledShow(0, 100, 100, 100, 0);
  else
    ledShow(0, 0, 0, 0, 0);
  
  ledUpdate();
}



