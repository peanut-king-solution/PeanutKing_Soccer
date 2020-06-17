#include "PeanutKing_Soccer_V2.h"

static PeanutKing_Soccer_V2* V2bot = NULL;

PeanutKing_Soccer_V2::PeanutKing_Soccer_V2(void) :
  tcsblPin(32),
  ledPin  (33),
  actledPin(30),
  buttonPin{42, 47, 48},

  pwmPin   { 5,  4,  3,  2},
  //dirPin[4]   = {22, 23, 24, 25},  // v2.1
  dirPin   {12, 10,  8,  6},  // v2.2
  dir2Pin  {13, 11,  9,  7},
  diagPin  {50, 51, 52, 53},
  // xsound:     xf, xl. xr. xb
  trigPin  {37, 35, 34, 36},
  echoPin  {41, 39, 38, 40},
  // rgbs:       s0, s1, s2, s3
  tcsSxPin {43, 44, 45, 46},
  tcsRxPin {28, 27, 29, 26},

  irPin    {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11}
  {
  if (V2bot == NULL)  {
    V2bot = this;
  }
}

ISR (TIMER1_COMPA_vect) {
  if (V2bot != NULL ) {
    V2bot->autoScanning();
  }
}


// initialize all IOs, Serial.begin, I2C, timer interrupt, 
// External interrupt different settings depends on version number 
void PeanutKing_Soccer_V2::init(uint8_t mode) {
  Wire.begin();
  Serial.begin(9600);
  Serial1.begin(9600);
  
  for (uint8_t i=0; i<4; i++) {
    pinMode(echoPin[i], INPUT);
    pinMode(trigPin[i], OUTPUT);
    pinMode(pwmPin[i],  OUTPUT);
    pinMode(dirPin[i],  OUTPUT);
    pinMode(dir2Pin[i], OUTPUT);
    pinMode(diagPin[i], OUTPUT);
    pinMode(tcsSxPin[i], OUTPUT);
    pinMode(tcsRxPin[i], INPUT);
    digitalWrite(diagPin[i], HIGH);
  }
  digitalWrite(tcsSxPin[0], HIGH);  
  digitalWrite(tcsSxPin[1], HIGH);
  
  for (uint8_t i=0; i<12; i++)
    pinMode(irPin[i], INPUT);
  for (uint8_t i=0; i<3; i++)
    pinMode(buttonPin[i], INPUT);
  
  pinMode(actledPin, INPUT);
  digitalWrite(actledPin, HIGH);
  
  lcdSetup();
  
  ledSetup(0, ledPin, numLEDs);
  if ( !ledEnabled ) {
    ledShow(255, 0, 0, 0, 0);
    ledUpdate();
  }
  delay(5);
  
  ledSetup(1, tcsblPin, 1);
  if ( mode == 0 ) 
    ledShow(1, 255, 255, 255, 255, 1);
  else
    ledShow(1, 0, 0, 0, 0, 1);
    
  ledUpdate(1);
  
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

//                                  Sensor Read
// ================================================================

// buttonRead -----------------------------------------------------
bool PeanutKing_Soccer_V2::buttonRead(uint8_t button_no) {
  if ( button_no == 1 || button_no == 2 ) 
    return rawButton(buttonPin[button_no]);
  else
    return 0;
}

bool PeanutKing_Soccer_V2::buttTrigRead(uint8_t y) {
  bool temp = buttonTriggered[y];
  buttonTriggered[y] = false;
  return temp;
}

void PeanutKing_Soccer_V2::buttons(void) {
  static bool lastButton[3] = {false};

  for (uint8_t i=0; i<3; i++) {
    button[i] = rawButton(buttonPin[i]);
    buttonPressed[i] = ( button[i] && !lastButton[i] );
    buttonReleased[i] = ( !button[i] && lastButton[i] );  

    if ( buttonPressed[i] )    buttonTriggered[i] = true;

    lastButton[i] = button[i];
  }
}

// compassRead ----------------------------------------------------
uint16_t PeanutKing_Soccer_V2::compassRead(void) {
  if ( !autoScanEnabled )
    compass = rawCompass(compass_address, GET_READING);
  
  //rawAccel compass_address, 0x57
  //rawGyro  compass_address, 0x56
  //rawCompass66  16, 0x55
// Software Set Compass Home -------------------------------------
  //setHome  compass_address, SET_HOME
  return compass;
}

// return single eye reading --------------------------------------
uint16_t PeanutKing_Soccer_V2::compoundEyeRead (uint8_t eye_no) {
  if ( !autoScanEnabled )
    compoundEyes();
  
  if ( eye_no >0 && eye_no <= 12 )
    return eye[eye_no];
  else if ( eye_no == 13 ) 
    return maxEye;
  else if ( eye_no == 14 ) 
    return eye[maxEye];
  else 
    return 0;
}

void PeanutKing_Soccer_V2::compoundEyes(void) {
  maxEye = 1;
  minEye = 1;
  for (int i=1; i<13; i++) {
    eye[i] = rawCompoundEye(irPin[i-1]);
  //debugging use
  //Serial.print("eye[");Serial.print(i+1);Serial.print("]: ");Serial.println(eye[i]);
    if( eye[i]>eye[maxEye] )
      maxEye = i;
    else if( eye[i]<eye[minEye] )
      minEye = i;
  }
  
  int16_t
    upper = (maxEye==12) ? eye[1] : eye[maxEye+1],
    lower = (maxEye==1) ? eye[12] : eye[maxEye-1],
    add = 15.0 * (upper-lower) / (eye[maxEye] - ((upper<lower) ? upper : lower) );
  
  eyeAngle = 30 * maxEye + add;
  
  eyeAngle += ( eyeAngle>=30 ) ? -30 : 330;
}

// ultrasonicRead -------------------------------------------------
uint16_t PeanutKing_Soccer_V2::ultrasonicRead(uint8_t ultrasonic_no) {
  if ( !autoScanEnabled )
    ultrasonic[ultrasonic_no] = rawUltrasonic(trigPin[ultrasonic_no], echoPin[ultrasonic_no]);
  if ( ultrasonic_no<0 || ultrasonic_no>=4 ) return 999;
  return ultrasonic[ultrasonic_no];
}

uint8_t PeanutKing_Soccer_V2::floorColorReadRaw(uint8_t pin_no, uint8_t mono) {
  const uint8_t& out = tcsRxPin[pin_no];
  
  digitalWrite(tcsSxPin[2], LOW);
  digitalWrite(tcsSxPin[3], LOW);
  delayMicroseconds(100);
  colorRGB[pin_no].r = rawMonoColor(out);
  
  digitalWrite(tcsSxPin[3], HIGH);
  delayMicroseconds(100);
  colorRGB[pin_no].b = rawMonoColor(out);
  delayMicroseconds(100);
  colorRGB[pin_no].g = rawMonoColor(out);
  
  if ( mono == red )        return colorRGB[pin_no].r;
  else if ( mono == green ) return colorRGB[pin_no].g;
  else if ( mono == blue )  return colorRGB[pin_no].b;
  else return 0;
}

// return single color sensor reading
uint8_t PeanutKing_Soccer_V2::floorColorRead(uint8_t pin_no) {
  floorColorReadRaw(pin_no);
  //colorRGB[pin_no].b *= 1.15;
  
  hsv_t& op = colorHSV[pin_no];
  op = rgb2hsv(colorRGB[pin_no]);
  
  isWhite[pin_no] = ( op.s < 10 && op.v > 85 );
  
  // COLOR decision making
  if ( op.v < 30 )                     return black;
  else if ( op.s < 10 && op.v > 150 )  return white;
  else if ( op.h < 50 || op.h > 315 )  return red;
  else if ( op.h < 100 )               return yellow;
  else if ( op.h < 175 )               return green;
  else if ( op.h < 250 )               return blue;
  else                                 return magenta;
}

bool PeanutKing_Soccer_V2::whiteLineCheck(uint8_t pin_no) {
  floorColorRead(pin_no);
  return isWhite[pin_no];
}



/* =============================================================================
 *                                  Motors
 * ============================================================================= */

// simple motor turn, motor_no cannot add, one by one 
void PeanutKing_Soccer_V2::motorSet(uint8_t motor_no, int16_t speed) {
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

void PeanutKing_Soccer_V2::motorControl(float mAngle, float mSpeed, float rotate) {
  int16_t mc[4];

  mc[0] = mSpeed*sin( (mAngle+45.0)*pi/180.0 );
  mc[1] = mSpeed*cos( (mAngle+45.0)*pi/180.0 );
  mc[2] = -mc[0];
  mc[3] = -mc[1];

  for(int8_t i=3; i>=0; i--) {
    motorSet(i, mc[i] + rotate);
  }
}

void PeanutKing_Soccer_V2::move(int16_t speed_X, int16_t speed_Y) {
  double mAngle = atan((double)speed_Y/(double)speed_X) * pi;
  if ( speed_X<0 ) mAngle += 180;
  if ( mAngle<0 )  mAngle += 360;
  
  uint16_t mSpeed = sqrt( speed_X*speed_X + speed_Y*speed_Y );
  
  moveSmart(mAngle, mSpeed);
}

// motor move + compass as reference
void PeanutKing_Soccer_V2::moveSmart(uint16_t angular_direction, int16_t speed, int16_t angle, uint8_t precision) {
  if (angle >= 360) angle = 0;

  int16_t c = compassRead();
  c -= angle;
  int16_t rotation = c < 180 ? -c : 360 - c;
  
  Serial.print("angle = "); Serial.println(angle);
  Serial.print("c = "); Serial.println(c);
  Serial.print("rotation = "); Serial.println(rotation);

  //speed - 50
  //rotation = abs(speed) < 120 ? rotation : rotation * 1.5;
  rotation = rotation * (precision+3)/12;
  if ( speed==0 ) {
    if (rotation>10) {
      rotation = rotation < 35 ? 35 : rotation;
    }
    if (rotation<-10) {
      rotation = rotation >-35 ?-35 : rotation;
    }
  }
  motorControl(angular_direction, speed, rotation);
}

void PeanutKing_Soccer_V2::motorStop(void) {
  for(uint8_t i=0; i<4; i++) {
    motorSet(i, 0);
  }
}


void PeanutKing_Soccer_V2::autoScanning(void) {
  static bool wasWhite[4] = {false}, ultsBlinkState[4] = {false};
  static uint32_t ultsBlinkTimer[4] = {0};
  static int8_t autoScanTicks = -1;
  uint8_t j = 0;
  
  uint16_t currentSensor = autoScanSensors;
  
  systemTime = (millis()/10) % 100;
  sysTicks ++;
  
  buttons();
  
  if ( buttonPressed[0] ) {
    motorEnabled = !motorEnabled;
    if ( !motorEnabled ) {
      motorStop();
    }
  }
  
  if ( !autoScanEnabled ) return;
  
  do {
    autoScanTicks++;
    if (autoScanTicks>16) autoScanTicks=0;
    currentSensor = autoScanSensors&(1<<autoScanTicks);
  } while( currentSensor == 0 );
  
  switch (currentSensor) {
    case COMPASS:
    case COMPOUNDEYE:
      sei();    //allow interrupts
      compass = rawCompass(compass_address, GET_READING);
      cli();    //disable interrupts
      compoundEyes();

    break;
    case ULTRASONIC0:
    case ULTRASONIC1:
    case ULTRASONIC2:
    case ULTRASONIC3:
      j = autoScanTicks-4;
      ultrasonic[j] = rawUltrasonic(trigPin[j], echoPin[j]);
    break;
    case COLORSENSOR0:
    case COLORSENSOR1:
    case COLORSENSOR2:
    case COLORSENSOR3:
      j = autoScanTicks-8;
      groundColor[j]  = floorColorRead(j);
      
      onBound[j] = isWhite[j] && ( ultrasonic[j]>25 && ultrasonic[j]<34 );
      outBound[j] = (!isWhite[j] && ultrasonic[j]<31) && ( !outBound[j] || wasWhite[j] );
      wasWhite[j] = isWhite[j];
    break;
  }
  
  if (ledEnabled) {
    // ultrasonic range 5-200 -> led delay 2-100
    for(uint8_t i=0; i<4; i++) {
      uint16_t limit = constrain(ultrasonic[i]/2, 2, 125);
      if ( sysTicks - ultsBlinkTimer[i] > limit ) {
        uint8_t e = ultsBlinkState[i] ? 127 : 0;
        switch (i) {
          case front:
            ledSetPixels(1<<0, e, 0, 0, 0);
          break;
          case left:
            ledSetPixels(1<<6, e, 0, 0, 0);
          break;
          case right:
            ledSetPixels(1<<2, e, 0, 0, 0);
          break;
          case back:
            ledSetPixels(1<<4, e, 0, 0, 0);
        }
        ultsBlinkState[i] = !ultsBlinkState[i];
        ultsBlinkTimer[i] = sysTicks;
      }
    }
    ledClear();
    for(uint8_t i=0; i<4; i++) {
      uint8_t c = isWhite[i] ? 127 : 0;
      switch (i) {
        case front:
          ledSetPixels(1<<7, 0, 0, 0, c);
        break;
        case left:
          ledSetPixels(1<<5, 0, 0, 0, c);
        break;
        case right:
          ledSetPixels(1<<1, 0, 0, 0, c);
        break;
        case back:
          ledSetPixels(1<<3, 0, 0, 0, c);
        break;
      }
    }
    uint16_t AngleLED = 360 - compass;
    uint8_t a = AngleLED<350 ? (AngleLED+10)/45 : (AngleLED-350)/45;
    uint8_t b = AngleLED>=10 ? (AngleLED-10)/45 : (AngleLED+350)/45;
    ledSetPixels( 1<<a | 1<<b, 0, 0, 180, 0 );
    
    if ( eye[maxEye] > EYEBOUNDARY ) {
      uint8_t c = eyeAngle<350 ? (eyeAngle+10)/45 : (eyeAngle-350)/45;
      uint8_t d = eyeAngle>=10 ? (eyeAngle-10)/45 : (eyeAngle+350)/45;
      ledSetPixels( 1<<c | 1<<d, 0, 180, 0, 0);
    }
    ledUpdate();
  }
}


/* =============================================================================
 *                                  Testing
 * ============================================================================= */
void PeanutKing_Soccer_V2::lcdMenu(void) {
  static uint32_t lcdTime = 0;
  static int8_t page = 0;
  static int8_t lastPage = 1;
  static uint16_t ticks = 0;
  
  ledEnabled = false;
  //if      ( buttTrigRead(1) ) page--;
  //else 
  if ( buttTrigRead(2) ) page++;
  else if ( millis() - lcdTime < 200) {
    delay(2);
    return;
  }
  else
    ticks++;

  lcdTime = millis();
  
  if      ( page > PAGEUPPERLIMIT ) page = PAGELOWERLIMIT;
  //else if ( page < PAGELOWERLIMIT ) page = PAGEUPPERLIMIT;
  
  if ( page != lastPage )  {
    ticks = 0;
    lcdClear();
    switch(page) {
      case 0:
        setScreen(0, 0, "Press A To Start");
      break;
      case 1:
        setScreen(0, 0, "1 CompassUP");
        setScreen(0, 1, "  CompassDn");
      break;
      case 2:
        setScreen(0, 0, "2 Eye");
        setScreen(0, 1, "Max:");
      break;
      case 3:
        setScreen(0, 0, "3 ULTRASONIC");
      break;
      case 4:
        setScreen(0, 0, "4 ColorSense");
      break;
      case 5:
        setScreen(0, 0, "5 LED Test");
        autoScanEnabled = false;
      break;
      case 6:
        setScreen(0, 0, "6 Motor Test");
        motorEnabled = true;
      break;
    }
    switch(lastPage) {
      case 0:
      
      break;
      case 1:
      
      break;
      case 2:
      
      break;
      case 3:
      
      break;
      case 4:
      
      break;
      case 5:
        autoScanEnabled = true;
        ledShow(255, 0, 0, 0, 0);
        ledUpdate();
      break;
      case 6:
        motorEnabled = false;
        motorStop();
      break;
    }
  }
  delay(1);
  switch(page) {
    case 0:
    break;
    case 1:
      setScreen(12, 0, (int16_t)compass);
    break;
    case 2:
      setScreen(5, 1, maxEye);
      print(" ");
      setScreen(10, 1, eye[maxEye]);
    break;
    case 3:
      setScreen(0, 1, ultrasonic[0]);
      setScreen(4, 1, ultrasonic[1]);
      setScreen(8, 1, ultrasonic[2]);
      setScreen(12, 1, ultrasonic[3]);
    break;
    case 4:
      setScreen(0, 1, isWhite[0]);
      setScreen(4, 1, isWhite[1]);
      setScreen(8, 1, isWhite[2]);
      setScreen(12, 1, isWhite[3]);
    break;
    case 5:
      ledTest();
    break;
    case 6:
      motorTest();
    break;
  }
  lastPage = page;
}

void PeanutKing_Soccer_V2::testProgram (void) {
  uint8_t unit = testLED;
  
  enableScanning(false, 0, false);
  
  /*
  static uint32_t lcdTime = 0;
  static int8_t page = 0;
  static int8_t lastPage = 1;
  static uint16_t ticks = 0;
  
  if ( buttTrigRead(2) ) page++;
  else if ( millis() - lcdTime < 200) {
    delay(2);
    return;
  }
  else
    ticks++;

  lcdTime = millis();
  
  enableScanning(true);
  debug(ALLSENSORS);
  */
  
  setScreen(0, 0, "testProgram");
  while(true) {
    switch(unit) {
      case testLED:
        ledTest();
      break;
      case testMotor:
        motorTest();
      break;
      case testCompass:
        setScreen(12, 0, (int16_t)compass);
        print("  ");
      break;
      case testUltrasonic:
        setScreen(0, 1, ultrasonic[0]);
        setScreen(4, 1, ultrasonic[0]);
        setScreen(8, 1, ultrasonic[0]);
        setScreen(12, 1, ultrasonic[0]);
      break;
      case testCompoundeye:
        setScreen(5, 1, maxEye);
        print(" ");
        setScreen(10, 1, eye[maxEye]);
        print("  ");
      break;
      case testColor:
        setScreen(0, 1, isWhite[0]);
        setScreen(4, 1, isWhite[1]);
        setScreen(8, 1, isWhite[2]);
        setScreen(12, 1, isWhite[3]);
      break;
      case testBT:
        btTest();
      break;
    }
    if ( buttTrigRead(1) ) {
      motorStop();
      unit++;
      //ticks = 0;
      lcdClear();
      
      switch(unit) {
        case testLED:
          setScreen(0, 0, "LED Test");
        break;
        case testMotor:
          setScreen(0, 0, "6 Motor Test");
          motorEnabled = true;
        break;
        case testCompass:
          setScreen(0, 0, "CompassUP real");
          setScreen(0, 1, "CompassDown");
        break;
        case testUltrasonic:
          setScreen(0, 0, "3 ULTRASONIC");
        break;
        case testCompoundeye:
          setScreen(0, 0, "2 Eye");
          setScreen(0, 1, "Max:");
        break;
        case testColor:
          setScreen(0, 0, "4 ColorSense");
        break;
        case testBT:
          setScreen(0, 0, "4 testBT");
        break;
      }
    }
  }
}

// LED test ------------------------------------------------------
void PeanutKing_Soccer_V2::ledTest (uint8_t state) {
  static uint32_t ledTimer = 0;
  static uint8_t index = 0, i = 0, j = 0;
  uint32_t timeNow = millis();
  
  if ( state == STATERESET )
    index = 0, i = 0, j = 0;
  
  if ( timeNow - ledTimer > 250) {
    ledTimer = timeNow;
    index |= (1<<j);
    switch(i) {
      case 0:
      ledShow(index, 255, 0, 0, 0);
      break;
      case 1:
      ledShow(index, 0, 255, 0, 0);
      break;
      case 2:
      ledShow(index, 0, 0, 255, 0);
      break;
      case 3:
      ledShow(index, 0, 0, 0, 255);
      break;
    }
    ledUpdate();
    j++;
    if ( j==8 ) {
      j = 0;
      i++;
      if ( i==4 )
        i=0;
      index = 0;
    }
  }
}

// motor test ------------------------------------------------------
uint8_t PeanutKing_Soccer_V2::motorTest (void) {
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

void PeanutKing_Soccer_V2::btTest(void) {
  if (Serial1.available()) {
    char v = Serial1.read();
    Serial.print(v);
    /*
    char msg[20];
    int i =0;
    do {
      msg[i] = Serial1.read();
      i++;
    } while (msg[i]!=10);
    Serial.println(msg);*/
  }
  if (Serial.available()) {
    char v = Serial.read();
    Serial.print(v);
    Serial1.write(v);
  }
}

uint8_t PeanutKing_Soccer_V2::pressureTest(void) {
  compoundEyes();
  int16_t direct, speed = 180,
    eyeAngle = eyeAngle,
    x = (ultrasonicRead(left) - ultrasonicRead(right))/2,
    y = ultrasonicRead(back) - 15,
    ballAngle = maxEye,
    reading = eye[ballAngle];
    
  static pressureTestStatus state = chaseball;
  
  switch ( state ) {
    case chaseball:
      //setScreen(0, 1, "1 Chaseball");
      direct = eyeAngle;
      for (uint8_t i=0; i<4; i++) {
        if (ultrasonic[i] <10) {
          state = gohome;
          setScreen(0, 0, "go Home");
        }
      }
    break;
    case goal:
      //setScreen(0, 1, "2 goal");
      if (eyeAngle<180)
        direct = eyeAngle*1.5;
      else
        direct = eyeAngle * 1.5 - 180; //359 - (359-eyeAngle)*1.5
    
      if (ultrasonic[front] < 10) {
        state = gohome2;
        setScreen(0, 0, "go Home 2");
      }
    break;
    case gofront:
      
    break;
    case gohome:
    case gohome2:
      speed = 150;
      if ( x > 5 )
        direct = 270;
      else if ( x < -5 )
        direct = 90;
      else if ( y > 5 )
        direct = 180;
      else if ( y < -5 )
        direct = 0;
      else {
        direct = 180;
        speed = 0;
        if (state == gohome2) {
          state = chaseball;
          setScreen(0, 0, "chaseball");
        }
        else {
          state = goal;
          setScreen(0, 0, "goal");
        }
      }
  }
  if ( speed == 0 )
    motorStop();
  else
    moveSmart(direct,  speed);
  
  //ledTest();
}

void PeanutKing_Soccer_V2::bluetoothAttributes(void) {
  static btDataType btDataHeader = Idle;
  static uint8_t len = 0;

  if (Serial1.available()) {
    char v = Serial1.read();
    Serial.print(v);
    
    switch (btDataHeader) {
      case Idle:
        switch (v) {
          case 'D':
            btDataHeader = Attributes;
            break;
        }
        break;
      case Attributes:
        if (len<5) {
          btAttributes[len] = v-'0';//code.toInt();
          len ++;
        }
        else {
          len = 0;
          btDataHeader = Idle;
        }
        break;
    }
  }
}




/* ===============================================================================
 *                                  Advance Control
 * =============================================================================== */

void PeanutKing_Soccer_V2::bluetoothRemote(void) {
  static btDataType btDataHeader = Idle;
  static uint32_t btSendTimer = 0;
  static uint8_t btState = 0, len = 0;
  static int btAngle = 0;
  static String deg = "", dis = "", buttonVal = "";
  static float speed = 1.0;
  
  if (Serial1.available()) {
    char v = Serial1.read();
    //Serial.print(v);
    
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
            btDataHeader = Attributes;
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
              //Serial.print(btDegree); Serial.print(' ');
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
    Serial.print("buttun pressed ");
    Serial.print(btButtonIndex); Serial.print(btGestureCode); Serial.println(' ');

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
          if ( eye[maxEye] > EYEBOUNDARY ) {
            Chase(btDegree, btDistance, btRotate);
          }
          Back(btDegree, btDistance, btRotate);
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
      speed = 1.0;
      btRotate = 0;
    }
    // Execute
    /*
      Serial.print(btDegree); Serial.print(' ');
      Serial.print(btDistance); Serial.print(' ');
      Serial.print(btRotate); Serial.println(' ');
    */
    if ( btRotate==0 ) {
      moveSmart(btDegree, btDistance*speed, btAngle);
    }
    else
      motorControl(0, 0, btRotate);
  }
  else if (btDataHeader == DemoMode) {
    if ( eye[maxEye] > EYEBOUNDARY ) {
      Chase(btDegree, btDistance, btRotate);
    }
    Back(btDegree, btDistance, btRotate);
    moveSmart(btDegree, btDistance*speed, btAngle);
  }

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


//                                  strategy
// =================================================================================
void PeanutKing_Soccer_V2::Chase(int& direct, int& speed, int& rotation) {
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
  
    speed = reading > 500 ? BallPossessionSpeed : attackSpeed;
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
    /*
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

void PeanutKing_Soccer_V2::Back(int& direct, int& speed, int& rotation) {
  //  attack
  //  Defend
  //  MovingSpeed
  //  BallPossession
  //  Precision

  int16_t 
    defendSpeed = 50 + btAttributes[2]*12;          // 80
  speed = defendSpeed;
  int16_t
    y = ultrasonic[back] - (11 - btAttributes[2]) * 8,
    x = (ultrasonic[left] - ultrasonic[right])/2;

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


//                                  ???
// =================================================================================

void PeanutKing_Soccer_V2::debug(uint16_t sensorType) {
  static uint32_t sensorPrintTimer = 0;
  uint32_t timeNow = millis();
  autoScanEnabled = true;
  
  if ( timeNow - sensorPrintTimer < 1000 ) {
    delay(5);
    return;
  }
  sensorPrintTimer = timeNow;
  
  Serial.print("Ticks: ");
  Serial.print( timeNow/1000.0, 2);
  Serial.print("   systemTime: ");
  Serial.println(systemTime);
  
  if ( sensorType&COMPASS ) {
    Serial.print("Angle: ");
    Serial.print(compass);
    Serial.print("    ");
  }
  if ( sensorType&COMPOUNDEYE ) {
    Serial.print("maxEye: ");
    Serial.print(maxEye);
    Serial.print("   MaxReading: ");
    Serial.println(eye[maxEye]);
    Serial.print("Eyes:  ");
    for (int i=1; i<=12; i++) {
      Serial.print(eye[i]);
      Serial.print("  ");
    }
  }
  if ( sensorType&(COMPASS|COMPOUNDEYE) )
    Serial.println();
  
  if ( sensorType&(ULTRASONIC|COLORSENSOR) ) {
    for (int i=0; i<4; i++) {
      switch( i ) {
        case front:  Serial.print("Front");  break;
        case left:   Serial.print("Left ");  break;
        case right:  Serial.print("Right");  break;
        case back:   Serial.print("Back ");  break;
      }
      if ( sensorType&ULTRASONIC ) {
        Serial.print(" |  ultrasonic ");
        printSpace(ultrasonic[i], 3);
      }
      if ( sensorType&COLORSENSOR ) {
        Serial.print("  |  rgb  ");
        Serial.print(colorRGB[i].r);
        Serial.print(", ");
        Serial.print(colorRGB[i].g);
        Serial.print(", ");
        Serial.print(colorRGB[i].b);
        Serial.print("  |  hsv ");
        printSpace(colorHSV[i].h);
        Serial.print(", ");
        Serial.print(colorHSV[i].s);
        Serial.print(", ");
        Serial.print(colorHSV[i].v);
        Serial.print(isWhite[i] ? "  |  is white " : "  |  " );
        /*
        switch( GroundColor[i] ) {
          case black:   Serial.print("black  ");  break;
          case white:   Serial.print("white  ");  break;
          case grey:    Serial.print("grey   ");  break;
          case red:     Serial.print("red    ");  break;
          case green:   Serial.print("green  ");  break;
          case blue:    Serial.print("blue   ");  break;
          case yellow:  Serial.print("yellow ");  break;
          case cyan:    Serial.print("cyan   ");  break;
          case magenta: Serial.print("magenta");  break;
        }
        */
      }
      Serial.println();
    }
  }
  Serial.println();
}




