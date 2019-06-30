
#include "PeanutKing_Soccer.h"

PeanutKing_Soccer* singlebot = NULL;

PeanutKing_Soccer::PeanutKing_Soccer(uint8_t a) {
  if (singlebot == NULL)  {
    singlebot = this;
  }
}

ISR (TIMER1_COMPA_vect) {
  if (singlebot != NULL ) {
    singlebot->autoScanning();
  }
}

hsv PeanutKing_Soccer::rgb2hsv(rgb in) {
  hsv      out;
  int16_t  min, max, delta;

  min = in.r < in.g ? in.r : in.g;
  min = min  < in.b ? min  : in.b;

  max = in.r > in.g ? in.r : in.g;
  max = max  > in.b ? max  : in.b;
  
  out.v = max;                                // v
  delta = max - min;
  if ( max == 0 ) { // if max is 0, then r = g = b = 0
    out.s = 0;                                // s = 0
    out.h = NAN;                              // h is now undefined
  }
  else if ( delta < 1 ) { // grey color
    out.s = 0;
    out.h = 0;                                // undefined
  }
  else {
  // NOTE: if Max is == 0, this divide would cause a crash
    out.s = 255 * delta / max;                // s

    if ( in.g >= max )                    // > is bogus, just keeps compilor happy
      out.h = 120 + int16_t( in.b - in.r ) * 60 / delta;  // between cyan & yellow
    else
    if ( in.b >= max )    
      out.h = 240 + int16_t( in.r - in.g ) * 60 / delta;  // between magenta & cyan
    else {
      out.h = 360 + int16_t( in.g - in.b ) * 60 / delta;  // between yellow & magenta
      if ( out.h > 360 )
        out.h -= 360;
    }
  }
  return out;
}


// initialize all IOs, Serial.begin, I2C, timer interrupt, 
// External interrupt different settings depends on version number 
void PeanutKing_Soccer::init() {
  Wire.begin();
  Serial.begin(9600);
  Serial1.begin(9600);
  
  for (uint8_t i=0; i<4; i++) {
    pinMode(echoPin[i], INPUT);
    pinMode(trigPin[i], OUTPUT);
    pinMode(pwmPin[i], OUTPUT);
    pinMode(dirPin[i], OUTPUT);
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
  
  LCDSetup();
  
  ledSetup(0, ledPin, numLEDs);
  ledSetup(1, tcsblPin, 1);
  ledShow(0, 255, 255, 255, 0, 1);
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
}

void PeanutKing_Soccer::autoScanning(void) {
  static bool wasWhite[4] = {false}, ultsBlinkState[4] = {false};
  static uint32_t ultsBlinkTimer[4] = {0};
  static int8_t autoScanTicks = -1;
  
  uint16_t currentSensor = autoScanSensors;
  
  systemTime = (millis()/10) % 100;
  sysTicks ++;
  
  buttons();
  /*
  if ( buttonPressed[2] ) {
    motorEnabled = !motorEnabled;
    if ( !motorEnabled ) {
      stop();
    }
  }
  */
  
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
      Compass = rawCompass();
      cli();    //disable interrupts
      compoundEyes();
      if (ledEnabled) {
        ledClear();
        for(uint8_t i=0; i<4; i++) {
          uint8_t c = isWhite[i] ? 127 : 0;
          switch (i) {
            case front:
              ledAddPixels(1<<7, 0, 0, 0, c);
            break;
            case left:
              ledAddPixels(1<<5, 0, 0, 0, c);
            break;
            case right:
              ledAddPixels(1<<1, 0, 0, 0, c);
            break;
            case back:
              ledAddPixels(1<<3, 0, 0, 0, c);
            break;
          }
        }
        uint16_t AngleLED = 360 - Compass;
        uint8_t a = AngleLED<350 ? (AngleLED+10)/45 : (AngleLED-350)/45;
        uint8_t b = AngleLED>=10 ? (AngleLED-10)/45 : (AngleLED+350)/45;
        ledAddPixels( 1<<a | 1<<b, 0, 0, 150, 0 );
        
        if ( Eye[MaxEye] > EYEBOUNDARY ) {
          // eyeAngle -> 16 divisions
          uint8_t c = eyeAngle<350 ? (eyeAngle+10)/45 : (eyeAngle-350)/45;
          uint8_t d = eyeAngle>=10 ? (eyeAngle-10)/45 : (eyeAngle+350)/45;
          
          ledAddPixels( 1<<c | 1<<d, 0, 150, 0, 0);
        }
      }
      else {
        ledShow(255, 0, 0, 0, 0);
      }
    break;
    case ULTRASONIC0:
    case ULTRASONIC1:
    case ULTRASONIC2:
    case ULTRASONIC3:
      Xsonic[autoScanTicks-4] = rawUltrasonic(autoScanTicks-4);
    break;
    case COLORSENSOR0:
    case COLORSENSOR1:
    case COLORSENSOR2:
    case COLORSENSOR3:
      uint8_t j = autoScanTicks-8;
      GroundColor[autoScanTicks-8]  = goundColorRead(autoScanTicks-8);
      
      onBound[j] = isWhite[j] && ( Xsonic[j]>25 && Xsonic[j]<34 );
      outBound[j] = (!isWhite[j] && Xsonic[j]<31) && ( !outBound[j] || wasWhite[j] );
      wasWhite[j] = isWhite[j];
    break;
  }
  
  // ultrasonic range 5-200 -> led delay 2-100
  if (ledEnabled) {
    for(uint8_t i=0; i<4; i++) {
      uint16_t limit = constrain(Xsonic[i]/2, 2, 125);
      if ( sysTicks - ultsBlinkTimer[i] > limit ) {
        uint8_t e = ultsBlinkState[i] ? 127 : 0;
        switch (i) {
          case front:
            ledAddPixels(1<<0, e, 0, 0, 0);
          break;
          case left:
            ledAddPixels(1<<6, e, 0, 0, 0);
          break;
          case right:
            ledAddPixels(1<<2, e, 0, 0, 0);
          break;
          case back:
            ledAddPixels(1<<4, e, 0, 0, 0);
        }
        ultsBlinkState[i] = !ultsBlinkState[i];
        ultsBlinkTimer[i] = sysTicks;
      }
    }
  }
  ledUpdate();
}

void PeanutKing_Soccer::testProgram (void) {
  static bool start = false, ledStart = false, motorStart = false;

  if ( buttonRead(0) ) {
    start = !start;
    if ( !start ) {
      motorStop();
    }
  }
  if ( buttonRead(1) ) {
    ledStart = !ledStart;
    if ( !ledStart ) {
      ledShow(255, 0, 0, 0, 0);
      ledUpdate();
    }
  }
  if ( buttonRead(2) ) {
    motorStart = !motorStart;
    if ( !motorStart ) {
      motorStop();
    }
  }
  if ( motorStart )
    motorTest();
  if ( ledStart )
    ledTest();
  if ( !start ) {
    debug(ALLSENSOR);
  }
}

// LED test ------------------------------------------------------
void PeanutKing_Soccer::ledTest (uint8_t state) {
  static uint32_t lastTimeIn = 0;
  static uint8_t index = 0, i = 0, j = 0;
  
  if ( state == STATERESET )
    index = 0, i = 0, j = 0;
  
  if ( millis() - lastTimeIn > 250) {
    lastTimeIn = millis(); 
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
uint8_t PeanutKing_Soccer::motorTest (void) {
  static uint32_t lastTimeIn = 0;
  static uint8_t i = 0;

  if ( millis() - lastTimeIn > 1000) {
    lastTimeIn = millis();
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

void PeanutKing_Soccer::btTest(void) {
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

void PeanutKing_Soccer::debug(uint16_t sensorType) {
  const uint16_t updateInterval = 1000;
  static uint32_t lastTimeIn = 0;
  
  for (uint8_t i=0; i<3; i++) {
    if ( buttonPressed[i] ) {
      Serial.print("Button ");
      Serial.print(i);
      Serial.println("pressed. ");
    }
  }
  if ( millis() - lastTimeIn < updateInterval ) {
    delay(5);
    return;
  }
  lastTimeIn = millis();
  
  Serial.print("Ticks: ");
  Serial.print(lastTimeIn/1000.0, 2);
  Serial.print("   systemTime: ");
  Serial.println(systemTime);
  
  if ( sensorType&COMPASS ) {
    Serial.print("Angle: ");
    Serial.print(Compass, 2);
    Serial.print("    ");
    /*
    Serial.print("Gyro: ");
    Serial.print( rawGyro(), 2 );
    Serial.print("    ");
    Serial.print("Accel: ");
    Serial.print( rawAccel(), 2 );*/
  }
  if ( sensorType&COMPOUNDEYE ) {
    Serial.print("MaxEye: ");
    Serial.print(MaxEye);
    Serial.print("   MaxReading: ");
    Serial.print(Eye[MaxEye]);
    Serial.print("   EyeAngle: ");
    Serial.println(eyeAngle);
    Serial.print("Eyes:  ");
    for (int i=1; i<=12; i++) {
      Serial.print(Eye[i]);
      Serial.print("  ");
    }
  }
  if ( sensorType&(COMPASS|COMPOUNDEYE) )
    Serial.println();
  
  if ( sensorType&(ULTRASONIC|COLORSENSOR) ) {
    for (int i=0; i<4; i++) {
      switch( i ) {
        case front:  Serial.print("Front |  ");  break;
        case left:   Serial.print("Left  |  ");  break;
        case right:  Serial.print("Right |  ");  break;
        case back:   Serial.print("Back  |  ");  break;
      }
      if ( sensorType&ULTRASONIC ) {
        Serial.print("xsonic ");
        printSpace(Xsonic[i], 3);
      }
      if ( sensorType&COLORSENSOR ) {
        Serial.print("  |  rgb  ");
        Serial.print(colorRGB[i].r);
        Serial.print(", ");
        Serial.print(colorRGB[i].g);
        Serial.print(", ");
        Serial.print(colorRGB[i].b);
        Serial.print("  |  hsv ");
        printSpace(colorHSV[i].h, 3);
        Serial.print(", ");
        Serial.print(colorHSV[i].s);
        Serial.print(", ");
        Serial.print(colorHSV[i].v);
        Serial.print(isWhite[i] ? "  |  True " : "  |  False" );
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


void PeanutKing_Soccer::LCDMenu(void) {
  static uint32_t LCDTime = 0;
  static int8_t page = 0;
  static int8_t lastPage = 1;
  static uint16_t ticks = 0;
  
  if      ( buttonRead(1) ) page--;
  else if ( buttonRead(2) ) page++;
  else if ( millis() - LCDTime < 200) {
    delay(2);
    return;
  }
  else
    ticks++;

  LCDTime = millis();
  
  if      ( page > PAGEUPPERLIMIT ) page = PAGELOWERLIMIT;
  else if ( page < PAGELOWERLIMIT ) page = PAGEUPPERLIMIT;
  
  if ( page != lastPage )  {
    ticks = 0;
    LCDClear();
    switch(page) {
      case 0:
        setScreen(0, 0, "0 Debug");
      break;
      case 1:
        setScreen(0, 0, "1 Angle");
      break;
      case 2:
        setScreen(0, 0, "2 Eye");
      break;
      case 3:
        setScreen(0, 0, "3 ULTRASONIC");
      break;
      case 4:
        setScreen(0, 0, "4 ColorSense");
        ledShow(255, 0, 0, 0, 0);
        ledUpdate();
      break;
      case 5:
        setScreen(0, 0, "5 LED Test");
      break;
      case 6:
        setScreen(0, 0, "6 Motor Test");
        ledShow(255, 0, 0, 0, 0);
        ledUpdate();
      break;
    }
  }
  delay(5);
  switch(page) {
    case 0:
    break;
    case 1:
      setScreen(0, 1, (int16_t)Compass);
      print("  ");
    break;
    case 2:
      //setScreen(0, 1, "Max:");
      setScreen(3, 1, MaxEye);
      print(" ");
      setScreen(10, 1, Eye[MaxEye]);
      print("  ");
    break;
    case 3:
      setScreen(0, 1, Xsonic[0]);
      LCDPrintSpace(Xsonic[0]);
      print(" ");
      print(Xsonic[1]);
      LCDPrintSpace(Xsonic[1]);
      print(" ");
      print(Xsonic[2]);
      LCDPrintSpace(Xsonic[2]);
      print(" ");
      print(Xsonic[3]);
      LCDPrintSpace(Xsonic[3]);
      print(" ");
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

//                                  Sensor Read
// =================================================================================
// buttonRead ------------------------------------------------------
bool PeanutKing_Soccer::buttonRead(uint8_t y) {
  bool temp = buttonTriggered[y];
  buttonTriggered[y] = false;
  return temp;
}

// compassRead -----------------------------------------------------
uint16_t PeanutKing_Soccer::compassRead(void) {
  if ( !autoScanEnabled )
    Compass = rawCompass();
  return Compass;
}

// return single eye reading ---------------------------------------
inline uint16_t PeanutKing_Soccer::compoundEyeRead (uint8_t eye_no) {
  //if ( !autoScanEnabled )
    //Eye[eye_no] = rawCompoundEye(eye_no);
  return rawCompoundEye(eye_no);
//  return Eye[eye_no];
}

// ultrasonicRead ------------------------------------------------------
uint16_t PeanutKing_Soccer::ultrasonicRead(uint8_t Xsonic_no) {
  if ( !autoScanEnabled )
    Xsonic[Xsonic_no] = rawUltrasonic(Xsonic_no);
  return Xsonic[Xsonic_no];
}

// return single color sensor reading
uint8_t PeanutKing_Soccer::goundColorRead(uint8_t pin) {
  const uint8_t& out = tcsRxPin[pin];
  
  digitalWrite(tcsSxPin[2], LOW);
  digitalWrite(tcsSxPin[3], LOW);
  delayMicroseconds(100);
  colorRGB[pin].r = rawMonoColor(out);
  
  digitalWrite(tcsSxPin[3], HIGH);
  delayMicroseconds(100);
  colorRGB[pin].b = rawMonoColor(out);
  
  digitalWrite(tcsSxPin[2], HIGH);
  delayMicroseconds(100);
  colorRGB[pin].g = rawMonoColor(out);
  
  //colorRGB[pin].b *= 1.15;
  
  hsv& op = colorHSV[pin];
  op = rgb2hsv(colorRGB[pin]);
  
  isWhite[pin] = ( op.s < 10 && op.v > 85 );
  
  // COLOR decision making
  if ( op.v < 30 )                     return black;
  else if ( op.s < 10 && op.v > 150 )  return white;
  else if ( op.h < 50 || op.h > 315 )  return red;
  else if ( op.h < 100 )               return yellow;
  else if ( op.h < 175 )               return green;
  else if ( op.h < 250 )               return blue;
  else                                 return magenta;
}


//                                  Motors
// =================================================================================

void PeanutKing_Soccer::move(int16_t speed_X, int16_t speed_Y) {
  double mAngle = atan((double)speed_Y/(double)speed_X) * pi;
  if ( speed_X<0 ) mAngle += 180;
  if ( mAngle<0 )  mAngle += 360;
  
  uint16_t mSpeed = sqrt( speed_X*speed_X + speed_Y*speed_Y );
  
  moveSmart(mAngle, mSpeed);
}

// motor move + compass as reference
void PeanutKing_Soccer::moveSmart(uint16_t angular_direction, int16_t speed) {
  int16_t c = Compass;
  int16_t rotation = c < 180 ? -c : 360 - c;
  
  //rotation = abs(speed) < 120 ? rotation : rotation * 1.5;
  rotation*=0.7;
  motorControl(angular_direction, speed, rotation);
}

void PeanutKing_Soccer::motorControl(float mAngle, float mSpeed, float rotate) {
  int16_t mc[4];

  mc[0] = mSpeed*sin( (mAngle+45.0)*pi/180.0 );
  mc[1] = mSpeed*cos( (mAngle+45.0)*pi/180.0 );
  mc[2] = -mc[0];
  mc[3] = -mc[1];

  for(int8_t i=3; i>=0; i--) {
    motorSet(i, mc[i] + rotate);
  }
}

void PeanutKing_Soccer::motorStop(void) {
  for(uint8_t i=0; i<4; i++) {
    motorSet(i, 0);
  }
}

// simple motor turn, motor_no cannot add, one by one 
void PeanutKing_Soccer::motorSet(uint8_t motor_no, int16_t speed) {
  //static int16_t previousSpeed[4] = {0,0,0,0};
  if ( motorEnabled == false ) speed = 0;
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

// turn on/off specific sensory system scannig function
// CompoundEye + Compass + Ultrasonic + ColorSense
void PeanutKing_Soccer::enableScanning(bool enabled, uint8_t sensorType) {
  if ( true ) {
  }
}

// col(0-15), row(0-1) --------------------------------------------
void PeanutKing_Soccer::setScreen(uint8_t col, uint8_t row, char string[]) {
  setCursor(col, row);
  print(string);
}

void PeanutKing_Soccer::setScreen(uint8_t col, uint8_t row, int16_t numbers) {
  setCursor(col, row);
  print(numbers);
}

void PeanutKing_Soccer::bluetoothSend(char string[]) {
// send char
  Serial1.print(string);
}

void PeanutKing_Soccer::bluetoothReceive(void) {
// send char
  Serial1.read();
}

void PeanutKing_Soccer::printSpace(uint32_t data, uint8_t digit) {
  for ( int i=1; i<digit; i++ ) {
    if ( data < pow(10, i) )   Serial.print(" ");
  }
  Serial.print(data);
}

void PeanutKing_Soccer::LCDPrintSpace(int16_t x) {
  if ( x<100 )
    print(" ");
  if ( x<10 )
    print(" ");
}


// Hidden functions ---------------------------------------------
void PeanutKing_Soccer::buttons(void) {
  static bool lastButton[3] = {false};

  for (uint8_t i=0; i<3; i++) {
    Button[i] = rawButton(i);
    buttonPressed[i] = ( Button[i] && !lastButton[i] );
    buttonReleased[i] = ( !Button[i] && lastButton[i] );  

    if ( buttonPressed[i] )    buttonTriggered[i] = true;

    lastButton[i] = Button[i];
  }
}

void PeanutKing_Soccer::compoundEyes(void) {
  MaxEye = 1;
  MinEye = 1;
  for (int i=1; i<13; i++) {
    Eye[i] = rawCompoundEye(i);
  //Serial.print("eye[");Serial.print(i+1);Serial.print("]: ");Serial.println(eye[i]); //debugging use
    if( Eye[i]>Eye[MaxEye] )
      MaxEye = i;
    else if( Eye[i]<Eye[MinEye] )
      MinEye = i;
  }
  
  int16_t
    upper = (MaxEye==12) ? Eye[1] : Eye[MaxEye+1],
    lower = (MaxEye==1) ? Eye[12] : Eye[MaxEye-1],
    add = 15.0 * (upper-lower) / (Eye[MaxEye] - ((upper<lower) ? upper : lower) );
  
  eyeAngle = 30 * MaxEye + add;
  
  eyeAngle += ( eyeAngle>=30 ) ? -30 : 330;
}


//                                  RAW
// =================================================================================

// Software Set Compass Home -------------------------------------
uint16_t PeanutKing_Soccer::setHome(void) { 
  uint8_t received_byte[2] = {0,0};  
  uint8_t i=0;
  uint16_t answer = 0;
  //uint16_t counter =0;
  Wire.beginTransmission(compass_address);  
  Wire.write(SET_HOME);
  Wire.endTransmission(); 
  Wire.requestFrom(compass_address, 2); 
  while (Wire.available() ) { received_byte[i++] = Wire.read(); }
  answer = received_byte[1]; 
  return answer;
}

// rawCompass ----------------------------------------------------
float PeanutKing_Soccer::rawCompass(void) {
  //uint8_t jerho[3] = {0,0,0};
  uint8_t received_byte[3] = {0,0,0};
  uint8_t i=0;
  uint16_t temp=0;
  float answer = 888;
  Wire.beginTransmission(compass_address);
  Wire.write(GET_READING);
  Wire.endTransmission();
  Wire.requestFrom(compass_address, 3);
  while (Wire.available()) {
    received_byte[i++] = Wire.read();
  }
  temp = received_byte[1]&0xFF;
  temp |= (received_byte[2]<<8);
  //jerho[0] = received_byte[0];
  //jerho[1] = received_byte[1];
  //jerho[2] = received_byte[2];
  answer = temp/100.0;

  return answer;
}

// rawGyro -------------------------------------------------------
float PeanutKing_Soccer::rawGyro(void) {
  uint8_t received_byte[3] = {0,0,0};
  uint8_t i=0;  
  uint16_t temp=0;
  float answer = 888; 
  
  Wire.beginTransmission(compass_address);
  Wire.write(0x56);  
  Wire.endTransmission(); 
  Wire.requestFrom(compass_address, 3); 
  while (Wire.available()) { 
    received_byte[i++] = Wire.read(); 
  }
  temp = received_byte[1]&0xFF;  
  temp |= (received_byte[2]<<8);
  answer = temp/128.0;
  return answer;
}

// rawAccel -------------------------------------------------------
float PeanutKing_Soccer::rawAccel(void) {
  uint8_t received_byte[3] = {0,0,0};
  uint8_t i=0;  
  int16_t temp=0;
  float answer = 888; 
  
  Wire.beginTransmission(compass_address);
  Wire.write(0x57);  
  Wire.endTransmission(); 
  Wire.requestFrom(compass_address, 3); 
  while (Wire.available()) { 
    received_byte[i++] = Wire.read(); 
  }
  temp = received_byte[1]&0xFF;  
  temp |= (received_byte[2]<<8);

  answer = temp;

  return answer;
}

// rawUltrasonic ------------------------------------------------
uint16_t PeanutKing_Soccer::rawUltrasonic(uint8_t Xsonic_no) {
  uint32_t duration=0;
  uint16_t distance=0;
  if ( Xsonic_no<0 || Xsonic_no>=4 ) return 999;
  digitalWrite(trigPin[Xsonic_no], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin[Xsonic_no], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin[Xsonic_no], LOW);
  duration = pulseIn(echoPin[Xsonic_no], HIGH, 13000);
  distance = ( duration==0 ) ? 888 : duration*0.017; //0.034/2;
  return distance;
}

// rawButton  ---------------------------------------------------
inline bool PeanutKing_Soccer::rawButton(uint8_t button_no) {
  return !digitalRead(buttonPin[button_no]);
}

// return single eye reading  -----------------------------------
inline uint16_t PeanutKing_Soccer::rawCompoundEye (uint8_t eye_no) {
  return 1023 - analogRead(irPin[eye_no-1]);
}

// rawMonoColor -------------------------------------------------
uint8_t PeanutKing_Soccer::rawMonoColor(uint8_t out) {
  const uint16_t MAX = 900, MIN = 100;
  int16_t temp = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH, 2000);
  //static const uint8_t TCSARRAYCOUNT = 1;
  //uint16_t rread[TCSARRAYCOUNT],
  //for (uint8_t i=0; i<TCSARRAYCOUNT; i++)
  //colorRGB[pin].r = sort(rread, TCSARRAYCOUNT);
  if ( temp == 0 ) temp = MAX;
  
  // v2.1 robot 180-20
  temp = map(temp, MAX, MIN, 0, 255);
  temp = constrain(temp, 0, 255);
  
  return temp;
}

//                                  LEDs
// =================================================================================
void PeanutKing_Soccer::ledSetup (uint8_t x, uint8_t p, uint8_t n) {
  leds[x].port = portOutputRegister(digitalPinToPort(p));
  leds[x].mask = digitalPinToBitMask(p);
  leds[x].numLEDs = n;
  // Allocate new data -- note: ALL PIXELS ARE CLEARED
  leds[x].numBytes = n * 4; // Size of 'pixels' buffer below (3 bytes/pixel)
  if((leds[x].pixels = (uint8_t *)malloc(leds[x].numBytes))) {
    memset(leds[x].pixels, 0, leds[x].numBytes);
  }
  pinMode(p, OUTPUT);
  digitalWrite(p, LOW);
}

// not using adaFruit, we write our own library, will check if same \
// color, Which_led use the same methodology as module, 
// just like: LED_1 + LED_2.... if multiple color at same time}
// r = (r * brightness) >> 8;
// Offset:    W          R          G          B
// NEO_GRB  ((1 << 6) | (1 << 4) | (0 << 2) | (2))   GRBW 
// Set pixel color from separate R,G,B components:
void PeanutKing_Soccer::ledShow(uint8_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t x) {
  //static uint32_t lastTime = millis();
  for (uint8_t i=0; i<8; i++) {
    if ( n & (1<<i) ) {
      uint8_t *p = &leds[x].pixels[n * 4];  // 4 bytes per pixel
      if(n < numLEDs) {
        p[1] = r;                   // R
        p[0] = g;                   // G
        p[2] = b;                   // B
        p[3] = w;                   // W
      }
    }
  }
}

void PeanutKing_Soccer::ledAddPixels(uint8_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  for (uint8_t i=0; i<8; i++) {
    if ( n & (1<<i) ) {
      uint8_t *p = &leds[0].pixels[i * 4];  // 4 bytes per pixel
      if ( r > 0 )
        p[1] = r;                   // R
      if ( g > 0 )
        p[0] = g;                   // G
      if ( b > 0 )
        p[2] = b;                   // B
      if ( w > 0 )
        p[3] = w;                   // W
    }
  }
}

void PeanutKing_Soccer::ledClear(void) {
  for (uint8_t i=0; i<8; i++) {
    ledShow(i, 0, 0, 0, 0);
  }
  ledUpdate();
}

void PeanutKing_Soccer::ledUpdate(uint8_t x) {
  volatile uint16_t
    i    = leds[x].numBytes;  // Loop counter (numBytes)
  volatile uint8_t
   *port = leds[x].port,      // Output PORT register
    pinMask = leds[x].mask,   // Output PORT bitmask
   *ptr  = leds[x].pixels,    // Pointer to next byte
    b    = *ptr++,            // Current byte value
    hi   = *port |  pinMask,  // PORT w/output bit set high
    lo   = *port & ~pinMask,  // PORT w/output bit set low
    next = lo,
    bit  = 8;

  cli();
  // 20 inst. clocks per bit: HHHHHxxxxxxxxLLLLLLL  62.5ns/cycle
  // ST instructions:         ^   ^        ^       (T=0,5,13)
  asm volatile(
   "head20:"                   "\n\t" // Clk  Pseudocode    (T =  0)
    "st   %a[port],  %[hi]"    "\n\t" // 2    PORT = hi     (T =  2)
    "sbrc %[byte],  7"         "\n\t" // 1-2  if(b & 128)
     "mov  %[next], %[hi]"     "\n\t" // 0-1   next = hi    (T =  4)
    "dec  %[bit]"              "\n\t" // 1    bit--         (T =  5)
    "st   %a[port],  %[next]"  "\n\t" // 2    PORT = next   (T =  7)
    "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T =  8)
    "breq nextbyte20"          "\n\t" // 1-2  if(bit == 0) (from dec above)
    "rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 10)
    "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 12)
    "nop"                      "\n\t" // 1    nop           (T = 13)
    "st   %a[port],  %[lo]"    "\n\t" // 2    PORT = lo     (T = 15)
    "nop"                      "\n\t" // 1    nop           (T = 16)
    "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 18)
    "rjmp head20"              "\n\t" // 2    -> head20 (next bit out)
   "nextbyte20:"               "\n\t" //                    (T = 10)
    "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 11)
    "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 13)
    "st   %a[port], %[lo]"     "\n\t" // 2    PORT = lo     (T = 15)
    "nop"                      "\n\t" // 1    nop           (T = 16)
    "sbiw %[count], 1"         "\n\t" // 2    i--           (T = 18)
     "brne head20"             "\n"   // 2    if(i != 0) -> (next byte)
    : [port]  "+e" (port),
      [byte]  "+r" (b),
      [bit]   "+r" (bit),
      [next]  "+r" (next),
      [count] "+w" (i)
    : [ptr]    "e" (ptr),
      [hi]     "r" (hi),
      [lo]     "r" (lo)
  );
  sei();
}


//                                  LCD LIBRARY
// =================================================================================

void PeanutKing_Soccer::LCDSetup (void) {
  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
  delay(5);
  
  // Now we pull both RS and R/W low to begin commands
  expanderWrite(0);  // reset expander and turn backlight off (Bit 8 =1)
  delay(1000);

  // put the LCD into 4 bit mode
  // this is according to the hitachi HD44780 datasheet
  // figure 24, pg 46

  // we start in 8bit mode, try to set 4 bit mode
  write4bits(0x03 << 4);
  delayMicroseconds(4500); // wait > 4.1ms

  // second try
  write4bits(0x03 << 4);
  delayMicroseconds(4500); // wait > 4.1ms

  // third go!
  write4bits(0x03 << 4); 
  delayMicroseconds(150);

  // finally, set to 4-bit interface
  write4bits(0x02 << 4); 

  // set # lines, font size, etc.
  send(LCD_FUNCTIONSET | LCD_displayfunction, 0);      // **********send**********
  
  // turn the display on with no cursor or blinking default
  send(LCD_DISPLAYCONTROL | LCD_displaycontrol, 0);

  // clear it off
  LCDClear();
  
  // set the entry mode
  send(LCD_ENTRYMODESET | LCD_displaymode, 0);         // **********send**********
  
  send(LCD_RETURNHOME, 0);  // set cursor position to zero
  delayMicroseconds(2000);  // this command takes a long time!
  
  LCD_backlightval=LCD_BACKLIGHT;
}


/********** high level commands, for the user! */
void PeanutKing_Soccer::LCDClear(void) {
  send(LCD_CLEARDISPLAY, 0);// clear display, set cursor position to zero
  delayMicroseconds(2000);  // this command takes a long time!
}

void PeanutKing_Soccer::setCursor(uint8_t col, uint8_t row) {
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if ( row > LCD_lineNum ) {
    row = LCD_lineNum-1;    // we count rows starting w/0
  }
  send(LCD_SETDDRAMADDR | (col + row_offsets[row]), 0);   // **********send**********
}

size_t PeanutKing_Soccer::printNumber(unsigned long n, uint8_t base)
{
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];

  *str = '\0';
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

size_t PeanutKing_Soccer::print(long n, int base)
{
  if (base == 0) {
    return write(n);
  } else if (base == 10) {
    if (n < 0) {
      int t = print('-');
      n = -n;
      return printNumber(n, 10) + t;
    }
    return printNumber(n, 10);
  } else {
    return printNumber(n, base);
  }
}

size_t PeanutKing_Soccer::print(const char str[]) {
  size_t n = 0;
  size_t l = strlen(str);
  while (1) {
    if (n==l)   break;
    if (write(*str++)) n++;
    else break;
  }
  return n;
}
//send(c, Rs)

inline size_t PeanutKing_Soccer::write(uint8_t value) {
  send(value, Rs);
  return 1;
}

//command(uint8_t value)   send(value, 0);
/************ low level data pushing commands **********/

// write either command or data
void PeanutKing_Soccer::send(uint8_t value, uint8_t mode) {
  uint8_t highnib =  value    & 0xf0;     // HHHH0000
  uint8_t lownib  = (value<<4)& 0xf0;     // LLLL0000
  
  write4bits((highnib)|mode);
  write4bits((lownib)|mode);
}

void PeanutKing_Soccer::write4bits(uint8_t value) {
  volatile bool temp = autoScanEnabled;
  autoScanEnabled = false;
  Wire.beginTransmission(LCD_Addr);
  
  Wire.write((int)(value) | LCD_backlightval);
  Wire.write((int)(value | En) | LCD_backlightval);    //pulseEnable
  delayMicroseconds(1);        // enable pulse must be >450ns
  Wire.write((int)(value & ~En) | LCD_backlightval);
  
  Wire.endTransmission();
  autoScanEnabled = temp;
  
  delayMicroseconds(40);       // commands need > 37us to settle
}

void PeanutKing_Soccer::expanderWrite(uint8_t _data) {
  Wire.beginTransmission(LCD_Addr);
  Wire.write((int)(_data) | LCD_backlightval);
  Wire.endTransmission();
}


