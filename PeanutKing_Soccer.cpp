
#include "PeanutKing_Soccer.h"

PeanutKing_Soccer* singlePeanutKing_Soccer = NULL;

PeanutKing_Soccer::PeanutKing_Soccer(uint8_t a) {
  if (singlePeanutKing_Soccer == NULL)  {
    singlePeanutKing_Soccer = this;
  }
}

ISR (TIMER1_COMPA_vect) {
  if (singlePeanutKing_Soccer != NULL ) {
    singlePeanutKing_Soccer->autoScanning();
  }
}

uint16_t PeanutKing_Soccer::sort(uint16_t a[], uint8_t size) {
  for(uint8_t i=0; i<(size-1); i++) {
    for(uint8_t o=0; o<(size-(i+1)); o++) {
      if(a[o] > a[o+1]) {
        uint16_t t = a[o];
        a[o] = a[o+1];
        a[o+1] = t;
      }
    }
  }
  return a[(size-1)/2];
}

hsv PeanutKing_Soccer::rgb2hsv(rgb in) {
  hsv         out;
  double      min, max, delta;

  min = in.r < in.g ? in.r : in.g;
  min = min  < in.b ? min  : in.b;

  max = in.r > in.g ? in.r : in.g;
  max = max  > in.b ? max  : in.b;

  out.v = max;                              // v
  delta = max - min;
  if ( delta < 0.00001 ) { // grey color
    out.s = 0;
    out.h = 0;     // undefined, maybe nan?
    return out;
  }
  if ( max == 0.0 ) { // if max is 0, then r = g = b = 0
    out.s = 0.0;                            // s = 0
    out.h = NAN;                            // h is now undefined
    return out;
  }
// NOTE: if Max is == 0, this divide would cause a crash
  out.s = (delta / max);                    // s

  if ( in.r >= max )                        // > is bogus, just keeps compilor happy
    out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
  else
  if ( in.g >= max )
    out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
  else
    out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

  out.h *= 60.0;                            // degrees

  if ( out.h < 0.0 )
    out.h += 360.0;

  return out;
}


// initialize all IOs, Serial.begin, I2C, timer interrupt, 
// External interrupt different settings depends on version number 
void PeanutKing_Soccer::init() {
  Wire.begin();
  Serial.begin(9600);
  Serial1.begin(9600);
  
  for (uint8_t i=0; i<4; i++) {
    pinMode(trigPin[i], OUTPUT);
    pinMode(echoPin[i], INPUT);
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
  
  //LCDSetup();
  
  ledSetup(0, ledPin, numLEDs);
  ledSetup(1, tcsblPin, 1);
  //ledSetPixels(1, 0, 0, 0, 0, 0);
  ledSetPixels(1, 0, 255, 255, 255, 0);
  ledUpdate(1);
  
  delay(100);
  
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
  static uint8_t cnt = 0;
  static bool wasWhite[4] = {false};
  
  systemTime = (millis()/10) %100;
  
  buttons();
  if ( buttonPressed[2] ) {
    motorEnabled = !motorEnabled;
    if ( !motorEnabled ) {
      for (int j=0; j<4; j++) {
        motorSet( j, 0 );
      }
    }
  }
  
  if ( !autoScanEnabled ) return;
  
  switch (cnt) {
    case 1:
      if ( autoScanSensors & COMPASS ) {
        sei();    //allow interrupts
        Compass = rawCompass();
        cli();    //disable interrupts
      }
      if ( autoScanSensors & COMPOUNDEYE )
        compoundEyes();
      
      if ( autoScanSensors & COLORSENSOR ) {
        GroundColor[back] = colorSenseRead(back);
        /*
        onBound[j] = isWhite[j] && ( Xsonic[j]>25 && Xsonic[j]<34 );
        outBound[j] = (!isWhite[j] && Xsonic[j]<31) && ( !outBound[j] || wasWhite[j] );
        wasWhite[j] = isWhite[j];
        */
      }
    break;
    case 2:
      if ( autoScanSensors & COLORSENSOR ) {
        GroundColor[left] = colorSenseRead(left);
        //GroundColor[right] = colorSenseRead(right);
      }
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
        ledUpdate();
      }
    break;
    case 3:
      if ( autoScanSensors & ULTRASONIC )
        Xsonic[left] = rawUltrasonic(left);
    break;
    case 4:
      if ( autoScanSensors & ULTRASONIC )
        Xsonic[right] = rawUltrasonic(right);
    break;
    case 5:
      if ( autoScanSensors & ULTRASONIC )
        Xsonic[back] = rawUltrasonic(back);
      cnt = 0;
    break;
    case 6:
      if ( autoScanSensors & ULTRASONIC )
        Xsonic[front] = rawUltrasonic(front);
    break;
  }
  
  /*
  for(uint8_t i=0; i<4; i++) {
    uint8_t c = (systemTime/200) ? 127 : 0;
    if ( Xsonic[i] )
    switch (i) {
      case front:
        ledAddPixels(1<<0, 0, 0, 0, c);
      break;
      case left:
        ledAddPixels(1<<5, 0, 0, 0, c);
      break;
      case right:
        ledAddPixels(1<<1, 0, 0, 0, c);
      break;
      case back:
        ledAddPixels(1<<3, 0, 0, 0, c);
    }
  }
  */
  cnt ++;
}


void PeanutKing_Soccer::strategy() {
  int16_t angularDirection = 0;
  int16_t x, y;
  static int16_t stuckTime;
  static bool stuckFlag;
  
  if ( !autoScanEnabled ) {
    autoScanEnabled = true;
    compassRead();
    compoundEyes();
    for (uint8_t i=0; i<4; i++ ) {
      colorSenseRead(i);
      xsonicRead(i);
    }
  }
  
  if ( Eye[MaxEye] > EYEBOUNDARY )
    moveSmart(eyeAngle, 160);
  else
    stop();          // Stop
  
  return;
  
  
  x = (Xsonic[left] - Xsonic[right])/2;
  y = Xsonic[back] - 15;
  
  if ( Eye[MaxEye] > EYEBOUNDARY ) {
    if ( outBound[left] )
      moveSmart(90, 80);
    else if ( outBound[right] )
      moveSmart(270, 80);
    else if ( onBound[left] ) {
      switch (MaxEye) {
        case 1:
        case 12:
          moveSmart(0, 180);          // move front
        break;
        case 11:
          moveSmart(0, 0);            // Stop
        break;
        case 10:
        case 9:
        case 8:
          moveSmart(180, 150);        // move back
        break;
        default:
          moveSmart(eyeAngle*1.5, 180);
      }
    }
    else if ( onBound[right] ) {
      switch (MaxEye) {
        case 1:
        case 2:
          moveSmart(0, 180);          // move front
        break;
        case 3:
          moveSmart(0, 0);            // Stop
        break;
        case 4:
        case 5:
        case 6:
          moveSmart(180, 150);        // move back
        break;
        default:
          moveSmart( 360- (360-eyeAngle)*1.5, 180);
      }
    }
    else if ( outBound[front] && outBound[front] )
      moveSmart(0, 0);          // Stop
    else {
      moveSmart(eyeAngle, 80);
    }
  }
  else {
    if ( x > 5 )
      moveSmart(270, 120);       // move left
    else if ( x < -5 )
      moveSmart(90, 120);        // move right
    else if ( y > 5 )
      moveSmart(180, 120);       // move back
    else if ( y < -5 )
      moveSmart(0, 100);         // move front
    else {
      moveSmart(0, 0);          // Stop
      for (uint8_t i=0; i<4; i++ )
        outBound[i] = false;
    }
  }
}

void PeanutKing_Soccer::ledTest (void) {
  uint8_t index = 0;
  
  autoScanEnabled = false;
  setScreen( 1, 0, "LED Test" );
  for (uint8_t i = 0; i<4; i++ ) {
    for (uint8_t j = 0; j<8; j++ ) {
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
      delay(250);
    }
  }
}

// motor test ------------------------------------------------------
void PeanutKing_Soccer::motorTest (void) {
  while (true) {
    for (int i=0; i<9; i++) {
      for (int j=0; j<4; j++) {
        if ( i<4 )
          motorSet( j, i==j ? 100 : 0 );
        else
          motorSet( j, (i-4)==j ? -100 : 0 );
      }
      do {
        buttons();
        delay(10);
      } while ( ! buttonPressed[2] );
    }
  }
}


// Low level functions :
bool PeanutKing_Soccer::buttonRead(uint8_t y) {
  if ( !autoScanEnabled )
    return !digitalRead(buttonPin[y]);
  
  bool temp = buttonTriggered[y];
  buttonTriggered[y] = false;
  return temp;
}

// compassRead -----------------------------------------------------
inline float PeanutKing_Soccer::compassRead(void) {
  if ( !autoScanEnabled )
    Compass = rawCompass();
  return Compass;
}

// xsonicRead ------------------------------------------------------
inline uint16_t PeanutKing_Soccer::xsonicRead(uint8_t Xsonic_no) {
  if ( !autoScanEnabled )
    Xsonic[Xsonic_no] = rawUltrasonic(Xsonic_no);
  return Xsonic[Xsonic_no];
}

// return single eye reading ---------------------------------------
inline uint16_t PeanutKing_Soccer::compoundEyeRead (uint8_t eye_no) {
  if ( !autoScanEnabled )
    Eye[eye_no] = rawCompoundEye(eye_no);
  return Eye[eye_no];
}

uint16_t PeanutKing_Soccer::RawColorSensor(uint8_t out) {
  uint16_t temp = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH, 2000);
  if ( temp > 900 ) temp = 900;
  return temp == 0 ? 900 : temp;
}

//return single color sensor reading
uint8_t PeanutKing_Soccer::colorSenseRead(uint8_t pin) {
  static const uint8_t TCSARRAYCOUNT = 1;
  const uint8_t& out = tcsRxPin[pin];
  uint16_t 
    rread[TCSARRAYCOUNT],
    gread[TCSARRAYCOUNT],
    bread[TCSARRAYCOUNT];
  
  digitalWrite(tcsSxPin[2], LOW);
  digitalWrite(tcsSxPin[3], LOW);
  delayMicroseconds(150);
  for (uint8_t i=0; i<TCSARRAYCOUNT; i++)
    rread[i] = RawColorSensor(out);
    //rread[i] = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH, 10000);
  digitalWrite(tcsSxPin[3], HIGH);
  delayMicroseconds(150);
  for (uint8_t i=0; i<TCSARRAYCOUNT; i++)
    bread[i] = RawColorSensor(out);
    //bread[i] = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH, 10000);
  digitalWrite(tcsSxPin[2], HIGH);
  delayMicroseconds(150);
  for (uint8_t i=0; i<TCSARRAYCOUNT; i++)
    gread[i] = RawColorSensor(out);
    //gread[i] = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH, 10000);

  /*
  for (uint8_t i=0; i<TCSARRAYCOUNT; i++) {
    Serial.print(rread[i]);  Serial.print("  ");
  }
  Serial.print(" | ");
  for (uint8_t i=0; i<TCSARRAYCOUNT; i++) {
    Serial.print(gread[i]);  Serial.print("  ");
  }
  Serial.print(" | ");
  for (uint8_t i=0; i<TCSARRAYCOUNT; i++) {
    Serial.print(bread[i]);  Serial.print("  ");
  }
  Serial.println("  ");
  */
  rgbData[pin].r = sort(rread, TCSARRAYCOUNT);
  rgbData[pin].b = sort(bread, TCSARRAYCOUNT);
  rgbData[pin].g = sort(gread, TCSARRAYCOUNT);
  //rgbData[pin].b *= 1.15;
  
  rgb rgbRaw;
  /*
  rgbRaw.r = map(rgbData[pin].r, 900, 75, 0, 100)/100.0;
  rgbRaw.g = map(rgbData[pin].g, 900, 75, 0, 100)/100.0;
  rgbRaw.b = map(rgbData[pin].b, 900, 75, 0, 100)/100.0;
  */
  rgbRaw.r = map(rgbData[pin].r, 128, 16, 0, 100)/100.0;
  rgbRaw.g = map(rgbData[pin].g, 128, 16, 0, 100)/100.0;
  rgbRaw.b = map(rgbData[pin].b, 128, 16, 0, 100)/100.0;
  
  rgbRaw.r = constrain(rgbRaw.r,  0, 1);
  rgbRaw.g = constrain(rgbRaw.g,  0, 1);
  rgbRaw.b = constrain(rgbRaw.b,  0, 1);
  
  hsv& op = hsvData[pin];
  op = rgb2hsv(rgbRaw);
  
  isWhite[pin] = ( op.s < 0.1 && op.v > 0.85 );
  
  // COLOR decision making
  if ( op.v < 0.1 )                     return black;
  else if ( op.s < 0.1 && op.v > 0.8 )  return white;
  else if ( op.h <  50 || op.h > 315 )  return red;
  else if ( op.h < 100 )                return yellow;
  else if ( op.h < 175 )                return green;
  else if ( op.h < 250 )                return blue;
  else                                  return magenta;
}

// simple motor turn, motor_no cannot add, one by one 
void PeanutKing_Soccer::motorSet(uint8_t motor_no, int16_t speed) {
  //static int16_t previousSpeed[4] = {0,0,0,0};
  if ( motorEnabled == false ) speed = 0;
  if      ( speed>0 && speed<255 ) {
    digitalWrite(dirPin[motor_no], LOW);
    digitalWrite(dir2Pin[motor_no], HIGH);
    analogWrite(pwmPin[motor_no], speed);
    //digitalWrite(diagPin[motor_no], HIGH);
  }
  else if ( speed<0 && speed>-255 ) {
    digitalWrite(dirPin[motor_no], HIGH);
    digitalWrite(dir2Pin[motor_no], LOW);
    analogWrite(pwmPin[motor_no], -speed);
    //digitalWrite(diagPin[motor_no], HIGH);
  }
  else{
    //digitalWrite(dirPin[motor_no], motorBrakeEnabled ?  HIGH : LOW);
    digitalWrite(dirPin[motor_no], HIGH);
    digitalWrite(dir2Pin[motor_no], HIGH);
    digitalWrite(pwmPin[motor_no], LOW);
    //digitalWrite(diagPin[motor_no], LOW);
  }
  //previousSpeed[motor_no] = speed;
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
  
  motorControl(angular_direction, speed, rotation*0.8);
}


inline void PeanutKing_Soccer::stop(void) {
  for(uint8_t i=0; i<4; i++) {
    motorSet(i, 0);
  }
}

// turn on/off specific sensory system scannig function
// CompoundEye + Compass + Ultrasonic + ColorSense
void PeanutKing_Soccer::enableScanning(bool enabled, uint8_t sensorType) {
  if ( true ) {
  }
}

void PeanutKing_Soccer::ledClear(void) {
  //static uint32_t lastTime = millis();
  for (uint8_t i=0; i<8; i++) {
    ledSetPixels(0, i, 0, 0, 0, 0);
  }
}
// not using adaFruit, we write our own library, will check if same \
// color, Which_led use the same methodology as module, 
// just like: LED_1 + LED_2.... if multiple color at same time}
void PeanutKing_Soccer::ledShow(uint8_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  //static uint32_t lastTime = millis();
  for (uint8_t i=0; i<8; i++) {
    if ( n & (1<<i) ) {
      ledSetPixels(0, i, r, g, b, w);
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

//r = (r * brightness) >> 8;
// Offset:    W          R          G          B
// NEO_GRB  ((1 << 6) | (1 << 4) | (0 << 2) | (2))   GRBW 
// Set pixel color from separate R,G,B components:
void PeanutKing_Soccer::ledSetPixels(uint8_t x, uint8_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  uint8_t *p = &leds[x].pixels[n * 4];  // 4 bytes per pixel
  if(n < numLEDs) {
    p[1] = r;                   // R
    p[0] = g;                   // G
    p[2] = b;                   // B
    p[3] = w;                   // W
  }
}

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

inline void PeanutKing_Soccer::ledUpdate(void) {
  ledUpdate(0);
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


// col(0-15), row(0-1) --------------------------------------------
void PeanutKing_Soccer::setScreen(uint8_t col, uint8_t row, char string[]) {
  lcd.backlight();
  lcd.setCursor(col, row);
  lcd.print("Hello");
  /*
  setCursor(col, row);
  print(string);*/
}

void PeanutKing_Soccer::bluetoothSend(char) {
// send char
 // Serial1.print();
}

void PeanutKing_Soccer::printSpace(uint32_t data, uint8_t digit) {
  for ( int i=1; i<digit; i++ ) {
    if ( data < pow(10, i) )   Serial.print(" ");
  }
  Serial.print(data);
}

void PeanutKing_Soccer::debugging(uint16_t updateRate, uint8_t sensorType) {
  static uint32_t lastTimeIn = 0;

  for (int i=0; i<3; i++) {
    if ( buttonPressed[i] ) {
      Serial.print("Button ");
      Serial.print(i);
      Serial.println("pressed. ");
    }
  }
  if ( millis() - lastTimeIn < updateRate ) {
    delay(5);
    return;
  }
  lastTimeIn = millis();
  
  
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
        Serial.print(rgbData[i].r, 2);
        Serial.print(", ");
        Serial.print(rgbData[i].g, 2);
        Serial.print(", ");
        Serial.print(rgbData[i].b, 2);
        Serial.print("  |  hsv ");
        printSpace(hsvData[i].h, 3 );
        Serial.print(", ");
        Serial.print(hsvData[i].s, 2);
        Serial.print(", ");
        Serial.print(hsvData[i].v, 2);
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

uint16_t PeanutKing_Soccer::setHome(void) { 
// software set compass home
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

// rawUltrasonic --------------------------------------------------
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

// return single eye reading  --------------------------------------
inline uint16_t PeanutKing_Soccer::rawCompoundEye (uint8_t eye_no) {
  return 1023 - analogRead(irPin[eye_no-1]);
}

void PeanutKing_Soccer::rawColor(uint8_t pin, uint16_t &rData, uint16_t &gData, uint16_t &bData) {
  const uint8_t& out = tcsRxPin[pin];
  
  digitalWrite(tcsSxPin[2], LOW);
  digitalWrite(tcsSxPin[3], LOW);
  delayMicroseconds(100);
  rData = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH, 2000);
  digitalWrite(tcsSxPin[3], HIGH);
  delayMicroseconds(100);
  bData = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH, 2000);
  digitalWrite(tcsSxPin[2], HIGH);
  delayMicroseconds(100);
  gData = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH, 2000);
}

// Hidden functions ------------------------------------------------------

void PeanutKing_Soccer::buttons(void) {
  static bool lastButton[3] = {false};

  for (uint8_t i=0; i<3; i++) {
    Button[i] = !digitalRead(buttonPin[i]);
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

// ===============================================================================


void PeanutKing_Soccer::LCDSetup (void) {
//  Wire.begin();

  _backlightval = LCD_NOBACKLIGHT;
  _displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
  
  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
  delay(5); 
  
  // Now we pull both RS and R/W low to begin commands
  expanderWrite(_backlightval);  // reset expanderand turn backlight off (Bit 8 =1)
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
  send(LCD_FUNCTIONSET | _displayfunction, 0);      // **********send**********
  
  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;

  _displaycontrol |= LCD_DISPLAYON;
  send(LCD_DISPLAYCONTROL | _displaycontrol, 0);

  // clear it off
  clear();
  
  // Initialize to default text direction (for roman languages)
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  
  // set the entry mode
  send(LCD_ENTRYMODESET | _displaymode, 0);         // **********send**********
  
  send(LCD_RETURNHOME, 0);  // set cursor position to zero
  delayMicroseconds(2000);  // this command takes a long time!
}


/********** high level commands, for the user! */
void PeanutKing_Soccer::clear(void) {
  send(LCD_CLEARDISPLAY, 0);// clear display, set cursor position to zero
  delayMicroseconds(2000);  // this command takes a long time!
}

void PeanutKing_Soccer::setCursor(uint8_t col, uint8_t row) {
// Turn the (optional) backlight off/on
  //_backlightval=LCD_NOBACKLIGHT;
  _backlightval=LCD_BACKLIGHT;
  expanderWrite(0);
  
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if ( row > _numlines ) {
    row = _numlines-1;    // we count rows starting w/0
  }
  send(LCD_SETDDRAMADDR | (col + row_offsets[row]), 0);   // **********send**********
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
  expanderWrite(value);
  
  expanderWrite(value | En);  // En high
  delayMicroseconds(1);    // enable pulse must be >450ns
  
  expanderWrite(value & ~En);  // En low
  delayMicroseconds(50);    // commands need > 37us to settle
}

void PeanutKing_Soccer::expanderWrite(uint8_t _data) {                                        
  Wire.beginTransmission(_Addr);
  Wire.write((int)(_data) | _backlightval);
  Wire.endTransmission();
}



