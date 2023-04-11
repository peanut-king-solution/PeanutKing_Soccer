/*
 * Copyright (c) 2021 PeanutKing Solution
 *
 * @file        PeanutKingSoccerV2.c
 * @summary     Soccer Robot Library using timer interrupt
 * @version     2.0
 * @author      Jack Kwok
 * @date        11 May 2021
 */

#include "PeanutKingSoccerV2.h"

#define PIN_READ_COLOR (PINA)
#define readColorPin0 ((PINA >> 6) & 1)
#define readColorPin1 ((PINA >> 5) & 1)
#define readColorPin2 ((PINA >> 7) & 1)
#define readColorPin3 ((PINA >> 4) & 1)

#define colorReadCycle 160         // 8us/cyce

static PeanutKingSoccerV2* V2bot = NULL;

volatile uint8_t pinRead;

volatile uint32_t xsTicks = 0;    // holds the pulse count
volatile uint32_t clTicks = 0;    // holds the pulse count
volatile uint32_t TIM1CNT = 0;  // holds the pulse count
volatile uint32_t Timer2Ticks = 0;  // holds the pulse count

volatile uint32_t XSi = 0;       // ultrasonic index
volatile uint32_t CLi = 0;       // ultrasonic index
uint8_t   CLc = 0;

volatile uint32_t adcCnt = 0;

pulsein_t XS[4] = {0};
pulseinColor_t clr[4] = {0};

const readpin_t xsPin[4] = {
  {&PING, 0},
  {&PING, 2},
  {&PIND, 7},
  {&PING, 1},
};

const readpin_t xsTrigPin[4] = {
  {&PORTC, 0},
  {&PORTC, 2},
  {&PORTC, 3},
  {&PORTC, 1},
};

const readpin_t clPin[4] = {
  {&PINA, 6},
  {&PINA, 5},
  {&PINA, 7},
  {&PINA, 4},
};


ISR(ADC_vect) {     //when new ADC value ready
  V2bot->eye[adcCnt] = 1023 - (ADCL>>6| ADCH<<2);   //update the new value from A0 (between 0 and 255)
  ++adcCnt;
  if (adcCnt==12) adcCnt = 0;
  ADMUX = (1 << REFS0) | (1 << ADLAR) | (adcCnt&0x07);
  
  // the MUX5 bit of ADCSRB selects whether we're reading from channels
  // 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((adcCnt >> 3) & 0x01) << MUX5);
  // ADCSRA |= (1 << ADSC);    //start ADC measurements
}

// for (int i=0; i<4; i++) {
//   pinRead = ((*clPin[i].port) >> clPin[i].bit) & 1;
//   if ( pinRead != clr[i].pSt) {
//     clr[i].t++;
//   }
//   clr[i].pSt = pinRead;
// }

// volatile uint8_t pColorPin = 0;
// n ^= 1 << k    // toggle a bit
// pinRead & pColorPin

ISR(TIMER1_COMPA_vect) {
  pinRead = PIN_READ_COLOR;
  if ( ((pinRead >> 6) & 1) != clr[0].pSt) {
    // if (clr[0].nxt != 0) {
    //   clr[0].nxt == 0;
    // }
    // else 
      // clr[0].dur[CLc] = (TIM1CNT - clr[0].t + clr[0].dur[CLc]) / 2;
    clr[0].dur[CLc] = TIM1CNT - clr[0].t;
    clr[0].t = TIM1CNT;
    clr[0].pSt = 1-clr[0].pSt;
    // clr[0].f++;
  } else
  if ( ((pinRead >> 5) & 1) != clr[1].pSt) {
    // if (clr[1].nxt != 0) {
    //   clr[1].nxt == 0;
    // }
    // else
      clr[1].dur[CLc] = TIM1CNT - clr[1].t;
      // clr[1].dur[CLc] = (TIM1CNT - clr[1].t + clr[1].dur[CLc]) / 2;
    clr[1].pSt = 1-clr[1].pSt;
    clr[1].t = TIM1CNT;
  } else
  if ( ((pinRead >> 7) & 1) != clr[2].pSt) {
    // if (clr[2].nxt != 0)
    //   clr[2].nxt == 0;
    // else 
      clr[2].dur[CLc] = TIM1CNT - clr[2].t;
      // clr[2].dur[CLc] = (TIM1CNT - clr[2].t + clr[2].dur[CLc]) / 2;
    clr[2].pSt = 1-clr[2].pSt;
    clr[2].t = TIM1CNT;
  } else
  if ( ((pinRead >> 4) & 1) != clr[3].pSt) {
    // if (clr[3].nxt != 0)
    //   clr[3].nxt == 0;
    // else 
      clr[3].dur[CLc] = TIM1CNT - clr[3].t;
      // clr[3].dur[CLc] = (TIM1CNT - clr[3].t + clr[3].dur[CLc]) / 2;
    clr[3].pSt = 1-clr[3].pSt;
    clr[3].t = TIM1CNT;
  }

  TIM1CNT++;
  // // clr[0].pinRead = (PINA >> 6) & 1;
  // if ( readColorPin0 == clr[0].pulse) {
  //   clr[0].t++;
  // }
  // else if (clr[0].t != 0) {
  //   clr[0].dur[CLc] = clr[0].t;
  //   clr[0].t = 0;
  // }
  // // clr[1].pinRead = (PINA >> 5) & 1;
  // if ( readColorPin1 == clr[1].pulse) {
  //   clr[1].t++;
  // }
  // else if (clr[1].t != 0) {
  //   clr[1].dur[CLc] = clr[1].t;
  //   clr[1].t = 0;
  // }
}


// ISR(TIMER1_COMPB_vect) {
//   // clr[2].pinRead = (PINA >> 7) & 1;
//   if ( readColorPin2 == clr[2].pulse) {
//     clr[2].t++;
//   }
//   else if (clr[2].t != 0) {
//     clr[2].dur[CLc] = clr[2].t;
//     clr[2].t = 0;
//   }
//   // clr[3].pinRead = (PINA >> 4) & 1;
//   if ( readColorPin3 == clr[3].pulse) {
//     clr[3].t++;
//   }
//   else if (clr[3].t != 0) {
//     clr[3].dur[CLc] = clr[3].t;
//     clr[3].t = 0;
//   }
// }

uint8_t CLbit = 0;

// PORTC |= 1 << 0;        // HIGH
// PORTC |= 1 << 2;        // HIGH
// PORTC |= 1 << 3;        // HIGH
// PORTC |= 1 << 1;        // HIGH
// _delay_us(1);
// PORTC &= ~(1 << 0);     // LOW
// PORTC &= ~(1 << 2);     // LOW
// PORTC &= ~(1 << 3);     // LOW
// PORTC &= ~(1 << 1);     // LOW
  // XS[0].pinRead = (PING >> 0) & 1;
  // XS[1].pinRead = (PING >> 2) & 1;
  // XS[2].pinRead = (PIND >> 7) & 1;
  // XS[3].pinRead = (PING >> 1) & 1;
  
  // clr[0].pinRead = (PINA >> 6) & 1;
  // clr[1].pinRead = (PINA >> 5) & 1;
  // clr[2].pinRead = (PINA >> 7) & 1;
  // clr[3].pinRead = (PINA >> 4) & 1;

/* 0.1715mm / us
 * 58.3us / cm
 * 932.9 cycle per it for 1 tick = 1 cm
 */
ISR(TIMER2_COMPA_vect) {
  pinRead = (*xsPin[XSi].port >> xsPin[XSi].bit) & 1;
  if ( pinRead != XS[XSi].pSt ) {
    XS[XSi].pSt = pinRead;
    if ( pinRead == 1 ) {
      XS[XSi].t = xsTicks;
    }
    else {
      // XS[XSi].dur = xsTicks-XS[XSi].t;
      V2bot->ultrasonic[XSi] = xsTicks-XS[XSi].t;
      XSi++;    if (XSi==4)   XSi = 0;

      *xsTrigPin[XSi].port |= 1 << xsTrigPin[XSi].bit;        // HIGH
      _delay_us(1);
      *xsTrigPin[XSi].port &= ~(1 << xsTrigPin[XSi].bit);     // LOW
    }
  }
  xsTicks++;

  if (xsTicks - clTicks > colorReadCycle) {
    for (int i=0; i<4; i++) {
      // clr[i].pSt = 1 - ((*clPin[i].port) >> clPin[i].bit) & 1;
      // clr[i].pulse = 1 - ((*clPin[i].port) >> clPin[i].bit) & 1;
      // clr[i].nxt = 1;
      // clr[i].dur[CLc] = clr[i].t;
      clr[i].fx[CLc] = clr[i].f;
      clr[i].f = 0;
    }
    clTicks = xsTicks;
    CLc = (CLc==2) ? 0 : CLc+1;
    switch (CLc) {
      case 0:     // red
        PORTL &= ~(1 << 4);     // LOW s2
        PORTL &= ~(1 << 3);     // LOW s3
      break;
      case 1:     // blue
        PORTL |= 1 << 3;        // HIGH
      break;
      case 2:     // green
        PORTL |= 1 << 4;        // HIGH
      break;
    }
    V2bot->buttonRead(0);
  }
}


// Constant  =================================================================




PeanutKingSoccerV2::PeanutKingSoccerV2(void) :
  tcsblPin(32),
  ledPin  (33),
  actledPin(30),
  buttonPin{42, 47, 48},

  // inhPin  { 4,  7, 10, 13},
  // in1Pin  { 2,  5,  8, 11},
  // in2Pin  { 3,  6,  9, 12},

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

void PeanutKingSoccerV2::init(void) {
  Serial.begin(115200);
  Serial1.begin(9600);
  
  compassHandle = gIIC->RegisterDevice(compass_address, 1, IICIT::Speed::SLOW);
  lcdHandle     = gIIC->RegisterDevice(LCD_Addr, 1, IICIT::Speed::SLOW);
  
  for (uint8_t i=0; i<4; i++) {
    pinMode(trigPin[i], OUTPUT);
    pinMode(echoPin[i], INPUT);
    pinMode(tcsSxPin[i], OUTPUT);
    pinMode(tcsRxPin[i], INPUT);
    pinMode(pwmPin[i],  OUTPUT);
    pinMode(dirPin[i],  OUTPUT);
    pinMode(dir2Pin[i], OUTPUT);
    pinMode(diagPin[i], OUTPUT);
    digitalWrite(diagPin[i], HIGH);
    analogWrite(pwmPin[i], 0);
  }
// S0 S1 OUTPUT FREQUENCY SCALING(f0)
// L L Power down
// L H 2%
// H L 20%
// H H 100%     fastest
  digitalWrite(tcsSxPin[0], HIGH);
  digitalWrite(tcsSxPin[1], LOW);
  // digitalWrite(tcsSxPin[1], HIGH);
  PORTL &= ~(1 << 4);     // LOW s2
  PORTL &= ~(1 << 3);     // LOW s3


  for (uint8_t i=0; i<12; i++)
    pinMode(irPin[i], INPUT);

  for (uint8_t i=0; i<3; i++)
    pinMode(buttonPin[i], INPUT);
  
  pinMode(actledPin, OUTPUT);
  digitalWrite(actledPin, HIGH);
  
  lcdSetup();
  
  ledSetup(0, ledPin, NUMPIXELS);
  // if ( !ledEnabled ) {
  ledShow(255, 0, 0, 0, 0);
  ledUpdate();
  // }
  delay(1);
  
  ledSetup(1, tcsblPin, 1);
  ledShow(1, 255, 255, 255, 255, 1);
  // ledShow(1, 0, 0, 0, 0, 1);
  ledUpdate(1);
  
  delay(1);

  cli();    //disable interrupts

  // uint8_t timer = digitalPinToTimer(pin);
  // TIMER4A, // PH 3 ** 6 ** PWM6  
  // TIMER4B, // PH 4 ** 7 ** PWM7  
  // TIMER4C, // PH 5 ** 8 ** PWM8  
  // TIMER2B, // PH 6 ** 9 ** PWM9  
  // TIMER2A, // PB 4 ** 10 ** PWM10  
  // TIMER1A, // PB 5 ** 11 ** PWM11  
  // TIMER1B, // PB 6 ** 12 ** PWM12  
  // TIMER0A, // PB 7 ** 13 ** PWM13  
  // TCCR4A &= ~_BV(COM4A1);
  // TCCR4A &= ~_BV(COM4B1);

//   Timer 1
  TCCR1A  = 0x00;           // Normal mode, just as a Timer
  TCNT1   = 0;
  OCR1A   = 2;             // 8 * 4 / 16 = 2us
  // OCR1B   = 1;            // Hz = ?
  
  TCCR1B = (1 << WGM12);    // CTC mode; Clear Timer on Compare
  // TCCR1B |= (1 << CS11); // Set CS#1 bit for 8 prescaler for timer 1
  TCCR1B |= (1 << CS10) | (1 << CS11);    // CLK i/o /64 (From Prescaler)  (must be <65536)
  // TCCR1B |= (1 << CS12);    // prescaler = 256
  // TCCR1B |= (1 << CS10) | (1 << CS12);    // prescaler = 1024
  TIMSK1 |= (1 << OCIE1A);
  // TIMSK1 |= (1 << OCIE1B);  // enable timer compare interrupt

//   Timer 2
  TCCR2A  = (1 << WGM21);   // CTC mode; Clear Timer on Compare
  OCR2A   = 6;             // 14.57    ,Hz = (16*10^6) / (OCR1A-1 * presc)
  // OCR2B   = 6;
  // TCCR2B |= (1 << CS21 | 1 << CS20);  // div 32
  // TCCR2B |= (1 << CS22);  // div 64
  TCCR2B |= (1 << CS22 | 1 << CS20);  // div 128
  // TCCR2B |= (1 << CS22 | 1 << CS21 | 1 << CS20);  // div 1024
  TIMSK2 |= (1 << OCIE2A);
  // TIMSK2 |= (1 << OCIE2B);

  // initADC
  ADMUX |= (1 << REFS0);  //set reference voltage
  ADMUX &= ~(1 << ADLAR);
  // ADMUX |= (1 << ADLAR);  //left align the ADC value- so we can read highest 8 bits from ADCH register only
  ADMUX |= _BV(MUX0);     // ADC0

  ADCSRA &= ~(1 << ADPS1);  //  bitClear(ADPS1, ADPS1);   //  101
  // ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  // sampling cycle = 13
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 128prescaler, 9.6 Sampling rate (KHz)
  ADCSRA |= (1 << ADATE);   //enabble auto trigger
  ADCSRA |= (1 << ADIE);    //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);    //enable ADC  ;; bitSet(ADCSRA, ADEN);
  ADCSRA |= (1 << ADSC);    //start ADC measurements

  // motors
  TCCR0A |= _BV(COM0B1);// | _BV(WGM01) | _BV(WGM00);
  // TCCR0A &= ~_BV(COM0A1);
//  TCCR2B = _BV(CS22);
  OCR0B = 0;
  
  TCCR3A |= _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1);// | _BV(WGM31) | _BV(WGM30); //Set Timer to varying top limit fast PWM mode
  // TCCR3B = _BV(WGM33) | _BV(WGM32)| _BV(CS31);
// | _BV(COM3A0) | _BV(COM3B0) | _BV(COM3C0)

//  TCCR3B = _BV(CS22);
  OCR3A = 0;
  OCR3B = 0;
  OCR3C = 0;

  sei();    //allow interrupts

  *xsTrigPin[XSi].port |= 1 << xsTrigPin[XSi].bit;        // HIGH
  _delay_us(1);
  *xsTrigPin[XSi].port &= ~(1 << xsTrigPin[XSi].bit);     // LOW

  delay(1);
}




uint16_t PeanutKingSoccerV2::sort(uint16_t a[], uint8_t size) {
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

hsv_t PeanutKingSoccerV2::rgb2hsv(rgb_t in) {
  hsv_t      out;
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


// return single eye reading --------------------------------------
uint16_t PeanutKingSoccerV2::compoundEyeRead (uint8_t eye_no) {
  maxEye = 0;
//  minEye = 1;
  for (int i=0; i<12; i++) {
  //debugging use
    // Serial.print("eye[");Serial.print(i);Serial.print("]: ");Serial.println(eye[i]);
    if( eye[i]>eye[maxEye] )
      maxEye = i;
//    else if( eye[i]<eye[minEye] )
//      minEye = i;
  }
  
  int16_t
    upper = (maxEye==12) ? eye[1] : eye[maxEye+1],
    lower = (maxEye==1) ? eye[12] : eye[maxEye-1],
    add = 15.0 * (upper-lower) / (eye[maxEye] - ((upper<lower) ? upper : lower) );

  eyeAngle = 30 * maxEye + add;
  eyeAngle += ( eyeAngle>=30 ) ? -30 : 330;
  if ( eye_no >0 && eye_no <= 12 )
    return eye[eye_no];
  else {

    if ( eye_no == 13 ) 
      return maxEye;
    else if ( eye_no == 14 ) 
      return eye[maxEye];
    else 
      return 0;
  }
}


// buttonRead -----------------------------------------------------
bool PeanutKingSoccerV2::buttonRead(uint8_t button_no) {
  static bool lastButton[3] = {false};
  button[0] = (PINL >> 7) & 1;
  button[1] = (PINL >> 2) & 1;
  button[2] = (PINL >> 1) & 1;
  for (uint8_t i=0; i<3; i++) {
    buttonPressed[i] = ( button[i] && !lastButton[i] );
    buttonReleased[i] = ( !button[i] && lastButton[i] );  

    if ( buttonPressed[i] )    buttonTriggered[i] = true;

    lastButton[i] = button[i];
  }
  if ( button_no == 1 || button_no == 2 ) {
    return button[button_no];
  }
  else
    return 0;
}

// ultrasonicRead -------------------------------------------------
uint16_t PeanutKingSoccerV2::ultrasonicRead(uint8_t i) {
  if ( i<0 || i>=4 ) return 999;  
  if ( ultrasonic[i]<0 )   ultrasonic[i]+=32768;
  return ultrasonic[i];
}

// return single color sensor reading
uint16_t PeanutKingSoccerV2::floorColorRead(uint8_t pin_no, uint8_t mono = 0) {
  if ( mono == red ) {
    colorRGB[pin_no].r = clr[pin_no].dur[0];
    return colorRGB[pin_no].r;
  } else
  if ( mono == green ) {
    colorRGB[pin_no].g = clr[pin_no].dur[2];
    return colorRGB[pin_no].g;
  } else
  if ( mono == blue ) {
    colorRGB[pin_no].b = clr[pin_no].dur[1];
    return colorRGB[pin_no].b;
  }

  // return clr[pin_no].dur[mono];
  //colorRGB[pin_no].b *= 1.15;
  // if ( clr[i].dur[CLc] != 0 ) {
  //   if      ( mono == red )   return colorRGB[pin_no].r;
  //   else if ( mono == green ) return colorRGB[pin_no].g;
  //   else if ( mono == blue )  return colorRGB[pin_no].b;
  // }
  // colorRGB[pin_no].b = clr[pin_no].dur[mono];

  hsv_t& op = colorHSV[pin_no];
  op = rgb2hsv(colorRGB[pin_no]);
  
  isWhite[pin_no] = ( op.s < 10 && op.v > 85 );
  
  if      ( mono == red )   return colorRGB[pin_no].r;
  else if ( mono == green ) return colorRGB[pin_no].g;
  else if ( mono == blue )  return colorRGB[pin_no].b;
  else return 0;

  // COLOR decision making
  if ( op.v < 30 )                     return black;
  else if ( op.s < 10 && op.v > 150 )  return white;
  else if ( op.h < 50 || op.h > 315 )  return red;
  else if ( op.h < 100 )               return yellow;
  else if ( op.h < 175 )               return green;
  else if ( op.h < 250 )               return blue;
  else                                 return magenta;
}

uint16_t PeanutKingSoccerV2::whiteLineCal(uint8_t pin_no, uint16_t calVal) {
  whiteLineThreshold = calVal;
  for (int j=0; j<3; j++) {
    floorColorRead(pin_no, j+3);
  }
  return (colorRGB[pin_no].r + colorRGB[pin_no].g + colorRGB[pin_no].b);
}

bool PeanutKingSoccerV2::whiteLineCheck(uint8_t pin_no) {
  for (int j=0; j<3; j++) {
    floorColorRead(pin_no, j+3);
  }
  return ((colorRGB[pin_no].r + colorRGB[pin_no].g + colorRGB[pin_no].b) < whiteLineThreshold);
}

// compassRead ----------------------------------------------------
uint16_t PeanutKingSoccerV2::compassRead(uint8_t addr, uint8_t cmd, uint8_t len) {
  uint8_t rxbuff[3] = {0};
  uint16_t temp=0;
  float answer = 888;
  uint8_t _status;
  uint8_t msg[1] = {cmd};

  _status = gIIC->Write(compassHandle, msg, 1);
  _status = gIIC->Read(compassHandle, rxbuff, 3);

  temp  = rxbuff[1] | (rxbuff[2] << 8);
  answer = temp/100.0;

  return answer;
}

/* =============================================================================
 *                                  Motors
 * ============================================================================= */

// simple single motor turn
void PeanutKingSoccerV2::motorSet(uint8_t pin, int16_t speed) {
  if ( !motorEnabled ) speed = 0;
  speed *= motorScale[pin];
  // cli();
  if      ( speed>0 && speed<256 ) {
    *motor[pin].port |=   1 << motor[pin].bitb;      // HIGH
    *motor[pin].port &= ~(1 << motor[pin].bita);
    *(uint16_t*)(motor[pin].pwm) = speed;
  }
  else if ( speed<0 && speed>-256 ) {
    *motor[pin].port |=   1 << motor[pin].bita;      // HIGH
    *motor[pin].port &= ~(1 << motor[pin].bitb);
    *(uint16_t*)(motor[pin].pwm) = -speed;
  }
  else{
    *(uint16_t*)(motor[pin].pwm) = 0;
    *motor[pin].port |=   1 << motor[pin].bita;      // HIGH
    *motor[pin].port |=   1 << motor[pin].bitb;      // HIGH
  }
  // sei();
}

void PeanutKingSoccerV2::motorControl(float mAngle, float mSpeed, float rotate) {
  int16_t mc[4];
  mc[0] = mSpeed*sin( (mAngle+45.0)*pi/180.0 );
  mc[1] = mSpeed*cos( (mAngle+45.0)*pi/180.0 );
  mc[2] = -mc[0];
  mc[3] = -mc[1];
  for(int8_t i=3; i>=0; i--) {
    motorSet(i, mc[i] + rotate);
  }
}

// motor move + compass as reference
void PeanutKingSoccerV2::moveSmart(uint16_t angular_direction, int16_t speed, int16_t angle, uint8_t precision) {
  if (angle >= 360) angle = 0;

  int16_t c = compassRead();
  c -= angle;
  if (c<0) c+=360;
  int16_t rotation = c < 180 ? -c : 360 - c;
  
  // Serial.print("angle = "); Serial.println(angle);
  // Serial.print("c = "); Serial.println(c);
  // Serial.print("rotation = "); Serial.println(rotation);

  //speed - 50
  //rotation = abs(speed) < 120 ? rotation : rotation * 1.5;
  rotation = rotation * (precision+3)/12;

  motorControl(angular_direction, speed, rotation);
}


// Bluetooth ------------------------------------------------------
void PeanutKingSoccerV2::bluetoothSend(char string[]) {
// send char
  Serial1.write(string, sizeof(string));
}

void PeanutKingSoccerV2::bluetoothReceive(void) {
// send char
  btRxBuffer[0] = Serial1.read();
}


void PeanutKingSoccerV2::bluetoothAttributes(void) {
  static btData_t btDataHeader = Idle;
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

//                                  
// ================================================================================


// col(0-15), row(0-1) --------------------------------------------
void PeanutKingSoccerV2::setScreen(uint8_t col, uint8_t row, char string[]) {
  setCursor(col, row);
  print(string);
}

void PeanutKingSoccerV2::setScreen(uint8_t col, uint8_t row, int16_t numbers, uint8_t digits) {
  setCursor(col, row);
  if (numbers>=0) {
    for ( int i=1; i<digits; i++ ) {
      if ( numbers < pow(10, i) )   print(" ");
    }
  }
  print(numbers);
  print(" ");
}

void PeanutKingSoccerV2::printSpace(uint32_t data, uint8_t digit) {
  for ( int i=1; i<digit; i++ ) {
    if ( data < pow(10, i) )   Serial.print(" ");
  }
  Serial.print(data);
}


//                                  LEDs
// =============================================================================
void PeanutKingSoccerV2::ledSetup (uint8_t x, uint8_t p, uint8_t n) {
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
void PeanutKingSoccerV2::ledShow(uint8_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t x) {
  //static uint32_t lastTime = millis();
  for (uint8_t i=0; i<8; i++) {
    if ( n & (1<<i) ) {
      uint8_t *p = &leds[x].pixels[i * 4];  // 4 bytes per pixel
      p[1] = r;                   // R
      p[0] = g;                   // G
      p[2] = b;                   // B
      p[3] = w;                   // W
    }
  }
}

void PeanutKingSoccerV2::ledSetPixels(uint8_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
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

void PeanutKingSoccerV2::ledClear(void) {
  ledShow(255, 0, 0, 0, 0);
  ledUpdate();
}

void PeanutKingSoccerV2::ledUpdate(uint8_t x) {
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
// ================================================================

void PeanutKingSoccerV2::lcdSetup (void) {
  // According to datasheet PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // we need at least 40ms after power rises above 2.7V before sending commands.
  // Arduino can turn on way befer 4.5V so we'll wait 50
  delay(5);
  
  // Now we pull both RS and R/W low to begin commands
  uint8_t msg[1] = {0 | LCD_backlightval};// reset expander and turn backlight off (Bit 8 =1)
  uint8_t _status = gIIC->Write(lcdHandle, msg, 1);
  delay(1000);

  // put the LCD into 4 bit mode, according to the hitachi HD44780 datasheet figure 24, pg 46
  write4bits(0x03 << 4);  delayMicroseconds(4500);  // Start in 8bit mode, try 4 bit mode, wait > 4.1ms
  write4bits(0x03 << 4);  delayMicroseconds(4500);  // second try, wait > 4.1ms
  write4bits(0x03 << 4);  delayMicroseconds(150);   // third go!
  write4bits(0x02 << 4);                            // finally, set to 4-bit interface

  // set # lines, font size, etc.
  send(LCD_FUNCTIONSET | LCD_displayfunction, 0);   // turn the display on with no cursor or blinking default
  send(LCD_DISPLAYCONTROL | LCD_displaycontrol, 0);

  lcdClear();               // clear it off

  send(LCD_ENTRYMODESET | LCD_displaymode, 0);      // set the entry mode
  send(LCD_RETURNHOME, 0);    delay(2);             // set cursor position to zero, takes a long time!
  
  LCD_backlightval=LCD_BACKLIGHT;
}


/********** high level commands, for the user! */
void PeanutKingSoccerV2::lcdClear(void) {
  send(LCD_CLEARDISPLAY, 0);// clear display, set cursor position to zero
  delayMicroseconds(2000);  // this command takes a long time!
}

void PeanutKingSoccerV2::setCursor(uint8_t col, uint8_t row) {
  static const uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if ( row > 2 )  row = 1;
  send(LCD_SETDDRAMADDR | (col + row_offsets[row]), 0);   // **********send**********
}

size_t PeanutKingSoccerV2::printNumber(unsigned long n, uint8_t base) {
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

size_t PeanutKingSoccerV2::print(long n, int base) {
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

size_t PeanutKingSoccerV2::print(const char str[]) {
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

inline size_t PeanutKingSoccerV2::write(uint8_t value) {
  send(value, Rs);
  return 1;
}

//command(uint8_t value)   send(value, 0);
/************ low level data pushing commands **********/

// write either command or data
void PeanutKingSoccerV2::send(uint8_t value, uint8_t mode) {
  uint8_t highnib =  value    & 0xf0;     // HHHH0000
  uint8_t lownib  = (value<<4)& 0xf0;     // LLLL0000
  
  write4bits((highnib)|mode);
  write4bits((lownib)|mode);
}

void PeanutKingSoccerV2::write4bits(uint8_t value) {
  // volatile bool temp = autoScanEnabled;
  // autoScanEnabled = false;

  uint8_t msg[3] = {
    ((int)(value) | LCD_backlightval),
    ((int)(value | En) | LCD_backlightval),    //pulseEnable
  // delayMicroseconds(1);        // enable pulse must be >450ns
    ((int)(value & ~En) | LCD_backlightval)
  };
  uint8_t _status = gIIC->Write(lcdHandle, msg, 3);

  // autoScanEnabled = temp;
  delayMicroseconds(40);       // commands need > 37us to settle
}


/* =============================================================================
 *                                  Testing
 * ============================================================================= */
void PeanutKingSoccerV2::lcdMenu(void) {
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

void PeanutKingSoccerV2::testProgram (void) {
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
void PeanutKingSoccerV2::ledTest (uint8_t state) {
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
uint8_t PeanutKingSoccerV2::motorTest (void) {
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

void PeanutKingSoccerV2::btTest(void) {
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

uint8_t PeanutKingSoccerV2::pressureTest(void) {
  compoundEyeRead();
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


//                                  ???
// =================================================================================

void PeanutKingSoccerV2::debug(uint16_t sensorType) {
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

