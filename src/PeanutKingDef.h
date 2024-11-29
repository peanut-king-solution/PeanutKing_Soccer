/*
 * Copyright (c) 2024 PeanutKing Solution
 *
 * @file        PeanutKingDef.h
 * @summary     PeanutKing Solution Arduino Defines
 * @version     1.0
 * @author      Jack Kwok
 * @date        3 January 2024
 */


#ifndef PeanutKingDef_H
#define PeanutKingDef_H






typedef struct {
  volatile uint16_t pwm;
  volatile uint8_t* port;
  uint8_t bita;
  uint8_t bitb;
} motor_t;

typedef struct {
  uint16_t r;       uint16_t g;       uint16_t b;
} rgb_t;

typedef struct {
  uint16_t h;       uint8_t s;        uint8_t v;
} hsv_t;

typedef struct {
  uint16_t h;       uint8_t s;        uint8_t l;
} hsl_t;


#define TAP_DURATION    40
#define HOLD_DURATION   1000
#define WAIT_DURATION   130

typedef enum {
  NONE  = 0,
  TAP   = 1,      // -
  PRESS = 2,      // ---
  HOLD  = 3,      // ------
  TAP2  = 4,      // - -
  TAP3  = 5,      // - - -
  RELEASE = 6,
  RELEASE_S = 7,
  RELEASE_L = 8,

  TAP1_W = 10,
  TAP2_W = 11,
  TAP3_W = 12,
  HOLD2  = 13,    // - --
  
  TAP2_R = 16,
  TAP3_R = 17,
} buttonStatus_t;

// typedef struct {
//   uint32_t holdTimer;
//   uint16_t button;
//   uint16_t b;
// } button_t;


/*  Only in Version 2  */

typedef struct {
//  trgPin;
//  echoPin;
  uint32_t t;
  uint8_t  pSt;
  uint8_t  pinRead;
  int32_t  dur;
} pulsein_t;

typedef struct {
  int32_t  dur[3];
  uint32_t t;
  uint32_t f;
  int32_t  fx[3];
  uint8_t  pSt;
  uint8_t  nxt;
  uint8_t  pulse;
  uint8_t  pinRead;
} pulseinColor_t;

typedef struct {
  volatile uint8_t* port;
  uint8_t bit;
} readpin_t;

typedef struct {
  volatile uint8_t *port;
  uint8_t mask;
  uint8_t numLEDs;
  uint8_t numBytes;
  uint8_t *pixels;     // Holds LED color values (3 or 4 bytes each)
} led_t;


const float pi = 3.1415926535897;

typedef enum { front = 0, left, right, back } sensorNum;

// typedef enum {
//   black=0,  white,   grey,
//   red,      blue,    green,
//   yellow,   cyan,    magenta
// } color_t;

typedef enum {
  Idle = 0,
  Joystick = 1,
  PadButton,
  ButtonDef,
  Attributes,
  EndOfData = 26,
  DemoMode = 25,
} btData_t;


// struct pin_map_t {
//   volatile uint8_t* ddr;
//   volatile uint8_t* pin;
//   volatile uint8_t* port;
//   uint8_t bit;
// };

// static const pin_map_t pinMap[] = {
//   {&DDRD, &PIND, &PORTD, 0},  // D0  0
//   {&DDRD, &PIND, &PORTD, 1},  // D1  1
//   {&DDRD, &PIND, &PORTD, 2},  // D2  2
//   {&DDRD, &PIND, &PORTD, 3},  // D3  3
//   {&DDRD, &PIND, &PORTD, 4},  // D4  4
//   {&DDRD, &PIND, &PORTD, 5},  // D5  5
//   {&DDRD, &PIND, &PORTD, 6},  // D6  6
//   {&DDRD, &PIND, &PORTD, 7},  // D7  7
//   {&DDRB, &PINB, &PORTB, 0},  // B0  8
//   {&DDRB, &PINB, &PORTB, 1},  // B1  9
//   {&DDRB, &PINB, &PORTB, 2},  // B2 10
//   {&DDRB, &PINB, &PORTB, 3},  // B3 11
//   {&DDRB, &PINB, &PORTB, 4},  // B4 12
//   {&DDRB, &PINB, &PORTB, 5},  // B5 13
//   {&DDRC, &PINC, &PORTC, 0},  // C0 14
//   {&DDRC, &PINC, &PORTC, 1},  // C1 15
//   {&DDRC, &PINC, &PORTC, 2},  // C2 16
//   {&DDRC, &PINC, &PORTC, 3},  // C3 17
//   {&DDRC, &PINC, &PORTC, 4},  // C4 18
//   {&DDRC, &PINC, &PORTC, 5}   // C5 19
// };
// static const uint8_t pinCount = sizeof(pinMap)/sizeof(pin_map_t);

// static inline uint8_t badPinNumber(void)
//  __attribute__((error("Pin number is too large or not a constant")));

// static inline __attribute__((always_inline))
//   uint8_t digitalReadFast(uint8_t pin) {
//   if (__builtin_constant_p(pin) && pin < pinCount) {
//     return (*pinMap[pin].pin >> pinMap[pin].bit) & 1;
//   } else {
//     return badPinNumber();
//   }
// }

// static inline __attribute__((always_inline))
//   void digitalWriteFast(uint8_t pin, uint8_t value) {
//   if (__builtin_constant_p(pin) && pin < pinCount) {
//     if (value) {
//       *pinMap[pin].port |= 1 << pinMap[pin].bit;
//     } else {
//       *pinMap[pin].port &= ~(1 << pinMap[pin].bit);
//     }
//   } else {
//     badPinNumber();
//   }
// }

// #define MY_PIN    8

// // do this once at setup
// uint8_t myPin_mask = digitalPinToBitMask(MY_PIN);
// volatile uint8_t *myPin_port = portInputRegister(digitalPinToPort(MY_PIN));

// // read the pin
// uint8_t pinValue = (*myPin_port & myPin_mask) != 0;

//   digitalReadFast
//   return (*pinMap[pin].pin >> pinMap[pin].bit) & 1;
//   pin41, PG0: return (PING >> 0) & 1;
  
//   digitalWriteFast
//   if (value) {
//     *pinMap[pin].port |= 1 << pinMap[pin].bit;
//   } else {
//     *pinMap[pin].port &= ~(1 << pinMap[pin].bit);
//   }
//   pin37, PC0: PORTC |= 1 << 0;
//   PORTC |= 1 << 0;
//   PORTC &= ~(1 << 0);

#endif


