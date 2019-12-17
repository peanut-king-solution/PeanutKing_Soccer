#include "PeanutKing_Soccer.h"


PeanutKing_Soccer::PeanutKing_Soccer() {
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

//                             RAW
// ================================================================

// rawButton  -----------------------------------------------------
bool PeanutKing_Soccer::rawButton(uint8_t pin) {
  return !digitalRead(pin);
}

// return single eye reading  -------------------------------------
uint16_t PeanutKing_Soccer::rawCompoundEye(uint8_t pin) {
  return 1023 - analogRead(pin);
}

// rawCompass -----------------------------------------------------
float PeanutKing_Soccer::rawCompass(int8_t addr, int8_t cmd) {
  uint8_t received_byte[3] = {0,0,0};
  uint8_t i=0;
  uint16_t temp=0;
  float answer = 888;
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  Wire.endTransmission();
  Wire.requestFrom(addr, 3);
  while (Wire.available()) {
    received_byte[i++] = Wire.read();
  }
  temp  = received_byte[1] & 0xFF;
  temp |= (received_byte[2] << 8);
  answer = temp/100.0;

  return answer;
}

// rawUltrasonic --------------------------------------------------
uint16_t PeanutKing_Soccer::rawUltrasonic(uint8_t txPin, uint8_t rxPin) {
  uint32_t duration=0;
  uint16_t distance=0;
  digitalWrite(txPin, LOW);
  delayMicroseconds(2);
  digitalWrite(txPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(txPin, LOW);
  duration = pulseIn(rxPin, HIGH, 13000);
  distance = ( duration==0 ) ? 888 : duration*0.017; //0.034/2;
  return distance;
}

// rawMonoColor ---------------------------------------------------
uint8_t PeanutKing_Soccer::rawMonoColor(uint8_t out) {
  // v2.1 robot 180-20
  const uint16_t MAX = 900, MIN = 100;
  int16_t temp = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH, 2000);

  if ( temp == 0 ) temp = MAX;
  temp = map(temp, MAX, MIN, 0, 255);
  temp = constrain(temp, 0, 255);
  return temp;
}

// Bluetooth ---------------------------------------------------
void PeanutKing_Soccer::bluetoothSend(char string[]) {
// send char
  Serial1.write(string, sizeof(string));
}

void PeanutKing_Soccer::bluetoothReceive(void) {
// send char
  btRxBuffer[0] = Serial1.read();
}


// turn on/off specific sensory system scannig function
// CompoundEye + Compass + Ultrasonic + ColorSense
void PeanutKing_Soccer::enableScanning(bool enable, uint16_t sensorType, bool enableLED) {
  autoScanEnabled = enable;
  autoScanSensors = sensorType;
  ledEnabled = enableLED;
}

// col(0-15), row(0-1) --------------------------------------------
void PeanutKing_Soccer::setScreen(uint8_t col, uint8_t row, char string[]) {
  setCursor(col, row);
  print(string);
}

void PeanutKing_Soccer::setScreen(uint8_t col, uint8_t row, int16_t numbers, uint8_t digits) {
  setCursor(col, row);
  for ( int i=1; i<digits; i++ ) {
    if ( numbers < pow(10, i) )   print(" ");
  }
  print(numbers);
}

void PeanutKing_Soccer::printSpace(uint32_t data, uint8_t digit) {
  for ( int i=1; i<digit; i++ ) {
    if ( data < pow(10, i) )   Serial.print(" ");
  }
  Serial.print(data);
}


//                                  LEDs
// ================================================================
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
      uint8_t *p = &leds[x].pixels[i * 4];  // 4 bytes per pixel
      p[1] = r;                   // R
      p[0] = g;                   // G
      p[2] = b;                   // B
      p[3] = w;                   // W
    }
  }
}

void PeanutKing_Soccer::ledSetPixels(uint8_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
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
  ledShow(255, 0, 0, 0, 0);
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
// ================================================================

void PeanutKing_Soccer::lcdSetup (void) {
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
  lcdClear();
  
  // set the entry mode
  send(LCD_ENTRYMODESET | LCD_displaymode, 0);         // **********send**********
  
  send(LCD_RETURNHOME, 0);  // set cursor position to zero
  delayMicroseconds(2000);  // this command takes a long time!
  
  LCD_backlightval=LCD_BACKLIGHT;
}


/********** high level commands, for the user! */
void PeanutKing_Soccer::lcdClear(void) {
  send(LCD_CLEARDISPLAY, 0);// clear display, set cursor position to zero
  delayMicroseconds(2000);  // this command takes a long time!
}

void PeanutKing_Soccer::setCursor(uint8_t col, uint8_t row) {
  static const uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if ( row > 2 )  row = 1;
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


