// This is the PDQ re-mixed version of Adafruit's library
// here is the original copyright notice:

/***************************************************
  This is a library for the Adafruit 1.8" SPI display.

This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
The 1.8" TFT shield
  ----> https://www.adafruit.com/product/802
The 1.44" TFT breakout
  ----> https://www.adafruit.com/product/2088
as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

//===============================================================
// This PDQ optimized version is by Xark 
//
// Inspiration from Paul Stoffregen and the Teensy 3.1 community.
//
// GOALS:
//  1) Maintain "sketch" compatibility with original Adafruit libraries.
//  2) Be as much faster as is reasonably possible honoring goal 1. :-)
//  3) Be at least as small as Adafruit libraries.
//
// I believe all three of these have largely been achieved:
// 1) Near full compatibility.  Only minor initialization changes in original sketch.
// 2) Between ~3.5 and ~12 times faster (fillRect ~2.5x, drawLine > ~14x).
// An average of almost 5x faster over entire "graphictest.ino" benchmark.
//
// Even if this library is faster, it was based on the Adafruit original. 
// Adafruit deserves your support for making their library open-source (and
// for having some nice LCD modules and all kinds of other great parts too).
// Consider giving them your support if possible!

#if !defined(_PDQ_ST7735H_)
#define _PDQ_ST7735H_

#include "PDQ_ST7735_config.h"			// PDQ: ST7735 pins and other setup for this sketchssss
// #include "Arduino.h"
#include "Print.h"

#include <PDQ_GFX.h>

#include <avr/pgmspace.h>
#include <SPI.h>

#if !defined(ST7735_CHIPSET) || !defined(ST7735_CS_PIN) || !defined(ST7735_DC_PIN)
#error Oops!  You need to #include "PDQ_ST7735_config.h" (modified with your pin configuration and options) from your sketch before #include "PDQ_ST7735.h".
#endif
#if defined(ST7735_SAVE_SPCR)
#warning ST7735_SAVE_SPCR is deprecated, use ST7735_SAVE_SPI_SETTINGS
#define ST7735_SAVE_SPI_SETTINGS ST7735_SAVE_SPCR
#endif


#include <PDQ_FastPin.h>

#if !defined(AVR_HARDWARE_SPI)
#error Oops!  Currently hardware SPI is required.  Bother Xark if you would like USI or bit-bang SPI supported.
#endif

#define INLINE		inline
#define INLINE_OPT	__attribute__((always_inline))

// Color definitions
enum
{
	ST7735_BLACK	= 0x0000,
	ST7735_BLUE	= 0x001F,
	ST7735_RED	= 0xF800,
	ST7735_GREEN	= 0x07E0,
	ST7735_CYAN	= 0x07FF,
	ST7735_MAGENTA	= 0xF81F,
	ST7735_YELLOW	= 0xFFE0,	
	ST7735_WHITE	= 0xFFFF,
};

class PDQ_ST7735 : public PDQ_GFX<PDQ_ST7735>
{
 public:
	// ST7735 commands (read commands omitted)
	// For datasheet see https://www.adafruit.com/products/358
	enum
	{
		ST7735_NOP	= 0x00,
		ST7735_SWRESET	= 0x01,

		ST7735_SLPIN	= 0x10,
		ST7735_SLPOUT	= 0x11,
		ST7735_PTLON	= 0x12,
		ST7735_NORON	= 0x13,

		ST7735_INVOFF	= 0x20,
		ST7735_INVON	= 0x21,
		ST7735_DISPOFF 	= 0x28,
		ST7735_DISPON 	= 0x29,
		ST7735_CASET	= 0x2A,
		ST7735_RASET 	= 0x2B,
		ST7735_RAMWR	= 0x2C,

		ST7735_COLMOD	= 0x3A,
		ST7735_MADCTL	= 0x36,

		ST7735_FRMCTR1	= 0xB1,
		ST7735_FRMCTR2	= 0xB2,
		ST7735_FRMCTR3	= 0xB3,
		ST7735_INVCTR	= 0xB4,
		ST7735_DISSET5	= 0xB6,

		ST7735_PWCTR1	= 0xC0,
		ST7735_PWCTR2	= 0xC1,
		ST7735_PWCTR3	= 0xC2,
		ST7735_PWCTR4	= 0xC3,
		ST7735_PWCTR5	= 0xC4,
		ST7735_VMCTR1	= 0xC5,

		ST7735_GMCTRP1	= 0xE0,
		ST7735_GMCTRN1	= 0xE1,

		ST7735_PWCTR6	= 0xFC,
	};

	// some other misc. constants
	enum
	{
		// screen dimensions
		ST7735_TFTWIDTH		= 128,
		ST7735_TFTHEIGHT_144	= 128,	// 1.44" display
		ST7735_TFTHEIGHT_18	= 160,	// 1.8" display

		// MADCTL bits
		ST7735_MADCTL_MH	= 0x04,	// bit 2 = 0 for refresh left -> right, 1 for refresh right -> left
		ST7735_MADCTL_RGB	= 0x00,	// bit 3 = 0 for RGB color order
		ST7735_MADCTL_BGR	= 0x08,	// bit 3 = 1 for BGR color order
		ST7735_MADCTL_ML	= 0x10,	// bit 4 = 0 for refresh top -> bottom, 1 for bottom -> top
		ST7735_MADCTL_MV	= 0x20,	// bit 5 = 0 for column, row order (portrait), 1 for row, column order (landscape)
		ST7735_MADCTL_MX	= 0x40,	// bit 6 = 0 for left -> right, 1 for right -> left order
		ST7735_MADCTL_MY	= 0x80,	// bit 7 = 0 for top -> bottom, 1 for bottom -> top

		// delay indicator bit for commandList()
		DELAY_TFT			= 0x80
	};
  void start_TFT();
	// higher-level routines
	PDQ_ST7735();
	// NOTE: initB and initR(tabcolor) are here for compatibility (but both just call begin()).
	// NOTE: You must set the ST7735 chip version using ST7735_CHIPSET in "PDQ_ST7735_config.h" file (in sketch folder)
	static void inline initB() __attribute__((always_inline))			{ begin(); }	// compatibility alias for begin
	static void inline initR(uint8_t tabcolor) __attribute__((always_inline))	{ (void)tabcolor; begin(); }	// compatibility alias for begin

	static void inline begin();
	static void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
	static void pushColor(uint16_t color);
	static void pushColor(uint16_t color, int count);

	// Pass 8-bit (each) R,G,B, get back 16-bit packed color

	// required driver primitive methods (all except drawPixel can call generic version in PDQ_GFX with "_" postfix).
	static void drawPixel(int x, int y, uint16_t color);
	static void drawFastVLine(int x, int y, int h, uint16_t color);
	static void drawFastHLine(int x, int y, int w, uint16_t color);
	static void setRotation(uint8_t r);
	static void invertDisplay(boolean i);

	static inline void fillScreen(uint16_t color) __attribute__((always_inline))
	{
		fillScreen_(color);			// call generic version
	}

	static void drawLine(int x0, int y0, int x1, int y1, uint16_t color);
	static void fillRect(int x, int y, int w, int h, uint16_t color);

	// === lower-level internal routines =========
	static void commandList(const uint8_t *addr);

	// NOTE: Make sure each spi_begin() is matched with a single spi_end() (and don't call either twice)
	// set CS back to low (LCD selected)
  static INLINE uint16_t color565(uint8_t r, uint8_t g, uint8_t b)
{
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
static INLINE uint16_t Color565(uint8_t r, uint8_t g, uint8_t b)	// older inconsistent name for compatibility
{
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
static inline void spi_begin() __attribute__((always_inline))
	{
#if ST7735_SAVE_SPI_SETTINGS
		SPCR = save_SPCR;
		SPSR = save_SPSR & 0x01; // SPI2X mask
#endif
		FastPin<ST7735_CS_PIN>::lo();		// CS <= LOW (selected)
	}

	// NOTE: Make sure each spi_begin() is matched with a single spi_end() (and don't call either twice)
	// reset CS back to high (LCD unselected)
	static inline void spi_end() __attribute__((always_inline))
	{
		FastPin<ST7735_CS_PIN>::hi();		// CS <= HIGH (deselected)
	}

	// 10 cycle delay (including "call")
	static void delay10() __attribute__((noinline)) __attribute__((naked)) __attribute__((used))
	{
		__asm__ __volatile__
		(
									// +4 (call to get here)
#if !defined(__AVR_HAVE_RAMPD__)	
			"	adiw	r24,0\n"	// +2 (2-cycle NOP)
#else
			"	nop\n"				// +1 (1-cycle NOP)
#endif
			"	ret\n"				// +4 (or +5 on >64KB AVR with RAMPD reg)
									// = 10 cycles
			: : : 
		);
	}

	// 13 cycle delay (including "call")
	static void delay13() __attribute__((noinline)) __attribute__((naked)) __attribute__((used))
	{
		__asm__ __volatile__
		(
									// +4 (call to get here)
			"	adiw	r24,0\n"	// +2 (2-cycle NOP)
			"	adiw	r24,0\n"	// +2 (2-cycle NOP)
#if !defined(__AVR_HAVE_RAMPD__)	
			"	nop\n"				// +1 (1-cycle NOP)
#endif
			"	ret\n"				// +4 (or +5 on >64KB AVR with RAMPD reg)
									// = 13 cycles
			: : : 
		);
	}

	// 15 cycle delay (including "call")
	static void delay15() __attribute__((noinline)) __attribute__((naked)) __attribute__((used))
	{
		__asm__ __volatile__
		(
									// +4 (call to get here)
			"	adiw	r24,0\n"	// +2 (2-cycle NOP)
			"	adiw	r24,0\n"	// +2 (2-cycle NOP)
			"	adiw	r24,0\n"	// +2 (2-cycle NOP)
#if !defined(__AVR_HAVE_RAMPD__)	
			"	nop\n"				// +1 (1-cycle NOP)
#endif
			"	ret\n"				// +4 (or +5 on >64KB AVR with RAMPD reg)
									// = 15 cycles
			: : : 
		);
	}

	// 17 cycle delay (including "call")
	static void delay17() __attribute__((noinline)) __attribute__((naked)) __attribute__((used))
	{
		__asm__ __volatile__
		(
									// +4 (call to get here)
			"	adiw	r24,0\n"	// +2 (2-cycle NOP)
			"	adiw	r24,0\n"	// +2 (2-cycle NOP)
			"	adiw	r24,0\n"	// +2 (2-cycle NOP)
			"	adiw	r24,0\n"	// +2 (2-cycle NOP)
#if !defined(__AVR_HAVE_RAMPD__)	
			"	nop\n"				// +1 (2-cycle NOP)
#endif
			"	ret\n"				// +4 (or +5 on >64KB AVR with RAMPD reg)
									// = 17 cycles
			: : : 
		);
	}
	
	// normal SPI write with minimal hand-tuned delay (assuming max DIV2 SPI rate)
	static INLINE void spiWrite(uint8_t data) INLINE_OPT
	{
		SPDR = data;
		__asm__ __volatile__
		(
			"	call	_ZN10PDQ_ST77357delay17Ev\n"	// call mangled delay17 (compiler would needlessly save/restore regs)
			: : : 
		);
	}

	// special SPI write with minimal hand-tuned delay (assuming max DIV2 SPI rate) - minus 2 cycles for RS (etc.) change
	static INLINE void spiWrite_preCmd(uint8_t data) INLINE_OPT
	{
		SPDR = data;

		__asm__ __volatile__
		(
			"	call	_ZN10PDQ_ST77357delay15Ev\n"	// call mangled delay15 (compiler would needlessly save/restore regs)
			: : : 
		);
	}

	// SPI 16-bit write with minimal hand-tuned delay (assuming max DIV2 SPI rate)
	static INLINE void spiWrite16(uint16_t data) INLINE_OPT
	{
		uint8_t temp;
		__asm__ __volatile__
		(
			"	out	%[spi],%[hi]\n"				// write SPI data (18 cycles until next write)
			"	call	_ZN10PDQ_ST77357delay17Ev\n"	// call mangled delay17 (compiler would needlessly save/restore regs)
			"	out	%[spi],%[lo]\n"				// write SPI data (18 cycles until next write)
			"	call	_ZN10PDQ_ST77357delay17Ev\n"	// call mangled delay17 (compiler would needlessly save/restore regs)

			: [temp] "=d" (temp)
			: [spi] "i" (_SFR_IO_ADDR(SPDR)), [lo] "r" ((uint8_t)data), [hi] "r" ((uint8_t)(data>>8))
			: 
		);
	}

	// SPI 16-bit write with minimal hand-tuned delay (assuming max DIV2 SPI rate) minus 2 cycles
	static INLINE void spiWrite16_preCmd(uint16_t data) INLINE_OPT
	{
		uint8_t temp;
		__asm__ __volatile__
		(
			"	out	%[spi],%[hi]\n"				// write SPI data (18 cycles until next write)
			"	call	_ZN10PDQ_ST77357delay17Ev\n"	// call mangled delay17 (compiler would needlessly save/restore regs)
			"	out	%[spi],%[lo]\n"				// write SPI data (18 cycles until next write)
			"	call	_ZN10PDQ_ST77357delay15Ev\n"	// call mangled delay15 (compiler would needlessly save/restore regs)

			: [temp] "=d" (temp)
			: [spi] "i" (_SFR_IO_ADDR(SPDR)), [lo] "r" ((uint8_t)data), [hi] "r" ((uint8_t)(data>>8))
			: 
		);
	}

	// SPI 16-bit write with minimal hand-tuned delay (assuming max DIV2 SPI rate) for drawLine
	static INLINE void spiWrite16_lineDraw(uint16_t data) INLINE_OPT
	{
		uint8_t temp;
		__asm__ __volatile__
		(
			"	out	%[spi],%[hi]\n"				// write SPI data (18 cycles until next write)
			"	call	_ZN10PDQ_ST77357delay17Ev\n"	// call mangled delay17 (compiler would needlessly save/restore regs)
			"	out	%[spi],%[lo]\n"				// write SPI data (18 cycles until next write)

			: [temp] "=d" (temp)
			: [spi] "i" (_SFR_IO_ADDR(SPDR)), [lo] "r" ((uint8_t)data), [hi] "r" ((uint8_t)(data>>8))
			: 
		);
	}

	// normal SPI write with minimal hand-tuned delay (assuming max DIV2 SPI rate)
	static INLINE void spiWrite16(uint16_t data, int count) INLINE_OPT
	{
		uint8_t temp;
		__asm__ __volatile__
		(
			"	sbiw	%[count],0\n"				// test count
			"	brmi	4f\n"					// if < 0 then done
			"	breq	4f\n"					// if == 0 then done
			"1:	out	%[spi],%[hi]\n"				// write SPI data (18 cycles until next write)
			"	call	_ZN10PDQ_ST77357delay17Ev\n"	// call mangled delay17 (compiler would needlessly save/restore regs)
			"	out	%[spi],%[lo]\n"				// write SPI data (18 cycles until next write)
			"	call	_ZN10PDQ_ST77357delay13Ev\n"	// call mangled delay13 (compiler would needlessly save/restore regs)
			"	sbiw	%[count],1\n"				// +2	decrement count
			"	brne	1b\n"					// +2/1	if != 0 then loop
										// = 13 + 2 + 2 (17 cycles)			
			"4:\n"

			: [temp] "=d" (temp), [count] "+w" (count)
			: [spi] "i" (_SFR_IO_ADDR(SPDR)), [lo] "r" ((uint8_t)data), [hi] "r" ((uint8_t)(data>>8))
			: 
		);
	}
	// write SPI byte with RS (aka D/C) pin set low to indicate a command byte (and then reset back to high when done)
	static INLINE void writeCommand(uint8_t data) INLINE_OPT
	{
		FastPin<ST7735_DC_PIN>::lo();			// RS <= LOW indicate command byte
		spiWrite_preCmd(data);
		FastPin<ST7735_DC_PIN>::hi();			// RS <= HIGH indicate data byte (always assumed left in data mode)
	}

	// write SPI byte with RS assumed low indicating a data byte
	static inline void writeData(uint8_t data) __attribute__((always_inline))
	{
		spiWrite(data);
	} 

	// internal version that does not spi_begin()/spi_end()
	static INLINE void setAddrWindow_(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) INLINE_OPT
	{
		writeCommand(ST7735_CASET); 		// column address set
		if ((ST7735_CHIPSET == ST7735_INITR_GREENTAB) || (ST7735_CHIPSET == ST7735_INITR_144GREENTAB))
		{
			spiWrite16(x0+2);			// XSTART
			spiWrite16_preCmd(x1+2);		// XEND
		}
		else
		{
			spiWrite16(x0);				// XSTART
			spiWrite16_preCmd(x1);			// XEND
		}

		writeCommand(ST7735_RASET); 		// row address set
		if (ST7735_CHIPSET == ST7735_INITR_GREENTAB)
		{
			spiWrite16(y0+1);			// YSTART
			spiWrite16_preCmd(y1+1);	 	// YEND
		}
		else if (ST7735_CHIPSET == ST7735_INITR_144GREENTAB)
		{
			spiWrite16(y0+3);			// YSTART
			spiWrite16_preCmd(y1+3);	 	// YEND
		}
		else
		{
			spiWrite16(y0);				// YSTART
			spiWrite16_preCmd(y1);		 	// YEND
		}

		writeCommand(ST7735_RAMWR); 		// write to RAM
	}

#if ST7735_SAVE_SPI_SETTINGS
	// our SPI settings, set these registers in spi_begin
	static volatile uint8_t	save_SPCR;
	static volatile uint8_t save_SPSR;
#endif
};

#endif		// !defined(_PDQ_ST7735H_)
