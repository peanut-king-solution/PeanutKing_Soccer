#include <PDQ_ST7735.h>


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

#if ST7735_SAVE_SPI_SETTINGS
// static data needed by base class
volatile uint8_t PDQ_ST7735::save_SPCR;
volatile uint8_t PDQ_ST7735::save_SPSR;
#endif

// Constructor when using hardware SPI.
PDQ_ST7735::PDQ_ST7735() : PDQ_GFX<PDQ_ST7735>(ST7735_TFTWIDTH, ST7735_TFTHEIGHT_18)
{
}

// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void PDQ_ST7735::commandList(const uint8_t *addr)
{
	uint8_t	numCommands, numArgs;
	uint16_t ms;

	numCommands = pgm_read_byte(addr++);		// Number of commands to follow
	while (numCommands--)				// For each command...
	{
		writeCommand(pgm_read_byte(addr++));	// Read, issue command
		numArgs	= pgm_read_byte(addr++);	// Number of args to follow
		ms = numArgs & DELAY_TFT;			// If hibit set, delay follows args
		numArgs &= ~DELAY_TFT;			// Mask out delay bit
		while (numArgs--)			// For each argument...
		{
			writeData(pgm_read_byte(addr++)); // Read, issue argument
		}

		if (ms)
		{
			ms = pgm_read_byte(addr++);	// Read post-command delay time (ms)
			if(ms == 255)
				ms = 500;		// If 255, delay for 500 ms
			delay(ms);
		}
	}
}
      
void PDQ_ST7735::begin()
{
	// set CS and RS pin directions to output
	FastPin<ST7735_CS_PIN>::setOutput();
	FastPin<ST7735_DC_PIN>::setOutput();

	FastPin<ST7735_CS_PIN>::hi();		// CS <= HIGH (so no spurious data)
	FastPin<ST7735_DC_PIN>::hi();		// RS <= HIGH (default data byte)

	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE0);
	SPI.setClockDivider(SPI_CLOCK_DIV2);	// 8 MHz (full! speed!) [1 byte every 18 cycles]

#if ST7735_SAVE_SPI_SETTINGS
	// save our SPI settings
	save_SPCR = SPCR;
	save_SPSR = SPSR;
#endif
	spi_begin();

	// ST7735B
	if (ST7735_CHIPSET == ST7735_INITB)
	{
		// Initialization commands for ST7735B screens
		static const uint8_t PROGMEM  Bcmd[] =	// ==== Initialization commands for 7735B screens
		{
		    18,                       // 18 commands in list:
		    ST7735_SWRESET,   DELAY_TFT,  //  1: Software reset, no args, w/delay
		      50,                     //     50 ms delay
		    ST7735_SLPOUT ,   DELAY_TFT,  //  2: Out of sleep mode, no args, w/delay
		      255,                    //     255 = 500 ms delay
		    ST7735_COLMOD , 1+DELAY_TFT,  //  3: Set color mode, 1 arg + delay:
		      0x05,                   //     16-bit color
		      10,                     //     10 ms delay
		    ST7735_FRMCTR1, 3+DELAY_TFT,  //  4: Frame rate control, 3 args + delay:
		      0x00,                   //     fastest refresh
		      0x06,                   //     6 lines front porch
		      0x03,                   //     3 lines back porch
		      10,                     //     10 ms delay
		    ST7735_MADCTL , 1      ,  //  5: Memory access ctrl (directions), 1 arg:
		      0x08,                   //     Row addr/col addr, bottom to top refresh
		    ST7735_DISSET5, 2      ,  //  6: Display settings #5, 2 args, no delay:
		      0x15,                   //     1 clk cycle nonoverlap, 2 cycle gate
					      //     rise, 3 cycle osc equalize
		      0x02,                   //     Fix on VTL
		    ST7735_INVCTR , 1      ,  //  7: Display inversion control, 1 arg:
		      0x0,                    //     Line inversion
		    ST7735_PWCTR1 , 2+DELAY_TFT,  //  8: Power control, 2 args + delay:
		      0x02,                   //     GVDD = 4.7V
		      0x70,                   //     1.0uA
		      10,                     //     10 ms delay
		    ST7735_PWCTR2 , 1      ,  //  9: Power control, 1 arg, no delay:
		      0x05,                   //     VGH = 14.7V, VGL = -7.35V
		    ST7735_PWCTR3 , 2      ,  // 10: Power control, 2 args, no delay:
		      0x01,                   //     Opamp current small
		      0x02,                   //     Boost frequency
		    ST7735_VMCTR1 , 2+DELAY_TFT,  // 11: Power control, 2 args + delay:
		      0x3C,                   //     VCOMH = 4V
		      0x38,                   //     VCOML = -1.1V
		      10,                     //     10 ms delay
		    ST7735_PWCTR6 , 2      ,  // 12: Power control, 2 args, no delay:
		      0x11, 0x15,
		    ST7735_GMCTRP1,16      ,  // 13: Magical unicorn dust, 16 args, no delay:
		      0x09, 0x16, 0x09, 0x20, //     (seriously though, not sure what
		      0x21, 0x1B, 0x13, 0x19, //      these config values represent)
		      0x17, 0x15, 0x1E, 0x2B,
		      0x04, 0x05, 0x02, 0x0E,
		    ST7735_GMCTRN1,16+DELAY_TFT,  // 14: Sparkles and rainbows, 16 args + delay:
		      0x0B, 0x14, 0x08, 0x1E, //     (ditto)
		      0x22, 0x1D, 0x18, 0x1E,
		      0x1B, 0x1A, 0x24, 0x2B,
		      0x06, 0x06, 0x02, 0x0F,
		      10,                     //     10 ms delay
		    ST7735_CASET  , 4      ,  // 15: Column addr set, 4 args, no delay:
		      0x00, 0x02,             //     XSTART = 2
		      0x00, 0x81,             //     XEND = 129
		    ST7735_RASET  , 4      ,  // 16: Row addr set, 4 args, no delay:
		      0x00, 0x02,             //     XSTART = 1
		      0x00, 0x81,             //     XEND = 160
		    ST7735_NORON  ,   DELAY_TFT,  // 17: Normal display on, no args, w/delay
		      10,                     //     10 ms delay
		    ST7735_DISPON ,   DELAY_TFT,  // 18: Main screen turn on, no args, w/delay
		      255	              //     255 = 500 ms delay
		};
		commandList(Bcmd);
	}
	else
	{
		static const uint8_t PROGMEM Rcmd1[] =	// ==== Init for 7735R, part 1 (red or green tab)
		{
		    15,                       // 15 commands in list:
		    ST7735_SWRESET,   DELAY_TFT,  //  1: Software reset, 0 args, w/delay
		      150,                    //     150 ms delay
		    ST7735_SLPOUT ,   DELAY_TFT,  //  2: Out of sleep mode, 0 args, w/delay
		      255,                    //     500 ms delay
		    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
		      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
		    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
		      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
		    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
		      0x01, 0x2C, 0x2D,       //     Dot inversion mode
		      0x01, 0x2C, 0x2D,       //     Line inversion mode
		    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
		      0x07,                   //     No inversion
		    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
		      0xA2,
		      0x02,                   //     -4.6V
		      0x84,                   //     AUTO mode
		    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
		      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
		    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
		      0x0A,                   //     Opamp current small
		      0x00,                   //     Boost frequency
		    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
		      0x8A,                   //     BCLK/2, Opamp current small & Medium low
		      0x2A,  
		    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
		      0x8A, 0xEE,
		    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
		      0x0E,
		    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
		    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
		      0xC8,                   //     row addr/col addr, bottom to top refresh
		    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
		      0x05                    //     16-bit color
		};
		// ST7735R with a few types denoted by "tab type" (color of tab on LCD cover sheet) and 144 for 1.44" LCD
		commandList(Rcmd1);
		if (ST7735_CHIPSET == ST7735_INITR_GREENTAB)
		{
			static const uint8_t PROGMEM Rcmd2green[] =	// ==== Init for 7735R, part 2 (green tab only)
			{
			    2,                        //  2 commands in list:
			    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
			      0x00, 0x02,             //     XSTART = 0
			      0x00, 0x7F+0x02,        //     XEND = 127
			    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
			      0x00, 0x01,             //     XSTART = 0
			      0x00, 0x9F+0x01         //     XEND = 159
			};
			commandList(Rcmd2green);
		}
		else if (ST7735_CHIPSET == ST7735_INITR_144GREENTAB)
		{
			static const uint8_t PROGMEM Rcmd2green144[] =	// ==== Init for 7735R, part 2 (green 1.44 tab)
			{
			    2,                        //  2 commands in list:
			    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
			      0x00, 0x00,             //     XSTART = 0
			      0x00, 0x7F,             //     XEND = 127
			    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
			      0x00, 0x00,             //     XSTART = 0
			      0x00, 0x7F              //     XEND = 127
			};
			_height = ST7735_TFTHEIGHT_144;
			commandList(Rcmd2green144);
		}
		else
		{
			static const uint8_t PROGMEM Rcmd2red[] =	// Init for 7735R, part 2 (red tab only)
			{
			    2,                        //  2 commands in list:
			    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
			      0x00, 0x00,             //     XSTART = 0
			      0x00, 0x7F,             //     XEND = 127
			    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
			      0x00, 0x00,             //     XSTART = 0
			      0x00, 0x9F	      //     XEND = 159
			};
			commandList(Rcmd2red);
		}

		static const uint8_t PROGMEM Rcmd3[] =	// ==== Init for 7735R, part 3 (red or green tab)
		{
		    4,                        //  4 commands in list:
		    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
		#if 0	// Adafruit or alternate gamma settings...
		      0x02, 0x1c, 0x07, 0x12,
		      0x37, 0x32, 0x29, 0x2d,
		      0x29, 0x25, 0x2B, 0x39,
		      0x00, 0x01, 0x03, 0x10,
		#else
		      0x0f, 0x1a, 0x0f, 0x18,
		      0x2f, 0x28, 0x20, 0x22,
		      0x1f, 0x1b, 0x23, 0x37,
		      0x00, 0x07, 0x02, 0x10,
		#endif
		    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
		#if 0	// Adafruit or alternate gamma settings...
		      0x03, 0x1d, 0x07, 0x06,
		      0x2E, 0x2C, 0x29, 0x2D,
		      0x2E, 0x2E, 0x37, 0x3F,
		      0x00, 0x00, 0x02, 0x10,
		#else
		      0x0f, 0x1b, 0x0f, 0x17,
		      0x33, 0x2c, 0x29, 0x2e, 
		      0x30, 0x30, 0x39, 0x3f,
		      0x00, 0x07, 0x03, 0x10,
		#endif      
		    ST7735_NORON  ,    DELAY_TFT, //  3: Normal display on, no args, w/delay
		      10,                     //     10 ms delay
		    ST7735_DISPON ,    DELAY_TFT, //  4: Main screen turn on, no args w/delay
		      100                     //     100 ms delay
		};
		
		commandList(Rcmd3);
		if (ST7735_CHIPSET == ST7735_INITR_BLACKTAB)
		{
			// ST7735S has a few differences
			writeCommand(ST7735_MADCTL);
			writeData(ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_RGB);
		}
	}
	
	spi_end();
}

void PDQ_ST7735::start_TFT(){
  begin();
}
void PDQ_ST7735::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	spi_begin();

	setAddrWindow_(x0, y0, x1, y1);

	spi_end();
}

void PDQ_ST7735::pushColor(uint16_t color)
{
	spi_begin();

	spiWrite16_preCmd(color);

	spi_end();
}

void PDQ_ST7735::pushColor(uint16_t color, int count)
{
	spi_begin();

	spiWrite16(color, count);

	spi_end();
}

void PDQ_ST7735::drawPixel(int x, int y, uint16_t color)
{
	if ((x < 0) ||(x >= _width) || (y < 0) || (y >= _height))
		return;

	spi_begin();

	setAddrWindow_(x, y, x, y);

	spiWrite16_preCmd(color);

	spi_end();
}

void PDQ_ST7735::drawFastVLine(int x, int y, int h, uint16_t color)
{
	// clipping
	if ((x < 0) || (x >= _width) || (y >= _height))
		return;
		
	if (y < 0)
	{
		h += y;
		y = 0;
	}

	int y1 = y+h;

	if (y1 < 0)
		return;

	if (y1 > _height) 
		h = _height-y;

	spi_begin();

	setAddrWindow_(x, y, x, _height);
	spiWrite16(color, h);

	spi_end();
}


void PDQ_ST7735::drawFastHLine(int x, int y, int w, uint16_t color)
{
	// clipping
	if ((x >= _width) || (y < 0) || (y >= _height))
		return;
		
	if (x < 0)
	{
		w += x;
		x = 0;
	}

	int x1 = x+w;

	if (x1 < 0)
		return;

	if (x1 > _width) 
		w = _width-w;

	spi_begin();
	
	setAddrWindow_(x, y, _width, y);
	spiWrite16(color, w);

	spi_end();
}

void PDQ_ST7735::fillRect(int x, int y, int w, int h, uint16_t color)
{
	// rudimentary clipping (drawChar w/big text requires this)
	if ((x >= _width) || (y >= _height))
		return;
	if (x < 0)
	{
		w += x;
		x = 0;
	}
	if (y < 0)
	{
		h += y;
		y = 0;
	}
	if ((x + w) > _width)
		w = _width  - x;
	if ((y + h) > _height)
		h = _height - y;

	spi_begin();

	setAddrWindow_(x, y, x+w-1, _height);

	for (; h > 0; h--)
	{
		spiWrite16(color, w);
	}

	spi_end();
}

// Bresenham's algorithm - thx Wikipedia
void PDQ_ST7735::drawLine(int x0, int y0, int x1, int y1, uint16_t color)
{
	int8_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep)
	{
		swapValue(x0, y0);
		swapValue(x1, y1);
	}

	if (x0 > x1)
	{
		swapValue(x0, x1);
		swapValue(y0, y1);
	}

	if (x1 < 0)
		return;

	int dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int err = dx / 2;
	int8_t ystep;

	if (y0 < y1)
	{
		ystep = 1;
	}
	else
	{
		ystep = -1;
	}
	
	uint8_t setaddr = 1;

	if (steep)	// y increments every iteration (y0 is x-axis, and x0 is y-axis)
	{
		if (x1 >= _height)
			x1 = _height - 1;

		for (; x0 <= x1; x0++)
		{
			if ((x0 >= 0) && (y0 >= 0) && (y0 < _width))
				break;

			err -= dy;
			if (err < 0)
			{
				err += dx;
				y0 += ystep;
			}
		}

		if (x0 > x1)
			return;

		spi_begin();
	
		for (; x0 <= x1; x0++)
		{
			if (setaddr)
			{
				setAddrWindow_(y0, x0, y0, _height);
				setaddr = 0;
			}
			spiWrite16_lineDraw(color);
			err -= dy;
			if (err < 0)
			{
				y0 += ystep;
				if ((y0 < 0) || (y0 >= _width))
					break;
				err += dx;
				setaddr = 1;
			}
			else
			{
				__asm__ __volatile__
				(
					"	call	_ZN10PDQ_ST77357delay10Ev\n"
					: : :
				);
			}
		}
	}
	else	// x increments every iteration (x0 is x-axis, and y0 is y-axis)
	{
		if (x1 >= _width)
			x1 = _width - 1;

		for (; x0 <= x1; x0++)
		{
			if ((x0 >= 0) && (y0 >= 0) && (y0 < _height))
				break;

			err -= dy;
			if (err < 0)
			{
				err += dx;
				y0 += ystep;
			}
		}

		if (x0 > x1)
			return;

		spi_begin();

		for (; x0 <= x1; x0++)
		{
			if (setaddr)
			{
				setAddrWindow_(x0, y0, _width, y0);
				setaddr = 0;
			}
			spiWrite16_lineDraw(color);
			err -= dy;
			if (err < 0)
			{
				y0 += ystep;
				if ((y0 < 0) || (y0 >= _height))
					break;
				err += dx;
				setaddr = 1;
			}
			else
			{
				__asm__ __volatile__
				(
					"	call	_ZN10PDQ_ST77357delay10Ev\n"
					: : :
				);
			}
		}
	}
	
	spi_end();
}

void PDQ_ST7735::setRotation(uint8_t m)
{
	rotation = (m & 3); // can't be higher than 3

	spi_begin();

	writeCommand(ST7735_MADCTL);

	switch (rotation)
	{
	default:
	case 0:
		if (ST7735_CHIPSET == ST7735_INITR_BLACKTAB)
			writeData(ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_RGB);
		else
			writeData(ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_BGR);
		_width	= ST7735_TFTWIDTH;
		if (ST7735_CHIPSET == ST7735_INITR_144GREENTAB)
			_height = ST7735_TFTHEIGHT_144;
		else
			_height = ST7735_TFTHEIGHT_18;
		
		break;
	case 1:
		if (ST7735_CHIPSET == ST7735_INITR_BLACKTAB)
			writeData(ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_RGB);
		else
			writeData(ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_BGR);
		if (ST7735_CHIPSET == ST7735_INITR_144GREENTAB)
			_width = ST7735_TFTHEIGHT_144;
		else
			_width = ST7735_TFTHEIGHT_18;
		_height = ST7735_TFTWIDTH;
		break;
	case 2:
		if (ST7735_CHIPSET == ST7735_INITR_BLACKTAB)
			writeData(ST7735_MADCTL_RGB);
		else
			writeData(ST7735_MADCTL_BGR);
		_width	= ST7735_TFTWIDTH;
		if (ST7735_CHIPSET == ST7735_INITR_144GREENTAB)
			_height = ST7735_TFTHEIGHT_144;
		else
			_height = ST7735_TFTHEIGHT_18;
		break;
	case 3:
		if (ST7735_CHIPSET == ST7735_INITR_BLACKTAB)
			writeData(ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_RGB);
		else
			writeData(ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_BGR);
		if (ST7735_CHIPSET == ST7735_INITR_144GREENTAB)
			_width = ST7735_TFTHEIGHT_144;
		else
			_width = ST7735_TFTHEIGHT_18;
		_height = ST7735_TFTWIDTH;
		break;
	}

	spi_end();
}

void PDQ_ST7735::invertDisplay(boolean i)
{
	spi_begin();

	writeCommand(i ? ST7735_INVON : ST7735_INVOFF);

	spi_end();
}
// Nothing, all in the header
