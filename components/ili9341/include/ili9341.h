#ifndef _ILI9341_H_
#define _ILI9341_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

/** 
 * Macros to convert color to 16bit RGB/BGR value
 **/
#define RGB(r,g,b)  (((r&0xF8)<<8)|((g&0xFC)<<3)|((b&0xF8)>>3)) //5 red | 6 green | 5 blue


/* some RGB color definitions                                                 */
#define Black           0x0000      /*   0,   0,   0 */
#define Navy            0x000F      /*   0,   0, 128 */
#define DarkGreen       0x03E0      /*   0, 128,   0 */
#define DarkCyan        0x03EF      /*   0, 128, 128 */
#define Maroon          0x7800      /* 128,   0,   0 */
#define Purple          0x780F      /* 128,   0, 128 */
#define Olive           0x7BE0      /* 128, 128,   0 */
#define LightGrey       0xC618      /* 192, 192, 192 */
#define DarkGrey        0x7BEF      /* 128, 128, 128 */
#define Blue            0x001F      /*   0,   0, 255 */
#define Green           0x07E0      /*   0, 255,   0 */
#define Cyan            0x07FF      /*   0, 255, 255 */
#define Red             0xF800      /* 255,   0,   0 */
#define Magenta         0xF81F      /* 255,   0, 255 */
#define Yellow          0xFFE0      /* 255, 255,   0 */
#define White           0xFFFF      /* 255, 255, 255 */
#define Orange          0xFD20      /* 255, 165,   0 */
#define GreenYellow     0xAFE5      /* 173, 255,  47 */


/**
 * Attach the ILI9341 to the SPI bus
 * @param spihost The SPI host device, can be VHSPI_HOST or HSPI_HOST
 * @param cspin   The chip select pin to use
 * @param mode    The SPI mode to use
 * @param speed   The SPI frequency, tested with 10Mhz, but the ILI9341 can accept higher.
 */
void lcd_attach(spi_host_device_t spihost, gpio_num_t cspin, uint8_t mode, uint32_t speed); 

/**
 * Returns the lcd width, based on orientation
 **/
uint32_t lcd_width();

/**
 * Returns the lcd height based on orientation of the LCD
 **/
uint32_t lcd_height();

/**
 * Sets the orientation of the LCD screen
 * @param orient The window orientation
 **/
void lcd_set_orientation(uint8_t orient);

/**
 * Get the LCD id from the lcd register.
 **/
uint32_t lcd_get_id();

/**
 * Initialize the LCD.
 * @param dc The Command/Data line gpio pin.
 * @param reset The reset gpio pin of the LCD
 **/
void lcd_init(gpio_num_t dc, gpio_num_t reset);

/**
 * Displays a single color pixel on screen 
 * @param x The x coordinate 
 * @param y The y coordinate
 * @param color The color of the pixel to display
 **/
void lcd_pixel(int x, int y, uint16_t color);

/**
 * Clears the display with a background color
 * @param backcolor The background color
 **/
void lcd_clear(uint16_t backcolor);

void lcd_hline(int x0, int x1, int y, uint16_t color);
void lcd_vline(int x, int y0, int y1, uint16_t color);
void lcd_line(int x0, int y0, int x1, int y1, uint16_t color);
void lcd_circle(int x0, int y0, int r, uint16_t color);
void lcd_fill_circle(int x0, int y0, int r, uint16_t color);
void lcd_rect(int x0, int y0, int x1, int y1, uint16_t color);
void lcd_fill_rect(int x0, int y0, int x1, int y1, uint16_t color);

// Text based routines
void lcd_setfont(unsigned char * font);
uint8_t lcd_columns();
uint8_t lcd_rows();
void lcd_locate(int x, int y);
void lcd_character(int x, int y, int c);
void lcd_putchar(char c);
void lcd_printf(const char * format, ...);

#endif

