/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"


/*
 This code displays some fancy graphics on the 320x240 LCD on an ESP-WROVER_KIT board.
 It is not very fast, even when the SPI transfer itself happens at 8MHz and with DMA, because
 the rest of the code is not very optimized. Especially calculating the image line-by-line
 is inefficient; it would be quicker to send an entire screenful at once. This example does, however,
 demonstrate the use of both spi_device_transmit as well as spi_device_queue_trans/spi_device_get_trans_result
 as well as pre-transmit callbacks.

 Some info about the ILI9341/ST7789V: It has an C/D line, which is connected to a GPIO here. It expects this
 line to be low for a command and high for data. We use a pre-transmit callback here to control that
 line: every transaction has as the user-definable argument the needed state of the D/C line and just
 before the transaction is sent, the callback will set this line to the correct state.
*/

#define PIN_NUM_MISO 19 
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define PIN_NUM_DC   16
#define PIN_NUM_RST  17

//#define PIN_NUM_MISO 25
//#define PIN_NUM_MOSI 23
//#define PIN_NUM_CLK  19
//#define PIN_NUM_CS   22

//#define PIN_NUM_DC   21
//#define PIN_NUM_RST  18
//#define PIN_NUM_BCKL 5

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

/*
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[]={
    {0xCF, {0x00, 0x83, 0X30}, 3},
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    {0xE8, {0x85, 0x01, 0x79}, 3},
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    {0xF7, {0x20}, 1},
    {0xEA, {0x00, 0x00}, 2},
    {0xC0, {0x26}, 1},
    {0xC1, {0x11}, 1},
    {0xC5, {0x35, 0x3E}, 2},
    {0xC7, {0xBE}, 1},
    {0x36, {0x28}, 1},
    {0x3A, {0x55}, 1},
    {0xB1, {0x00, 0x1B}, 2},
    {0xF2, {0x08}, 1},
    {0x26, {0x01}, 1},
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
    // WindowMax
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4}, 
    {0x2C, {0}, 0},
    {0xB7, {0x07}, 1},
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    // Two commands below needs delay after sending the command byte
    {0x11, {0}, 0x80},  // sleep out
    {0x29, {0}, 0x80},  // display ON
    {0, {0}, 0xff},
};

#define DEFAULT_ORIENTATION 2 
static int lcd_orientation = DEFAULT_ORIENTATION;

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t) 
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}


//Send a command to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd) 
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//Send data to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len) 
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

uint32_t lcd_width()
{
    if (lcd_orientation == 0 || lcd_orientation == 2) 
        return 320;
    else
        return 240;
}

uint32_t lcd_height()
{
    if (lcd_orientation == 0 || lcd_orientation == 2)
        return 320;
    else
        return 240;
}

void lcd_window(spi_device_handle_t spi, uint32_t x, uint32_t y, uint32_t w, uint32_t h)
{
    uint8_t databytes[4];
    databytes[0] = x >> 8;
    databytes[1] = x & 0xFF;
    databytes[2] = (x+w -1) >> 8;
    databytes[3] = (x+w -1) & 0xFF;

    lcd_cmd(spi, 0x2A);
    lcd_data(spi, &databytes[0], 4);

    databytes[0] = y >> 8;
    databytes[1] = y & 0xFF;
    databytes[2] = (y+h -1) >> 8;
    databytes[3] = (y+h -1) & 0xFF;

    lcd_cmd(spi, 0x2B);
    lcd_data(spi, &databytes[0], 4);
}

void lcd_window_max(spi_device_handle_t spi)
{
    lcd_window(spi, 0, 0, lcd_height(), lcd_width());
}

void lcd_set_orientation(spi_device_handle_t spi, uint8_t orient)
{
    uint8_t data;

    lcd_cmd(spi, 0x36);
    switch(orient)
    {
        case 0: data = 0x48;
                break;
        case 1: data = 0x28;
                break;
        case 2: data = 0x88;
                break;
        case 3: data = 0xE8;
                break;
        default:
                data = 0x28;
                break;
    }

    lcd_data(spi, &data, 1);
    lcd_window_max(spi);

    lcd_orientation = orient;
}


uint32_t lcd_get_id(spi_device_handle_t spi) 
{
    //get_id cmd
    lcd_cmd( spi, 0x04);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length=8*3;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1;

    esp_err_t ret = spi_device_transmit(spi, &t);
    assert( ret == ESP_OK );

    return *(uint32_t*)t.rx_data;
}

//Initialize the display
void lcd_init(spi_device_handle_t spi) 
{
    int cmd=0;
    const lcd_init_cmd_t* lcd_init_cmds;

    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    //gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    //detect LCD type
    uint32_t lcd_id = lcd_get_id(spi);
    int lcd_type;

    printf("LCD ID: %08X\n", lcd_id);

    printf("LCD ILI9341 initialization.\n");   
    lcd_init_cmds = ili_init_cmds;

    //Send all the commands
    while (lcd_init_cmds[cmd].databytes!=0xff) 
    {
        lcd_cmd(spi, lcd_init_cmds[cmd].cmd);
        lcd_data(spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
        // if (lcd_init_cmds[cmd].databytes & 0x80) {
        if (lcd_init_cmds[cmd].databytes == 0x80) 
        {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

    lcd_set_orientation(spi, DEFAULT_ORIENTATION);

    ///Enable backlight
    //gpio_set_level(PIN_NUM_BCKL, 0);
}

void lcd_pixel(spi_device_handle_t spi, int x, int y, uint16_t color)
{
    uint8_t tmpdata[2];
    // Set x coordinate
    lcd_cmd(spi, 0x2A);
    tmpdata[0] = x >> 8;
    tmpdata[1] = x & 0xFF;
    lcd_data(spi, &tmpdata[0], 2);
    // Set y coordinate
    lcd_cmd(spi, 0x2B);
    tmpdata[0] = y >> 8;
    tmpdata[1] = y & 0xFF;
    lcd_data(spi, &tmpdata[0], 2);
    // Set the pixel
    lcd_cmd(spi, 0x2C);
    tmpdata[0] = color >> 8;
    tmpdata[1] = color & 0xFF;
    lcd_data(spi, &tmpdata[0], 2);
}

void lcd_clear(spi_device_handle_t spi, uint16_t backcolor)
{
    int i; 
    // Number of pixels per line
    int linepixels = lcd_width(); 
    int lines = lcd_height();
    int bytesperline = linepixels * 2;

    // Allocate data enough for 1 line ( 1 line per spi transaction)
    uint8_t * dataline = (uint8_t *) heap_caps_malloc(bytesperline, MALLOC_CAP_DMA); // allocate memory that is DMA capable

    if (dataline)
    {
        // Fill the dataline with color
        for (i = 0; i < bytesperline; i+=2)
        {
            dataline[i] = backcolor >> 8;
	    dataline[i+1] = backcolor & 0xFF;
        }

        lcd_window_max(spi);
        lcd_cmd(spi, 0x2C);
        // send pixels, not using DMA for now
        for (i = 0; i < lines; i++)
        {
            lcd_data(spi, (const uint8_t *) dataline, bytesperline);
        }

        // free the data line allocated
        free(dataline);
    }
}

void lcd_hline(spi_device_handle_t spi, int x0, int x1, int y, uint16_t color)
{
    int i, w;
    w = x1 - x0 + 1;
    int linepixels = w * 2;

    uint8_t * dataline = (uint8_t *) heap_caps_malloc(linepixels, MALLOC_CAP_DMA);
    if (dataline)
    {
        for(i = 0; i < linepixels; i+=2)
        {
	    dataline[i] = color >> 8;
            dataline[i+1] = color & 0xFF;
        }

        lcd_window(spi, x0, y, w, 1);
        lcd_cmd(spi, 0x2C);  // send pixel
        lcd_data(spi, (const uint8_t *) dataline, linepixels);

        lcd_window_max(spi);

        free(dataline);
    }
}

void lcd_vline(spi_device_handle_t spi, int x, int y0, int y1, uint16_t color)
{
    int i, h;
    h = y1 - y0 + 1;
    int linepixels = h * 2;

    uint8_t * dataline = (uint8_t *) heap_caps_malloc(linepixels, MALLOC_CAP_DMA);
    if (dataline)
    {
        // Fill dataline with color data
        for( i = 0 ; i < h ; i+=2)
        {
	    dataline[i] = color >> 8;
            dataline[i+1] = color & 0xFF;
        }

        lcd_window(spi, x, y0, 1, h );
        lcd_cmd(spi, 0x2C);  // send pixel
        lcd_data(spi, (const uint8_t *) dataline, linepixels);

        lcd_window_max(spi);

        free(dataline);
    }

}

void lcd_vline2(spi_device_handle_t spi, int x, int y0, int h, uint16_t color)
{
    int i;
    int linepixels = h * 2;

    uint8_t * dataline = (uint8_t *) heap_caps_malloc(linepixels, MALLOC_CAP_DMA);
    if (dataline)
    {
        // Fill dataline with color data
        for( i = 0 ; i < h ; i+=2)
        {
        dataline[i] = color >> 8;
            dataline[i+1] = color & 0xFF;
        }

        lcd_window(spi, x, y0, 1, h );
        lcd_cmd(spi, 0x2C);  // send pixel
        lcd_data(spi, (const uint8_t *) dataline, linepixels);

        lcd_window_max(spi);

        free(dataline);
    }

}


void lcd_line(spi_device_handle_t spi, int x0, int y0, int x1, int y1, uint16_t color)
{
    int   dx = 0, dy = 0;
    int   dx_sym = 0, dy_sym = 0;
    int   dx_x2 = 0, dy_x2 = 0;
    int   di = 0;

    dx = x1-x0;
    dy = y1-y0;

    if (dx == 0) {        /* vertical line */
        if (y1 > y0) lcd_vline(spi, x0,y0,y1,color);
        else lcd_vline(spi, x0,y1,y0,color);
        return;
    }

    if (dx > 0) {
        dx_sym = 1;
    } else {
        dx_sym = -1;
    }
    if (dy == 0) {        /* horizontal line */
        if (x1 > x0) lcd_hline(spi, x0,x1,y0,color);
        else  lcd_hline(spi, x1,x0,y0,color);
        return;
    }

    if (dy > 0) {
        dy_sym = 1;
    } else {
        dy_sym = -1;
    }

    dx = dx_sym*dx;
    dy = dy_sym*dy;

    dx_x2 = dx*2;
    dy_x2 = dy*2;

    if (dx >= dy) {
        di = dy_x2 - dx;
        while (x0 != x1) {

            lcd_pixel(spi, x0, y0, color);
            x0 += dx_sym;
            if (di<0) {
                di += dy_x2;
            } else {
                di += dy_x2 - dx_x2;
                y0 += dy_sym;
            }
        }
        lcd_pixel(spi, x0, y0, color);
    } else {
        di = dx_x2 - dy;
        while (y0 != y1) {
            lcd_pixel(spi, x0, y0, color);
            y0 += dy_sym;
            if (di < 0) {
                di += dx_x2;
            } else {
                di += dx_x2 - dy_x2;
                x0 += dx_sym;
            }
        }
        lcd_pixel(spi, x0, y0, color);
    }
    return;
}

void lcd_circle(spi_device_handle_t spi, int x0, int y0, int r, uint16_t color)
{
    int x = -r, y = 0, err = 2-2*r, e2;
    do {
        lcd_pixel(spi, x0-x, y0+y, color);
        lcd_pixel(spi, x0+x, y0+y,color);
        lcd_pixel(spi, x0+x, y0-y,color);
        lcd_pixel(spi, x0-x, y0-y,color);
        e2 = err;
        if (e2 <= y) {
            err += ++y*2+1;
            if (-x == y && e2 <= x) e2 = 0;
        }
        if (e2 > x) err += ++x*2+1;
    } while (x <= 0);
}

void lcd_fill_circle(spi_device_handle_t spi, int x0, int y0, int r, uint16_t color)
{
    int x = -r, y = 0, err = 2-2*r, e2;
    do {
        lcd_vline(spi, x0-x, y0-y, y0+y, color);
        lcd_vline(spi, x0+x, y0-y, y0+y, color);
        e2 = err;
        if (e2 <= y) {
            err += ++y*2+1;
            if (-x == y && e2 <= x) e2 = 0;
        }
        if (e2 > x) err += ++x*2+1;
    } while (x <= 0);
}
// Draw a circle outline
void lcd_circle2(spi_device_handle_t spi, int16_t x0, int16_t y0, int16_t r,
        uint16_t color) 
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    lcd_pixel(spi, x0  , y0+r, color);
    lcd_pixel(spi,x0  , y0-r, color);
    lcd_pixel(spi,x0+r, y0  , color);
    lcd_pixel(spi,x0-r, y0  , color);

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        lcd_pixel(spi,x0 + x, y0 + y, color);
        lcd_pixel(spi,x0 - x, y0 + y, color);
        lcd_pixel(spi,x0 + x, y0 - y, color);
        lcd_pixel(spi,x0 - x, y0 - y, color);
        lcd_pixel(spi,x0 + y, y0 + x, color);
        lcd_pixel(spi,x0 - y, y0 + x, color);
        lcd_pixel(spi,x0 + y, y0 - x, color);
        lcd_pixel(spi,x0 - y, y0 - x, color);
    }
}

void lcd_circle2_helper( spi_device_handle_t spi, int16_t x0, int16_t y0,
        int16_t r, uint8_t cornername, uint16_t color) 
{
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        if (cornername & 0x4) {
            lcd_pixel(spi,x0 + x, y0 + y, color);
            lcd_pixel(spi,x0 + y, y0 + x, color);
        }
        if (cornername & 0x2) {
            lcd_pixel(spi,x0 + x, y0 - y, color);
            lcd_pixel(spi,x0 + y, y0 - x, color);
        }
        if (cornername & 0x8) {
            lcd_pixel(spi,x0 - y, y0 + x, color);
            lcd_pixel(spi,x0 - x, y0 + y, color);
        }
        if (cornername & 0x1) {
            lcd_pixel(spi,x0 - y, y0 - x, color);
            lcd_pixel(spi,x0 - x, y0 - y, color);
        }
    }
}

// Used to do circles and roundrects
void lcd_circle2_fill_helper(spi_device_handle_t spi, int16_t x0, int16_t y0, int16_t r,
        uint8_t cornername, int16_t delta, uint16_t color) 
{

    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;

        if (cornername & 0x1) {
            lcd_vline2(spi, x0+x, y0-y, 2*y+1+delta, color);
            lcd_vline2(spi, x0+y, y0-x, 2*x+1+delta, color);
        }
        if (cornername & 0x2) {
            lcd_vline2(spi, x0-x, y0-y, 2*y+1+delta, color);
            lcd_vline2(spi, x0-y, y0-x, 2*x+1+delta, color);
        }
    }
}

void lcd_circle2_fill(spi_device_handle_t spi, int16_t x0, int16_t y0, int16_t r,
        uint16_t color) {
    lcd_vline2(spi, x0, y0-r, 2*r+1, color);
    lcd_circle2_fill_helper(spi, x0, y0, r, 3, 0, color);
}



void lcd_rect(spi_device_handle_t spi, int x0, int y0, int x1, int y1, uint16_t color)
{

    if (x1 > x0) lcd_hline(spi,x0,x1,y0,color);
    else  lcd_hline(spi,x1,x0,y0,color);

    if (y1 > y0) lcd_vline(spi,x0,y0,y1,color);
    else lcd_vline(spi,x0,y1,y0,color);

    if (x1 > x0) lcd_hline(spi,x0,x1,y1,color);
    else  lcd_hline(spi,x1,x0,y1,color);

    if (y1 > y0) lcd_vline(spi,x1,y0,y1,color);
    else lcd_vline(spi,x1,y1,y0,color);

    return;
}

void lcd_fill_rect(spi_device_handle_t spi, int x0, int y0, int x1, int y1, uint16_t color)
{

    int h = y1 - y0 + 1;
    int w = x1 - x0 + 1;
    int i,linepixels = w * 2; 

    // A full width 320 and height 240, will need at least 153K of bytes
    // to be allocated 320x240x2 (2 bytes/pixel) = 153K
    // So we do the transfers in chunks of 2048 bytes for each SPI transaction
    // Check if the number of pixels is enough to fill our buffer
    uint8_t * dataline = (uint8_t *) heap_caps_malloc(linepixels, MALLOC_CAP_DMA); // allocate memory that is DMA capable
    if (dataline)
    {
	    // fill dataline with data
	    for (i = 0; i < linepixels; i+=2)
	    {
		    dataline[i] = color >> 8;
		    dataline[i+1] = color & 0xFF;
	    }

	    lcd_window(spi, x0,y0,w,h);

	    lcd_cmd(spi, 0x2C);  // send pixel 
	    for( i = 0; i < h; i++)
	    {
		    lcd_data(spi, (const uint8_t *) dataline, linepixels);
	    }

	    free(dataline);

    }
    lcd_window_max(spi);
}


void app_main()
{
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=10*1000*1000,               //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);
    //Initialize the LCD
    lcd_init(spi);
    //Go do nice stuff.
    while(1)
    {
        lcd_clear(spi, Black);
        vTaskDelay(1000 / portTICK_RATE_MS );
        lcd_clear(spi, Red);
	vTaskDelay(1000 / portTICK_RATE_MS );
	lcd_rect(spi, 100, 100, 150, 150, Yellow);
	vTaskDelay(1000 / portTICK_RATE_MS );
	lcd_fill_rect(spi, 50, 50, 100, 100, Blue);
	//vTaskDelay(1000 / portTICK_RATE_MS );
	//lcd_vline(spi, 100, 100, 50, Blue);
        vTaskDelay(1000 / portTICK_RATE_MS );
	lcd_line(spi, 10, 10, 200, 200, Yellow);
        vTaskDelay(1000 / portTICK_RATE_MS );
	lcd_circle(spi, 120, 180, 50, Black);
        vTaskDelay(1000 / portTICK_RATE_MS );
    }
        
}
