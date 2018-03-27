#include <stdlib.h>
#include <string.h>
#include "ili9341.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#
/**
 * The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
 **/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**
 * The spi handle for the ILI9341 device, this 
 * gets initialized on a call to lcd_attach()
 **/
static spi_device_handle_t spi;

/**
 * The gpio pin used for selecting command or data. Gets assigned during the
 * lcd_init() call
 **/
static gpio_num_t dcpin;

/**
 * The current font in use
 **/
static uint8_t * font = NULL;

/**
 * Current character location
 **/
static uint16_t char_x = 0;
static uint16_t char_y = 0;

/**
 * Current background and foreground
 * color for text
 **/
static uint16_t background = Black;
static uint16_t foreground = White;
    
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
    gpio_set_level(dcpin, dc);
}

//Send a command to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
static void lcd_cmd(const uint8_t cmd) 
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
static void lcd_data(const uint8_t *data, int len) 
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

static void lcd_window(uint32_t x, uint32_t y, uint32_t w, uint32_t h)
{
    uint8_t databytes[4];
    databytes[0] = x >> 8;
    databytes[1] = x & 0xFF;
    databytes[2] = (x+w -1) >> 8;
    databytes[3] = (x+w -1) & 0xFF;

    lcd_cmd(0x2A);
    lcd_data(&databytes[0], 4);

    databytes[0] = y >> 8;
    databytes[1] = y & 0xFF;
    databytes[2] = (y+h -1) >> 8;
    databytes[3] = (y+h -1) & 0xFF;

    lcd_cmd(0x2B);
    lcd_data(&databytes[0], 4);
}

static void lcd_window_max()
{
    lcd_window(0, 0, lcd_height(), lcd_width());
}

void lcd_set_orientation(uint8_t orient)
{
    uint8_t data;

    lcd_cmd(0x36);
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

    lcd_data(&data, 1);
    lcd_window_max();

    lcd_orientation = orient;
}


void lcd_attach(spi_host_device_t spihost, gpio_num_t cspin, uint8_t mode, uint32_t speed)
{
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=speed,               //Clock out at 10 MHz
        .mode=mode,                                //SPI mode 0
        .spics_io_num=cspin,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };

    // Attach the LCD to the SPI bus, intialize the spi
    // variable
    esp_err_t ret=spi_bus_add_device(spihost, &devcfg, &spi);
    assert(ret==ESP_OK);
}

uint32_t lcd_get_id() 
{
    //get_id cmd
    lcd_cmd(0x04);

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
void lcd_init(gpio_num_t dc, gpio_num_t reset) 
{
    int cmd=0;
    const lcd_init_cmd_t* lcd_init_cmds;

    // Initialize/Backup the dc pin to a local static
    // var
    dcpin = dc;

    //Initialize non-SPI GPIOs
    gpio_set_direction(dcpin, GPIO_MODE_OUTPUT);
    gpio_set_direction(reset, GPIO_MODE_OUTPUT);
    //gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    //Reset the display
    gpio_set_level(reset, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(reset, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    //detect LCD type
    uint32_t lcd_id = lcd_get_id();
    printf("LCD ID: %08X\n", lcd_id);

    printf("LCD ILI9341 initialization.\n");   
    lcd_init_cmds = ili_init_cmds;

    //Send all the commands
    while (lcd_init_cmds[cmd].databytes!=0xff) 
    {
        lcd_cmd(lcd_init_cmds[cmd].cmd);
        lcd_data(lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
        // if (lcd_init_cmds[cmd].databytes & 0x80) {
        if (lcd_init_cmds[cmd].databytes == 0x80) 
        {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

    lcd_set_orientation(DEFAULT_ORIENTATION);

    ///Enable backlight
    //gpio_set_level(PIN_NUM_BCKL, 0);
}

void lcd_pixel(int x, int y, uint16_t color)
{
    uint8_t tmpdata[2];
    // Set x coordinate
    lcd_cmd(0x2A);
    tmpdata[0] = x >> 8;
    tmpdata[1] = x & 0xFF;
    lcd_data(&tmpdata[0], 2);
    // Set y coordinate
    lcd_cmd(0x2B);
    tmpdata[0] = y >> 8;
    tmpdata[1] = y & 0xFF;
    lcd_data(&tmpdata[0], 2);
    // Set the pixel
    lcd_cmd(0x2C);
    tmpdata[0] = color >> 8;
    tmpdata[1] = color & 0xFF;
    lcd_data(&tmpdata[0], 2);
}

void lcd_clear(uint16_t backcolor)
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

        lcd_window_max();
        lcd_cmd(0x2C);
        // send pixels, not using DMA for now
        for (i = 0; i < lines; i++)
        {
            lcd_data((const uint8_t *) dataline, bytesperline);
        }

        // free the data line allocated
        free(dataline);
    }
}

void lcd_hline(int x0, int x1, int y, uint16_t color)
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

        lcd_window(x0, y, w, 1);
        lcd_cmd(0x2C);  // send pixel
        lcd_data((const uint8_t *) dataline, linepixels);

        lcd_window_max();

        free(dataline);
    }
}

void lcd_vline(int x, int y0, int y1, uint16_t color)
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

        lcd_window(x, y0, 1, h );
        lcd_cmd(0x2C);  // send pixel
        lcd_data((const uint8_t *) dataline, linepixels);

        lcd_window_max();

        free(dataline);
    }

}

void lcd_line(int x0, int y0, int x1, int y1, uint16_t color)
{
    int   dx = 0, dy = 0;
    int   dx_sym = 0, dy_sym = 0;
    int   dx_x2 = 0, dy_x2 = 0;
    int   di = 0;

    dx = x1-x0;
    dy = y1-y0;

    if (dx == 0) {        /* vertical line */
        if (y1 > y0) lcd_vline(x0,y0,y1,color);
        else lcd_vline(x0,y1,y0,color);
        return;
    }

    if (dx > 0) {
        dx_sym = 1;
    } else {
        dx_sym = -1;
    }
    if (dy == 0) {        /* horizontal line */
        if (x1 > x0) lcd_hline(x0,x1,y0,color);
        else  lcd_hline(x1,x0,y0,color);
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

            lcd_pixel(x0, y0, color);
            x0 += dx_sym;
            if (di<0) {
                di += dy_x2;
            } else {
                di += dy_x2 - dx_x2;
                y0 += dy_sym;
            }
        }
        lcd_pixel(x0, y0, color);
    } else {
        di = dx_x2 - dy;
        while (y0 != y1) {
            lcd_pixel(x0, y0, color);
            y0 += dy_sym;
            if (di < 0) {
                di += dx_x2;
            } else {
                di += dx_x2 - dy_x2;
                x0 += dx_sym;
            }
        }
        lcd_pixel(x0, y0, color);
    }
    return;
}

void lcd_circle(int x0, int y0, int r, uint16_t color)
{
    int x = -r, y = 0, err = 2-2*r, e2;
    do {
        lcd_pixel( x0-x, y0+y, color);
        lcd_pixel( x0+x, y0+y,color);
        lcd_pixel( x0+x, y0-y,color);
        lcd_pixel( x0-x, y0-y,color);
        e2 = err;
        if (e2 <= y) {
            err += ++y*2+1;
            if (-x == y && e2 <= x) e2 = 0;
        }
        if (e2 > x) err += ++x*2+1;
    } while (x <= 0);
}

void lcd_fill_circle(int x0, int y0, int r, uint16_t color)
{
    int x = -r, y = 0, err = 2-2*r, e2;
    do {
        lcd_vline( x0-x, y0-y, y0+y, color);
        lcd_vline( x0+x, y0-y, y0+y, color);
        e2 = err;
        if (e2 <= y) {
            err += ++y*2+1;
            if (-x == y && e2 <= x) e2 = 0;
        }
        if (e2 > x) err += ++x*2+1;
    } while (x <= 0);
}

void lcd_rect(int x0, int y0, int x1, int y1, uint16_t color)
{

    if (x1 > x0) lcd_hline(x0,x1,y0,color);
    else  lcd_hline(x1,x0,y0,color);

    if (y1 > y0) lcd_vline(x0,y0,y1,color);
    else lcd_vline(x0,y1,y0,color);

    if (x1 > x0) lcd_hline(x0,x1,y1,color);
    else  lcd_hline(x1,x0,y1,color);

    if (y1 > y0) lcd_vline(x1,y0,y1,color);
    else lcd_vline(x1,y1,y0,color);

    return;
}

void lcd_fill_rect(int x0, int y0, int x1, int y1, uint16_t color)
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

        lcd_window(x0,y0,w,h);

        lcd_cmd(0x2C);  // send pixel 
        for( i = 0; i < h; i++)
        {
            lcd_data((const uint8_t *) dataline, linepixels);
        }

        free(dataline);

    }
    lcd_window_max();
}

void lcd_setfont(unsigned char * f)
{
    font = f;
}

uint8_t lcd_columns()
{
    return lcd_width() / font[1];
}

uint8_t lcd_rows()
{
    return lcd_height() / font[2];
}

void lcd_locate(int x, int y)
{
    char_x = x;
    char_y = y;
}

void lcd_character(int x, int y, int c)
{
    unsigned int hor,vert,offset,bpl,j,i,b;
    unsigned char* charoffset;
    unsigned char z, w = 0;

    // return if no font is selected
    if (font == NULL)
        return;

    if ((c < 31) || (c > 127)) return;   // test char range
    // read font parameter from start of array
    offset = font[0];                    // bytes / char
    hor = font[1];                       // get hor size of font
    vert = font[2];                      // get vert size of font
    bpl = font[3];                       // bytes per line

    // Get the offset to the sign/symbol.
    charoffset = &font[((c -32) * offset) + 4]; // start of char bitmap

    // Calculate if we are within bounds, fix
    // char_x and char_y if out of bounds
    if (char_x + hor > lcd_width()) {
        char_x = 0;
        char_y = char_y + vert;
        if (char_y >= lcd_height() - font[2]) {
            char_y = 0;
        }
    }

    // Allocate data for transfers
    // vertical bytes * horizontal bytes * 2 bytes per pizel. 
    uint16_t *chardata = heap_caps_malloc( vert * hor * 2, MALLOC_CAP_DMA);
    uint16_t numsend = 0;
    if (chardata)
    {
        w = charoffset[0];     // width of actual char
        for (j=0; j<vert; j++) // Process vertical line
        {  
            for (i=0; i<hor; i++) // Process horizontal line 
            {   
                // Move to the start of the pixel array (skip the first byte, since its the 
                // encoded width)
                z =  charoffset[bpl * i + ((j & 0xF8) >> 3) + 1];
                b = 1 << (j & 0x07);

                if (( z & b ) == 0x00) {
                    chardata[numsend]   = background; 
                } else {
                    chardata[numsend]   = foreground;
                }

                numsend ++;
            }
        }

        // Define the character box
        lcd_window(char_x, char_y,hor,vert); 
        // Send pixel command
        lcd_cmd(0x2C);  
        // Send data over spi
        lcd_data((const uint8_t *) chardata, (numsend * 2)); // at 16bit per pixel
        // Delete the buffer
        free(chardata);
    }

    lcd_window_max();
    if ((w + 2) < hor) {                   // x offset to next char
        char_x += w + 2;
    } else char_x += hor;
}

void lcd_putchar(char c)
{
    if (c == '\n') {    // new line
        char_x = 0;
        char_y = char_y + font[2];
        if (char_y >= lcd_height() - font[2]) {
            char_y = 0;
        }
    } else {
        lcd_character(char_x, char_y, c);
    }
}

void lcd_printf(const char * format, ...)
{
    char buffer[256];
    uint8_t i, buflen;

    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);
    
    buflen = strlen(&buffer[0]);
    for (i = 0; i < buflen; i++)
    {
        lcd_putchar(buffer[i]);
    }
    
    va_end(args);
}

