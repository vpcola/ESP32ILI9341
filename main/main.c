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

#include "ili9341.h"
#include "Arial12x12.h"

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

void app_main()
{
    int i = 0;
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };

    //Initialize the SPI bus
    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
    //Attach the LCD to the SPI bus
    lcd_attach(VSPI_HOST, PIN_NUM_CS, 0, 10000000); 
    //Initialize the LCD
    lcd_init(PIN_NUM_DC, PIN_NUM_RST);
    //Go do nice stuff.
    lcd_clear(Black);
    lcd_setfont(&Arial12x12[0]);
    lcd_locate(10,30);
    while(1)
    {
        //vTaskDelay(1000 / portTICK_RATE_MS );
        //lcd_clear(Red);
        //vTaskDelay(1000 / portTICK_RATE_MS );
        //lcd_rect(100, 100, 150, 150, Yellow);
        //vTaskDelay(1000 / portTICK_RATE_MS );
        //lcd_fill_rect(50, 50, 100, 100, Blue);
        //vTaskDelay(1000 / portTICK_RATE_MS );
        //lcd_line(10, 10, 200, 200, Yellow);
        //vTaskDelay(1000 / portTICK_RATE_MS );
        //lcd_circle(120, 180, 50, Black);
        //vTaskDelay(1000 / portTICK_RATE_MS );
        lcd_printf("Vergil %i\n", i);
        i++;
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

}
