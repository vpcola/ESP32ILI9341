#ifndef _SPIMaster_H_
#define _SPIMaster_H_

#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"

#define MISO_PIN GPIO_NUM_19
#define MOSI_PIN GPIO_NUM_23
#define SCLK_PIN  GPIO_NUM_18

class SPIMaster
{
    public:
        SPIMaster(spi_host_device_t spi, 
                gpio_num_t miso=MISO_PIN, 
                gpio_num_t mosi=MOSI_PIN, 
                gpio_num_t sclk=SCLK_PIN,
                int dma_channel = 0,
                int quadwp = -1,
                int quadhd = -1,
                int max_sz = 0)
                :m_spi(spi),
                m_miso(miso),
                m_mosi(mosi),
                m_sclk(sclk),
                m_dma(dma_channel),
                m_quadwp(quadwp),
                m_quadhd(quadhd),
                max_transfer_sz(max_sz)
            {}

        void init();
        esp_err_t addDevice(spi_device_interface_config_t * dev_config, spi_device_handle_t * spi_handle);
        esp_err_t removeDevice(spi_device_handle_t handle);

    private:
        spi_host_device_t m_spi;
        gpio_num_t m_miso;
        gpio_num_t m_mosi;
        gpio_num_t m_sclk;
        int m_dma;
        int m_quadwp;
        int m_quadhd;
        int max_transfer_sz;
};

#endif
