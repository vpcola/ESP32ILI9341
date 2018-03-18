#include "SPIMaster.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

static const char * TAG = "SPI";

void SPIMaster::init()
{
    esp_err_t ret;
    spi_bus_config_t buscfg;
    buscfg.miso_io_num = m_miso;
    buscfg.mosi_io_num = m_mosi;
    buscfg.sclk_io_num = m_sclk;
    buscfg.quadwp_io_num = m_quadwp;
    buscfg.quadhd_io_num = m_quadhd;

    ESP_LOGI(TAG, "Initializing SPI bus (%d)", m_spi);
    ESP_LOGI(TAG, "Miso Pin = %d", m_miso);
    ESP_LOGI(TAG, "Mosi Pin = %d", m_mosi);
    ESP_LOGI(TAG, "SCLK pin = %d", m_sclk);

    ret = ::spi_bus_initialize(m_spi, &buscfg, m_dma); 
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Call to spi_bus_initialize() failed!");
    }
}

esp_err_t SPIMaster::addDevice(spi_device_interface_config_t * dev_config, spi_device_handle_t * spi_handle)
{
    return ::spi_bus_add_device(m_spi, dev_config, spi_handle);
}


esp_err_t  SPIMaster::removeDevice(spi_device_handle_t handle)
{
    return ::spi_bus_remove_device(handle);
}


