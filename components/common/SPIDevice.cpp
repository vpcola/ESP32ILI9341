#include "SPIDevice.h"

SPIDevice::~SPIDevice()
{
    if (m_initialized)
    {
        // Remove this device from the SPI bus
        m_spimaster.removeDevice(m_spi);
    }
}

void SPIDevice::init()
{
    esp_err_t ret;

    spi_device_interface_config_t config;

    config.mode = m_mode;
    config.spics_io_num = m_cs;
    config.queue_size = MAX_TRANSACTIONS;  // 7 transactions at a time
    config.clock_speed_hz = m_speed;
    config.command_bits = m_commandbits;
    config.address_bits = m_addressbits;
    config.flags = m_flags;
    config.pre_cb = &SPIDevice::on_pre_transfer;
    config.post_cb = &SPIDevice::on_post_transfer;
    // TODO: handle cs_ena_pretrans/cs_ena_posttrans

    // Add this device to the SPI bus and store the
    // spi handle
    ret = m_spimaster.addDevice(&config, &m_spi);
    assert(ret == ESP_OK);

    // Assuming no current transactions before a call
    // to init
    m_curtranidx = 0;
}

esp_err_t SPIDevice::spiTransfer(uint8_t address, uint8_t * txdata, uint8_t * rxdata, size_t len)
{
    spi_transaction_t trans;

    // TODO: Wait untill all queued transactions
    // are finalized (finished)

    trans.addr = address;
    trans.length = len * 8; // Transmit data length in bits
    trans.user = (void *) this;
    trans.tx_buffer = (void *) txdata;
    trans.rx_buffer = (void *) rxdata;

    return ::spi_device_transmit(m_spi, &trans);
}

esp_err_t SPIDevice::spiWrite(uint8_t address, uint8_t * data, size_t len)
{
    return spiTransfer(address, data, NULL, len);
}

esp_err_t SPIDevice::spiRead(uint8_t address, uint8_t * data, size_t len)
{
    return spiTransfer(address, NULL, data, len);
}


/*static*/ void SPIDevice::on_pre_transfer(spi_transaction_t * t)
{
    // Cast the passed transaction to extract the instance
    // of SPIDevice
    SPIDevice * dev = (SPIDevice *) t->user;
    if (dev)
    {
        dev->onTransactionStart(t);
    }
}

/*static*/ void SPIDevice::on_post_transfer(spi_transaction_t * t)
{
    // Cast the passed transaction to extract the instance
    // of SPIDevice
    SPIDevice * dev = (SPIDevice *) t->user;
    if (dev)
    {
        dev->onTransactionComplete(t);
    }
}

