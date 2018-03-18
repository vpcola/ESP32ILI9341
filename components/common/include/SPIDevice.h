#ifndef _SPIDEVICE_H_
#define _SPIDEVICE_H_

#include "SPIMaster.h"

#define MAX_TRANSACTIONS 7

class SPIDevice
{
    public:
        SPIDevice(SPIMaster & spimaster, 
                gpio_num_t cspin, 
                uint8_t mode, 
                int clkspeed,
                uint8_t command_bits = 0,
                uint8_t address_bits = 0,
                uint32_t flags = 0)
            :m_spimaster(spimaster),
            m_cs(cspin),
            m_mode(mode),
            m_speed(clkspeed),
            m_commandbits(command_bits),
            m_addressbits(address_bits),
            m_flags(flags),
            m_initialized(false)
        {}
        ~SPIDevice();

        void init();

        /* Non-async read and write */
        esp_err_t spiWrite(uint8_t address, uint8_t * data, size_t len);
        esp_err_t spiRead(uint8_t address, uint8_t * data, size_t len);
        esp_err_t spiTransfer(uint8_t address, uint8_t * txdata, uint8_t * rxdata, size_t len);
        /* Async read/write, queued transactions */

    protected:
        virtual void onTransactionComplete(spi_transaction_t * t) {}
        virtual void onTransactionStart(spi_transaction_t * t) {}
    private:
        SPIMaster & m_spimaster;
        gpio_num_t m_cs;
        uint8_t m_mode;
        int m_speed;
        uint8_t m_commandbits;
        uint8_t m_addressbits;
        uint32_t m_flags;
        bool m_initialized;

        spi_device_handle_t m_spi;
        // Transaction descriptors
        uint8_t m_curtranidx;
        spi_transaction_t m_transactions[MAX_TRANSACTIONS];

        // Called from an interrupt context
        static void on_pre_transfer(spi_transaction_t * t);
        static void on_post_transfer(spi_transaction_t * t);
};

#endif
