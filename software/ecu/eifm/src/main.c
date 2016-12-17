#include "platform.h"
#include "itm.h"
#include "gpio.h"

enum
{
    RAP_CMD_WRITE = 0x01,
    RAP_CMD_READ = 0x02,

    RAP_STATUS_OK = 0xF0,
    RAP_STATUS_INVALID_CMD = 0xF1, // invalid command byte
    RAP_STATUS_INVALID_REG = 0xF2, // no such register id
    RAP_STATUS_INVALID_ACC = 0xF3  // invalid reg access (R/W issue)
};

struct rap_register
{
    uint8_t id;
    
    enum
    {
        RAP_REG_R = 0x01,
        RAP_REG_W = 0x02
    } attrs;

    enum
    {
        RAP_REG_BUFFER,
        RAP_REG_HANDLER
    } mode;

    void* buffer;
    int len;
    uint8_t (*read)(uint8_t* data);
    void (*write)(uint8_t* data, uint8_t size);
};

struct rap_device
{
    SPI_TypeDef* spi;
    struct
    {
        GPIO_TypeDef* port;
        int pin;
    } spi_nss;
    struct rap_register* registers;

    uint8_t rx[300]; // hold 256 bytes of payload + command
    unsigned int rx_len;

    uint8_t tx[300];
    unsigned int tx_len;
    unsigned int tx_ptr;

    enum
    {
        RAP_IDLE,
        RAP_COMMAND
    } state;
};

struct rap_register* rap_find_register(struct rap_device* rap, uint8_t reg)
{
    for (struct rap_register* r = rap->registers; r->id; ++r)
    {
        if (r->id == reg)
            return r;
    }

    return 0;
}

/** Definitions **/

uint8_t reg0_read(uint8_t* data)
{
    data[0] = 0xCA;
    data[1] = 0xFE;
    return 2;
}

void reg0_write(uint8_t* data, uint8_t len)
{
    itm_printf("Write reg0 ");
    for (int i = 0; i < len; ++i)
        itm_printf("%02X ", data[i]);
    itm_printf("\n");
}

int reg1;

struct rap_register registers[] =
{
    {
        0xAB,
        RAP_REG_R | RAP_REG_W,
        RAP_REG_HANDLER,
        0, 0,
        &reg0_read,
        &reg0_write
    },
    {
        0xAC,
        RAP_REG_R | RAP_REG_W,
        RAP_REG_BUFFER,
        &reg1,
        sizeof(reg1),
        0, 0
    },
    { 0x00 }
};

/** Locals **/

struct rap_device rap;

int main()
{
    itm_init();

    // Enable GPIOs
    gpio_enable(GPIOB);
    gpio_enable(GPIOC);
    
    // Configure GPIOs to AF5
    gpio_init_af(GPIOB, 12);
    gpio_select_af(GPIOB, 12, 5);

    gpio_init_af(GPIOB, 10);
    gpio_select_af(GPIOB, 10, 5);
    
    gpio_init_af(GPIOC, 2);
    gpio_select_af(GPIOC, 2, 5);
    
    gpio_init_af(GPIOC, 3);
    gpio_select_af(GPIOC, 3, 5);

    // Enable SPI2 clock
    RCC->APB1ENR |= (0x01 << 14);

    // DFF = 0 (8 bit frames)
    SPI2->CR1 &= ~(0x01 << 11);

    // CPOL = 0 (SCK 0 when idle)
    SPI2->CR1 &= ~(0x01 << 1);
    // CPHA = 0 (first clock transition is first data capture)
    SPI2->CR1 |= (0x01 << 0);

    // LSBFIRST = 0
    SPI2->CR1 &= ~(0x01 << 7);
    // SSM = 0 (hardware NSS)
    SPI2->CR1 &= ~(0x01 << 9);

    // MSTR = 0 (slave mode)
    SPI2->CR1 &= ~(0x01 << 2);

    // TXEIE = 1
    SPI2->CR2 |= (0x01 << 7);
    // RXNEIE = 1
    SPI2->CR2 |= (0x01 << 6);
    // ERRIE = 1
    SPI2->CR2 |= (0x01 << 5);

    // SPI2 is interrupt #36
    NVIC->ISER[1] |= (0x01 << 4);
    // Set max priority
    NVIC->IP[36] = (0 << 4);

    // SPE = 1 (enable SPI2)
    SPI2->CR1 |= (0x01 << 6);

    rap.spi = SPI2;
    rap.spi_nss.port = GPIOB;
    rap.spi_nss.pin = 12;
    rap.registers = &registers[0];
    rap.state = RAP_IDLE;
    rap.rx_len = 0;
    rap.tx_len = 0;
    rap.tx_ptr = 0;

    for (;;)
    {
        if (gpio_read(rap.spi_nss.port, rap.spi_nss.pin) && rap.rx_len)
        {
            switch (rap.state)
            {
                case RAP_IDLE:
                    rap.state = RAP_COMMAND;
                    
                    rap.tx_len = 1;
                    rap.tx[0] = rap.rx[0]; // sync byte
                    
                    if (rap.rx[1] == RAP_CMD_WRITE)
                    {
                        struct rap_register* reg = rap_find_register(&rap, rap.rx[2]);

                        uint8_t status = RAP_STATUS_OK;
                        if (!reg)
                            status = RAP_STATUS_INVALID_REG;
                        else if (!(reg->attrs & RAP_REG_W))
                            status = RAP_STATUS_INVALID_ACC;
                        else
                        {
                            if (reg->mode == RAP_REG_BUFFER)
                            {
                                if (rap.rx[3]  != reg->len)
                                    status = RAP_STATUS_INVALID_ACC;
                                else
                                    __builtin_memcpy(reg->buffer, &rap.rx[4], rap.rx[3]);
                            }
                            else
                                (*reg->write)(&rap.rx[4], rap.rx[3]);
                        }

                        rap.tx[rap.tx_len++] = status;
                        rap.tx[rap.tx_len++] = 0; // 0 more bytes to read
                    }
                    else if (rap.rx[1] == RAP_CMD_READ)
                    {
                        struct rap_register* reg = rap_find_register(&rap, rap.rx[2]);

                        if (!reg || !(reg->attrs & RAP_REG_R))
                        {
                            rap.tx[rap.tx_len++] = reg ? RAP_STATUS_INVALID_ACC : RAP_STATUS_INVALID_REG;
                            rap.tx[rap.tx_len++] = 0;
                        }
                        else
                        {
                            rap.tx[rap.tx_len++] = RAP_STATUS_OK;
                            uint8_t* len = &rap.tx[rap.tx_len++];

                            if (reg->mode == RAP_REG_BUFFER)
                            {
                                __builtin_memcpy(&rap.tx[rap.tx_len], reg->buffer, reg->len);
                                *len = reg->len;
                            }
                            else
                                *len = (*reg->read)(&rap.tx[rap.tx_len]);

                            rap.tx_len += *len;
                        }
                    }
                    else
                    {
                        rap.tx[rap.tx_len++] = RAP_STATUS_INVALID_CMD;
                        rap.tx[rap.tx_len++] = 0; // 0 more bytes to read
                    }
                    break;

                case RAP_COMMAND:
                    if (!rap.tx_len)
                        rap.state = RAP_IDLE;
                    break;
            }

            rap.rx_len = 0;
        }
    }
}

void irq_spi2_handler()
{
    NVIC->ICPR[1] |= (0x01 << 4);

    volatile uint16_t sr = rap.spi->SR;

    if ((sr & (0x01 << 3)) || (sr & (0x01 << 6))) // UDR == 1 or OVR == 1
    {
        itm_printf("Error: 0x%08X\n", sr);

        // Clear error flag using this software sequence
        sr = rap.spi->DR;
        sr = rap.spi->SR;
        return;
    }
    else if (sr & (0x01 << 0)) // RXNE == 1
    {
        volatile uint16_t dr = rap.spi->DR;
        rap.rx[rap.rx_len++] = dr;
    }
    else if (sr & (0x01 << 1)) // TXE == 1
    {
        if (rap.tx_len)
        {
            rap.spi->DR = rap.tx[rap.tx_ptr++];

            if (rap.tx_ptr == rap.tx_len)
            {
                rap.tx_len = 0;
                rap.tx_ptr = 0;
            }
        }
        else
            rap.spi->DR = 0x00;
    }
}
