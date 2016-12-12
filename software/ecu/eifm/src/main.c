#include "platform.h"
#include "itm.h"
#include "gpio.h"

void spi_enable(SPI_TypeDef* port)
{
    if (port == SPI1)
        RCC->APB2ENR |= (0x01 << 12);
    else if (port == SPI2)
        RCC->APB1ENR |= (0x01 << 14);
    else if (port == SPI3)
        RCC->APB1ENR |= (0x01 << 15);
}

void spi_disable(SPI_TypeDef* port)
{
    if (port == SPI1)
        RCC->APB2ENR &= ~(0x01 << 12);
    else if (port == SPI2)
        RCC->APB1ENR &= ~(0x01 << 14);
    else if (port == SPI3)
        RCC->APB1ENR &= ~(0x01 << 15);
}

int main()
{
    itm_init();

    // Enable peripheral clock
    spi_enable(SPI2);

    // DFF = 0 (8 bit frames)
    SPI2->CR1 |= (0x01 << 11);

    // CPOL = 0 (SCK 0 when idle)
    SPI2->CR1 &= ~(0x01 << 1);
    // CPHA = 1 (second clock transition is first data capture)
    SPI2->CR1 &= ~(0x01 << 0);

    // LSBFIRST = 0
    SPI2->CR1 &= ~(0x01 << 7);

    // SSM = 0 (hardware NSS)
    SPI2->CR1 |= (0x01 << 9);
    // SSI = 0 (slave enabled)
    SPI2->CR1 &= ~(0x01 << 8);

    // MSTR = 0 (slave mode)
    SPI2->CR1 &= ~(0x01 << 2);

    // SPE = 1 (enable SPI2)
    SPI2->CR1 |= (0x01 << 6);

    // TXEIE = 0
    SPI2->CR2 &= ~(0x01 << 7);
    // RXNEIE = 1
    SPI2->CR2 |= (0x01 << 6);
    // ERRIE = 1
    SPI2->CR2 |= (0x01 << 5);

    // SPI2 is interrupt #36
    NVIC->ISER[1] |= (0x01 << 4);
    // Set max priority
    NVIC->IP[36] = (0 << 4);

    // SPI2->DR = 0x13;

    // Enable GPIOs
    gpio_enable(GPIOB);
    gpio_enable(GPIOC);
    
    // Configure GPIOs to AF5
    gpio_init_af(GPIOB, 10);
    gpio_select_af(GPIOB, 10, 5);
    
    gpio_init_af(GPIOC, 2);
    gpio_select_af(GPIOC, 2, 5);
    
    gpio_init_af(GPIOC, 3);
    gpio_select_af(GPIOC, 3, 5);

    for (;;);
}

enum
{
    RAP_NONE = 0x0000,
    RAP_WRITE = 0x0001,
    RAP_READ = 0x0002
};

enum
{
    SYNC_RCVE,
    SYNC_SEND,
    SYNCED,
    COMMAND
} rap_state = SYNC_RCVE;

uint16_t rap_command = RAP_NONE;

struct
{
    enum
    {
        WRITE_GET_REG_ID,
        WRITE_GET_REG_SIZE,
        WRITE_GET_REG_DATA
    } state;

    uint16_t reg_id;
    uint16_t size;
    uint16_t data[256];
    uint16_t pos;
} rap_write_cmd;

void irq_spi2_handler()
{
    // Clear interrupt
    NVIC->ICPR[1] |= (0x01 << 4);

    volatile uint16_t sr = SPI2->SR;

    if ((sr & (0x01 << 3)) || (sr & (0x01 << 6))) // UDR == 1 or OVR == 1
    {
        itm_printf("Error: 0x%08X\n", sr);

        rap_state = SYNC_RCVE;

        // Clear error flag using this software sequence
        volatile uint8_t dr = SPI2->DR;
        sr = SPI2->SR;
        return;
    }
    else if (sr & (0x01 << 0)) // RXNE == 1
    {
        volatile uint16_t dr = SPI2->DR;

        itm_printf("%04X\n", dr);

        switch (rap_state)
        {
            case SYNC_RCVE:
                SPI2->DR = dr;
                rap_state = SYNC_SEND;
                break;

            case SYNC_SEND:
                SPI2->DR = 0x00;
                rap_state = SYNCED;
                break;

            case SYNCED:
                rap_command = dr;
                rap_state = COMMAND;
                rap_write_cmd.state = WRITE_GET_REG_ID;
                break;

            case COMMAND:
                if (rap_command == RAP_WRITE)
                {
                    if (rap_write_cmd.state == WRITE_GET_REG_ID)
                    {
                        rap_write_cmd.reg_id = dr;
                        rap_write_cmd.state = WRITE_GET_REG_SIZE;
                    }
                    else if (rap_write_cmd.state == WRITE_GET_REG_SIZE)
                    {
                        rap_write_cmd.size = dr;
                        rap_write_cmd.state = WRITE_GET_REG_DATA;
                        rap_write_cmd.pos = 0;
                    }
                    else if (rap_write_cmd.state == WRITE_GET_REG_DATA)
                    {
                        rap_write_cmd.data[rap_write_cmd.pos++] = dr;
                        if (rap_write_cmd.pos >= rap_write_cmd.size)
                        {
                            rap_state = SYNC_RCVE;
                            itm_printf("Write register %04X with %04X bytes\n", rap_write_cmd.reg_id, rap_write_cmd.size);
                        }
                    }
                }
                break;
        }
    }
}
