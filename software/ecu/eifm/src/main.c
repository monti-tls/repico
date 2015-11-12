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
    SPI2->CR1 &= ~(0x01 << 11);

    // CPOL = 0 (SCK 0 when idle)
    SPI2->CR1 &= ~(0x01 << 1);
    // CPHA = 0 (first clock transition is first data capture)
    SPI2->CR1 |= (0x01 << 0);

    // LSBFIRST = 0
    SPI2->CR1 &= ~(0x01 << 7);

    // SSM = 0 (hardware NSS)
    SPI2->CR1 &= ~(0x01 << 9);
    // SSI = 0 (slave enabled)
    // SPI2->CR1 &= ~(0x01 << 8);

    // MSTR = 0 (slave mode)
    SPI2->CR1 &= ~(0x01 << 2);

    // SPE = 1 (enable SPI2)
    SPI2->CR1 |= (0x01 << 6);

    // TXEIE = 0
    SPI2->CR2 &= ~(0x01 << 7);
    // RXNEIE = 1
    SPI2->CR2 &= ~(0x01 << 6);
    // ERRIE = 1
    SPI2->CR2 &= ~(0x01 << 5);

    // SPI2 is interrupt #36
    NVIC->ISER[1] |= (0x01 << 4);
    // Set max priority
    NVIC->IP[36] = (0 << 4);

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

    uint8_t tx[] = { 0xF1, 0xF2, 0xF3 };
    uint8_t rx[sizeof(tx)];
    int ptx = 0;
    int prx = 0;

    for (;;)
    {
        SPI2->DR = tx[ptx++];
        
        while (ptx < sizeof(tx))
        {
            while (!(SPI2->SR & 2));
            SPI2->DR = tx[ptx++];
            while (!(SPI2->SR & 1));
            rx[prx++] = SPI2->DR;
        }

        while (!(SPI2->SR & 1));
            rx[prx++] = SPI2->DR;

        while (!(SPI2->SR & 2));
        while ((SPI2->SR & (1 << 7)));

        for (int i = 0; i < sizeof(tx); ++i)
            itm_printf("%02X ", rx[i]);
        itm_printf("\n");

        ptx = 0;
        prx = 0;
    }

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
    SYNC_RCVE1,
    SYNC_RCVE2,
    SYNC_SEND,
    SYNCED,
    COMMAND
} rap_state = SYNC_RCVE1;

uint16_t rap_command = RAP_NONE;
uint16_t rap_sync = 0x0000;

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

        rap_state = SYNC_RCVE1;

        // Clear error flag using this software sequence
        volatile __attribute__((unused)) uint8_t dr = SPI2->DR;
        sr = SPI2->SR;
        return;
    }
    
    if (sr & (0x01 << 0)) // RXNE == 1
    {
        volatile uint16_t dr = SPI2->DR;

        // itm_printf("%04X\n", dr);
    }
    else if (sr & (0x01 << 1)) // TXNE == 1
    {
        SPI2->DR = 0x1001;
    }
}
