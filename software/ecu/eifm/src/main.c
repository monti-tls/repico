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

unsigned int pos = 0;
uint8_t buf[8];

int main()
{
    itm_init();

    // Enable GPIOs
    gpio_enable(GPIOB);
    
    // Configure GPIOs to AF5
    gpio_init_af(GPIOB, 12);
    gpio_init_hs(GPIOB, 12);
    gpio_init_pp(GPIOB, 12);
    gpio_init_nopupd(GPIOB, 12);
    gpio_select_af(GPIOB, 12, 5);
    
    gpio_init_af(GPIOB, 13);
    gpio_init_hs(GPIOB, 13);
    gpio_init_pp(GPIOB, 13);
    gpio_init_nopupd(GPIOB, 13);
    gpio_select_af(GPIOB, 13, 5);
    
    gpio_init_af(GPIOB, 14);
    gpio_init_hs(GPIOB, 14);
    gpio_init_pp(GPIOB, 14);
    gpio_init_nopupd(GPIOB, 14);
    gpio_select_af(GPIOB, 14, 5);
    
    gpio_init_af(GPIOB, 15);
    gpio_init_hs(GPIOB, 15);
    gpio_init_pp(GPIOB, 15);
    gpio_init_nopupd(GPIOB, 15);
    gpio_select_af(GPIOB, 15, 5);

    // Enable peripheral clock
    spi_enable(SPI2);

    // DFF = 0 (8 bit frames)
    SPI2->CR1 &= ~(0x01 << 11);
    // CPOL = 0 (SCK 0 when idle)
    SPI2->CR1 &= ~(0x01 << 1);
    // CPHA = 0 (firct clock transition is first data capture)
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

    // TXEIE = 1
    SPI2->CR2 |= (0x01 << 7);
    // RXNEIE = 1
    SPI2->CR2 |= (0x01 << 6);
    // ERRIE = 1
    //SPI2->CR2 |= (0x01 << 5);

    // SPI2 is interrupt #36
    NVIC->ISER[1] |= (0x01 << 4);
    // Set max priority
    NVIC->IP[36] = (0 << 4);

    for (;;)
    {
        /*uint16_t sr = SPI2->SR;

        if (sr & (0x01 << 0))
        {
            buf[pos++] = SPI2->DR;
        }

        if (pos >= sizeof(buf)) {
            for (int i = 0; i < sizeof(buf); ++i)
                itm_printf("0x%02X ", buf[i]);
            itm_printf("\n");
            pos = 0;
        }
        
        if (sr & (0x01 << 1))
        {
            SPI2->DR = 0xCA;
        }*/
    }
}

void irq_spi2_handler()
{
    // Clear interrupt
    NVIC->ICPR[1] |= (0x01 << 4);

    uint16_t sr = SPI2->SR;

    // RXNE
    if (sr & (0x01 << 0))
    {
        buf[pos++] = SPI2->DR;

        if (pos >= sizeof(buf)) {
            for (int i = 0; i < sizeof(buf); ++i)
                itm_printf("0x%02X ", buf[i]);
            itm_printf("\n");
            pos = 0;
        }
    }
    // TXE
    else if (sr & (0x01 << 1))
    {
        // SPI2->DR = 0xAB;
    }
}
