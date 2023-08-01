#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <SPI.h>

// software SPI
// PB5 CS, PB4 SCK, PB3 MOSI, PA15 MISO
#define SPI_SS GPIO5
#define SPI_SCK GPIO4
#define SPI_MOSI GPIO3
#define SPI_MISO GPIO15

void init_SPI(void)
{
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOA);  
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPI_SS | SPI_SCK | SPI_MOSI);
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SPI_MISO);
  gpio_clear(GPIOB, SPI_SCK | SPI_MOSI);
  gpio_set(GPIOB, SPI_SS);
}

uint8_t transfer_byte_SPI(uint8_t data)
{
  uint8_t rx_data=0;
  GPIO_BSRR(GPIOB) = (SPI_SS<<16); 
  uint8_t i = 8;
  do
  {
    rx_data <<= 1;
    if (data&0x80) GPIO_BSRR(GPIOB) = (SPI_MOSI); 
    else           GPIO_BSRR(GPIOB) = (SPI_MOSI << 16);  
    GPIO_BSRR(GPIOB) = (SPI_SCK); 
    if (GPIO_IDR(GPIOA) & SPI_MISO) rx_data |= 0x1;
    GPIO_BSRR(GPIOB) = (SPI_SCK<<16); 
    data <<= 1;
  }while(--i);
  GPIO_BSRR(GPIOB) = (SPI_SS);   
  return rx_data;
}

uint16_t transfer_word_SPI(uint8_t reg, uint8_t data)
{
  uint16_t tx_data=data;
  tx_data |= reg << 8;
  uint16_t rx_data=0;
  GPIO_BSRR(GPIOB) = (SPI_SS<<16); 
  uint8_t i = 16;
  do
  {
    rx_data <<= 1;
    if (tx_data&0x8000) GPIO_BSRR(GPIOB) = (SPI_MOSI); 
    else             GPIO_BSRR(GPIOB) = (SPI_MOSI << 16);  
    GPIO_BSRR(GPIOB) = (SPI_SCK); 
    if (GPIO_IDR(GPIOA) & SPI_MISO) rx_data |= 0x1;
    GPIO_BSRR(GPIOB) = (SPI_SCK<<16); 
    tx_data <<= 1;
  }while(--i);
  GPIO_BSRR(GPIOB) = (SPI_SS);   
  return rx_data;
}

void transfer_buff(uint8_t *txpointer, uint8_t *rxpointer, uint8_t length)
{
  GPIO_BSRR(GPIOB) = (SPI_SS<<16); 
  do
  {
    uint8_t i = 8;
    uint8_t rx_data=0;
    uint8_t tx_data= *txpointer;
    do
    {
      rx_data <<= 1;
      if (tx_data&0x80) GPIO_BSRR(GPIOB) = (SPI_MOSI); 
      else             GPIO_BSRR(GPIOB) = (SPI_MOSI << 16);  
      GPIO_BSRR(GPIOB) = (SPI_SCK); 
      if (GPIO_IDR(GPIOA) & SPI_MISO) rx_data |= 0x1;
      GPIO_BSRR(GPIOB) = (SPI_SCK<<16); 
      tx_data <<= 1;
    } while (--i);
    *rxpointer = rx_data;
    txpointer++;
    rxpointer++;
  } while(--length);
  GPIO_BSRR(GPIOB) = (SPI_SS);   
}
