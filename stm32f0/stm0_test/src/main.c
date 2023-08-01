#include "main.h"

#include "stm32f0xx_gpio.h"
#include "stm32f0xx_spi.h"

/* Communication boards SPIx Interface */
#define SPIx SPI1
#define SPIx_CLK RCC_APB2Periph_SPI1
#define SPIx_IRQn SPI1_IRQn
#define SPIx_IRQHandler SPI1_IRQHandler

#define SPIx_SCK_PIN GPIO_Pin_5
#define SPIx_SCK_GPIO_PORT GPIOA
#define SPIx_SCK_GPIO_CLK RCC_AHBPeriph_GPIOA
#define SPIx_SCK_SOURCE GPIO_PinSource3
#define SPIx_SCK_AF GPIO_AF_0

#define SPIx_MISO_PIN GPIO_Pin_6
#define SPIx_MISO_GPIO_PORT GPIOA
#define SPIx_MISO_GPIO_CLK RCC_AHBPeriph_GPIOA
#define SPIx_MISO_SOURCE GPIO_PinSource14
#define SPIx_MISO_AF GPIO_AF_0

#define SPIx_MOSI_PIN GPIO_Pin_7
#define SPIx_MOSI_GPIO_PORT GPIOA
#define SPIx_MOSI_GPIO_CLK RCC_AHBPeriph_GPIOA
#define SPIx_MOSI_SOURCE GPIO_PinSource15
#define SPIx_MOSI_AF GPIO_AF_0

#define SPI_DATASIZE SPI_DataSize_8b
#define SPI_DATAMASK (uint8_t)0xFF

// #define BSRR_VAL 0x0C00

SPI_InitTypeDef SPI_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

volatile uint32_t timestamp = 0;
void SysTick_Handler(void) { timestamp++; }
static void SPI_Config(void);
int main(void) {
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f0xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */

  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);

  SPI_Config();
  /* Initializes the SPI communication */
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init(SPIx, &SPI_InitStructure);

  /* Initialize the FIFO threshold */
  SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_QF);

  /* Enable the Rx buffer not empty interrupt */
  // SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_RXNE, ENABLE);
  /* Enable the SPI Error interrupt */
  //  SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_ERR, ENABLE);
  /* Data transfer is performed in the SPI interrupt routine */

  /* Enable the SPI peripheral */
  SPI_Cmd(SPIx, ENABLE);

  /* GPIOC Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* Configure PC10 and PC11 in output pushpull mode */
  // const uint16_t led_pin = GPIO_Pin_1;
  const uint16_t pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Pin = pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //////////////////////////////////////
  /// ////////////////////////////////////

  /* To achieve GPIO toggling maximum frequency, the following  sequence is
     mandatory. You can monitor PC10 and PC11 on the scope to measure the output
     signal. If you need to fine tune this frequency, you can add more GPIO
     set/reset cycles to minimize more the infinite loop timing. This code needs
     to be compiled with high speed optimization option.  */
  int z = 0;
  while (1) {
    const BitAction ba = (0 == z) ? Bit_SET : Bit_RESET;
    GPIO_WriteBit(GPIOA, pin, ba);
    z = ~z;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

static void SPI_Config(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the SPI periph */
  RCC_APB2PeriphClockCmd(SPIx_CLK, ENABLE);

  /* Enable SCK, MOSI, MISO and NSS GPIO clocks */
  RCC_AHBPeriphClockCmd(
      SPIx_SCK_GPIO_CLK | SPIx_MISO_GPIO_CLK | SPIx_MOSI_GPIO_CLK, ENABLE);

  GPIO_PinAFConfig(SPIx_SCK_GPIO_PORT, SPIx_SCK_SOURCE, SPIx_SCK_AF);
  GPIO_PinAFConfig(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_SOURCE, SPIx_MOSI_AF);
  GPIO_PinAFConfig(SPIx_MISO_GPIO_PORT, SPIx_MISO_SOURCE, SPIx_MISO_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPIx_SCK_PIN;
  GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPIx_MOSI_PIN;
  GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPIx_MISO_PIN;
  GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);

  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(SPI1);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DATASIZE;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;

  /* Configure the SPI interrupt priority */
  NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
 * @}
 */

/**
  * @}
  */

