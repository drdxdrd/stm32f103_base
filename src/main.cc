#include <sys/unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdio>

#include "stm32f10x.h"
#include "stm32f10x_conf.h" 
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

void Delay(uint32_t val);
void GpioInit();
void UsartInit();
void UartSendString(const uint8_t* data);

int main() {
  SystemInit();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GpioInit();
  UsartInit();

  float x = 0.2;
  while (true) {
    x += 0.01;
    const uint16_t pin_val = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
    const BitAction ba =
        (1 == pin_val) ? BitAction::Bit_SET : BitAction::Bit_RESET;
    GPIO_WriteBit(GPIOB, GPIO_Pin_6, ba);
    printf("pin value = [%d] x = [%.2f]\n\r", pin_val, x);
    Delay(200);
  }

  return 0;
}

void Delay(uint32_t val) {
  volatile uint64_t cnt =
      0xBE0 * val;  // 0xBE0 - подобрано очень приблизительно, что-бы val было
                    // кратно 1мс
  while (cnt) {
    cnt--;
  }
}

void GpioInit() {
  // PB6, выход
  GPIO_InitTypeDef gpio_cfg;
  gpio_cfg.GPIO_Pin = GPIO_Pin_6;
  gpio_cfg.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &gpio_cfg);
  // PB7, вход
  gpio_cfg.GPIO_Pin = GPIO_Pin_7;
  gpio_cfg.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &gpio_cfg);
}

void UsartInit() {
  // PA9, на плате отмечен как TX
  GPIO_InitTypeDef gpio_cfg;
  gpio_cfg.GPIO_Pin = GPIO_Pin_9;
  gpio_cfg.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio_cfg);
  // PA10, на плате отмечен как RX
  gpio_cfg.GPIO_Pin = GPIO_Pin_10;
  gpio_cfg.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio_cfg);
  // Usart 115200 8N1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  USART_InitTypeDef usart_cfg;
  usart_cfg.USART_BaudRate = 115200;
  usart_cfg.USART_WordLength = USART_WordLength_8b;
  usart_cfg.USART_StopBits = USART_StopBits_1;
  usart_cfg.USART_Parity = USART_Parity_No;
  usart_cfg.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart_cfg.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &usart_cfg);
  USART_Cmd(USART1, ENABLE);

  // Выключаем буферизацию для printf
  setvbuf(stdin, NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);
}

void UartSendString(const uint8_t* data) {
  while (0 != *data) {
    while (RESET == USART_GetFlagStatus(USART1, USART_FLAG_TXE)) {
      ;
    }
    USART_SendData(USART1, (*data));
    data++;
  }
}

// переопределяем системный write
#ifdef __cplusplus
extern "C" {
#endif

int _write(int file, char* ptr, int len) {
  if ((STDOUT_FILENO == file) || (STDERR_FILENO == file)) {
    for (int i = 0; i < len; i++) {
      while (RESET == USART_GetFlagStatus(USART1, USART_FLAG_TXE)) {
        ;
      }
      USART_SendData(USART1, *ptr);
      ptr++;
    }
    return len;
  } else {
    errno = EBADF;
    return -1;
  }
}

#ifdef __cplusplus
}
#endif
