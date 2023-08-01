#pragma once

#include <stdio.h>

void init_USART(void);
void tx_UART(uint8_t c);
void tx_byte_UART(uint8_t c);
void tx_word_UART(uint16_t c);
void buff_UART(uint8_t *pointer, uint8_t length);
void str2UART(uint8_t *pointer);
void int2UART(uint16_t Number);