#pragma once

#include <stdint.h>
#include <stdio.h>

void init_SPI(void);
uint8_t transfer_byte_SPI(uint8_t data);
uint16_t transfer_word_SPI(uint8_t reg, uint8_t data);
void transfer_buff(uint8_t *txpointer, uint8_t *rxpointer, uint8_t length);
