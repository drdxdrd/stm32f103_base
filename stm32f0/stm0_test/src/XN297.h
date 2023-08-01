#pragma once

#include <stdint.h>
#include <stdio.h>

#define BUFF_LENGTH 20
extern uint8_t tx_buff[BUFF_LENGTH];
extern uint8_t rx_buff[BUFF_LENGTH];

void init_XN297(void);
void rx_pack(void);
void bind(void);
