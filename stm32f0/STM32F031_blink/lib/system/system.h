#pragma once
#include <stdio.h>

#define VCAL  423
#define CODE_CAL 0xA19
// 3.7V
#define CODE_MIN 0x970

void delay_us(uint16_t del_us);
void delay(uint32_t time);
void system_init(void);
void init_LEDs(void);
void LedMask(uint8_t mask);
void LedSet(uint8_t mask);
void LedClear(uint8_t mask);
void LedToggle(uint8_t mask);
void adc_init(void);
uint16_t adc_read(void);
void standby(void);
void init_PWM(void);
