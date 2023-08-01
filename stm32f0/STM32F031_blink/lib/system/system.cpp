#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/pwr.h>
//#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <system.h>

volatile uint32_t TimeCounter; 

void system_init(void)
{
  rcc_clock_setup_in_hsi_out_48mhz();  // set STM32 to clock by 48MHz from HSI oscillator 
  TimeCounter=0;
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  STK_CVR = 0;                                   // clear counter 
  systick_set_reload(rcc_ahb_frequency / 1000);  // Set up timer interrupt 
  systick_counter_enable();
  systick_interrupt_enable();
}

void delay_us(uint16_t del_us) 
{
  uint32_t cnt = del_us << 2;
  do
  {
    asm volatile("nop");
    asm volatile("nop");     
    asm volatile("nop");       
    asm volatile("nop");     
  } while (--cnt);
}

void delay(uint32_t time)
{
  TimeCounter = time;
  while(TimeCounter != 0);
}

void sys_tick_handler(void)
{
  if (TimeCounter |= 0) TimeCounter--;
}

void init_LEDs(void)
{
  // Enable clocks to the GPIO subsystems 
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOA);  
  // LED OUTPUTS
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2 | GPIO4 | GPIO12);
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO2);
  gpio_clear(GPIOA, GPIO2 | GPIO4 | GPIO12);
  gpio_clear(GPIOB, GPIO0 | GPIO2);
}

void LedMask(uint8_t mask)
{
  if (mask & 0x01) gpio_set(GPIOA, GPIO2); else gpio_clear(GPIOA, GPIO2);
  if (mask & 0x02) gpio_set(GPIOA, GPIO4); else gpio_clear(GPIOA, GPIO4);  
  if (mask & 0x04) gpio_set(GPIOB, GPIO0); else gpio_clear(GPIOB, GPIO0);  
  if (mask & 0x08) gpio_set(GPIOA, GPIO12); else gpio_clear(GPIOA, GPIO12);   
  if (mask & 0x10) gpio_set(GPIOB, GPIO2); else gpio_clear(GPIOB, GPIO2); 
}

void LedSet(uint8_t mask)
{
  if (mask & 0x01) gpio_set(GPIOA, GPIO2);
  if (mask & 0x02) gpio_set(GPIOA, GPIO4); 
  if (mask & 0x04) gpio_set(GPIOB, GPIO0);  
  if (mask & 0x08) gpio_set(GPIOA, GPIO12);
  if (mask & 0x10) gpio_set(GPIOB, GPIO2); 
}

void LedClear(uint8_t mask)
{
  if (mask & 0x01) gpio_clear(GPIOA, GPIO2);
  if (mask & 0x02) gpio_clear(GPIOA, GPIO4);  
  if (mask & 0x04) gpio_clear(GPIOB, GPIO0);  
  if (mask & 0x08) gpio_clear(GPIOA, GPIO12);   
  if (mask & 0x10) gpio_clear(GPIOB, GPIO2); 
}

void LedToggle(uint8_t mask)
{
  if (mask & 0x01) gpio_toggle(GPIOA, GPIO2);
  if (mask & 0x02) gpio_toggle(GPIOA, GPIO4);  
  if (mask & 0x04) gpio_toggle(GPIOB, GPIO0);  
  if (mask & 0x08) gpio_toggle(GPIOA, GPIO12);   
  if (mask & 0x10) gpio_toggle(GPIOB, GPIO2); 
}

void adc_init(void)
{
	rcc_periph_clock_enable(RCC_ADC);
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO7);
	adc_power_off(ADC1);
	adc_calibrate(ADC1);
  ADC1_CHSELR |= 1<<7;
  ADC1_CCR |= ADC_CCR_VREFEN;
  ADC1_CR |= ADC_CR_ADEN;                 //adc_power_on(ADC1);
  while (!(ADC1_ISR & ADC_ISR_ADRDY));
}


uint16_t adc_read(void)
{
    ADC1_CR |= ADC_CR_ADSTART;
    while (!(ADC1_ISR & ADC_ISR_EOC));
    return ADC1_DR;
}

void standby(void)
{
  PWR_CR |= PWR_CR_PDDS;  // Standby
  PWR_CR |= PWR_CR_CWUF;  // wakeup clear
  PWR_CR |= PWR_CR_LPDS;  //??
  __asm volatile ("cpsid i" : : : "memory");  //     disable_irq();    
  __asm volatile("wfi");
}

void init_PWM(void)
{
  // MOTOR PWM
  // TIM1 CH1 AF2 (PA8), TIM1 CH2 AF2 (PA9), TIM2 CH1 AF2 (PA0), TIM2 CH2 AF2 (PA1) 
  rcc_periph_clock_enable(RCC_TIM1);
  rcc_periph_clock_enable(RCC_TIM2);  
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO8 | GPIO9);
  gpio_set_af(GPIOA, GPIO_AF2, GPIO0 | GPIO1 | GPIO8 | GPIO9); 
  /*
  GPIO_AFRL(GPIOA) &= ~(GPIO_AFR_MASK(0) | GPIO_AFR_MASK(1)) ; 
  GPIO_AFRL(GPIOA) |= GPIO_AFR(0, GPIO_AF2) | GPIO_AFR(1, GPIO_AF2);
  GPIO_AFRH(GPIOA) &= ~(GPIO_AFR_MASK(0) | GPIO_AFR_MASK(1)) ; 
  GPIO_AFRH(GPIOA) |= GPIO_AFR(0, GPIO_AF2) | GPIO_AFR(1, GPIO_AF2);  
  */

  timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

  timer_set_prescaler(TIM1, (rcc_apb1_frequency/2000000-1));  //20kHz
  //servo 1mS 0, 1.5mS - 90, 2ms - 180 degree
  //timer_set_prescaler(TIM1, (rcc_apb1_frequency/100000));         //50Hz servo  
  timer_disable_preload(TIM1);
  timer_continuous_mode(TIM1);
  timer_set_period(TIM1, 100);
  //timer_set_period(TIM1, 2000-1);  // servo


  timer_set_prescaler(TIM2, (rcc_apb1_frequency/2000000-1));
  timer_disable_preload(TIM2);
  timer_continuous_mode(TIM2);
  timer_set_period(TIM2, 100);

  timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
  timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);
  timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_PWM1);
  timer_set_oc_mode(TIM2, TIM_OC2, TIM_OCM_PWM1);  

  timer_set_oc_value(TIM1, TIM_OC1, 25);
  timer_set_oc_value(TIM1, TIM_OC2, 50);  
  //timer_set_oc_value(TIM1, TIM_OC1, 100-1);  // 1ms  servo
  //timer_set_oc_value(TIM1, TIM_OC2, 200-1);  // 2ms dervo
  timer_set_oc_value(TIM2, TIM_OC1, 25);
  timer_set_oc_value(TIM2, TIM_OC2, 50);

  timer_set_oc_value(TIM1, TIM_OC1, 5);
  timer_set_oc_value(TIM1, TIM_OC2, 5);  
  timer_set_oc_value(TIM2, TIM_OC1, 5);
  timer_set_oc_value(TIM2, TIM_OC2, 5);



  timer_enable_oc_output(TIM1, TIM_OC1);
  timer_enable_oc_output(TIM1, TIM_OC2);
  timer_enable_break_main_output(TIM1);
  timer_enable_oc_output(TIM2, TIM_OC1);
  timer_enable_oc_output(TIM2, TIM_OC2);

  timer_enable_counter(TIM1);
  timer_enable_counter(TIM2);
}
