#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
//#include <libopencm3/stm32/usart.h>
//#include <libopencm3/stm32/usart.h>
//#include <libopencm3/stm32/i2c.h>


//#include <libopencm3/cm3/cortex.h>


#include <XN297.h>
#include <SPI.h>
#include <UART.h>
#include <MPU9250.h>
#include <system.h>


/*
int main(void)
{
  system_init();
  init_LEDs();
  //init_USART();
  while (1)
  {
    uint8_t mask = 1;
    for(uint8_t i=0; i<5; i++)
    {
      LedMask(mask);
      mask <<=1;
      delay(500);
    }
  }
}
*/

void CheckVoltage(void)
{
  adc_read();
  delay(100);
  adc_read();
  if (adc_read()<CODE_MIN)
  {  
    for(uint8_t j=0; j<6; j++)
    {
      uint8_t mask = 1;
      for(uint8_t i=0; i<5; i++)
      {
        LedMask(mask);
        mask <<=1;
        delay(100);
      }
    }
    standby();
  }
}

void TestRadio(void)
{
  static bool binded = false;
  uint16_t out16;
  static uint8_t blink_cnt=0;
  out16 = transfer_word_SPI(0x07, 0xFF);
  if (out16 & 0x40)
  { 
    if (binded) rx_pack();
    else 
    {
      bind();
      binded = true;
      LedClear(0x10);
    }
    buff_UART(rx_buff,16);
  }
  else
  {
    if (!binded) 
    {
      blink_cnt++;
      if (blink_cnt==50)
      {
        blink_cnt=0;
        LedToggle(0x10);
      }
    }
  }
  delay(2);
}
  
/*
int main(void)
{
  system_init();
  init_LEDs();
  adc_init();
  init_USART();
  init_SPI();
  init_XN297();
  CheckVoltage();
  uint8_t mask = 1;
  for(uint8_t i=0; i<5; i++)
  {
    LedMask(mask);
    mask <<=1;
    delay(500);
  }
  while (1)
  {
    TestRadio(); 
  }
}
*/

uint8_t u8map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) 
{
  return  (x - in_min) *(out_max - out_min) / (in_max - in_min) + out_min;
}

/*
int main(void)
{
  system_init();
  init_LEDs();
  init_PWM();
  init_SPI();
  init_USART();
  init_XN297();

  bool binded = false;
  while (1)
  {
    uint16_t out16;
    out16 = transfer_word_SPI(0x07, 0xFF);
    if (out16 & 0x40)
    { 
      if (binded) 
      {
        rx_pack();
        buff_UART(rx_buff,16);    
        uint8_t status;
        status = rx_buff[14];
        if(status &0x04) gpio_set(GPIOA, GPIO2);  else gpio_clear(GPIOA, GPIO2);
        if(status &0x08) gpio_set(GPIOA, GPIO4);  else gpio_clear(GPIOA, GPIO4);
        if(status &0x10) gpio_set(GPIOA, GPIO12); else gpio_clear(GPIOA, GPIO12);
        if(status &0x20) gpio_set(GPIOB, GPIO0);  else gpio_clear(GPIOB, GPIO0);
        if(status &0x40) gpio_set(GPIOB, GPIO2);  else gpio_clear(GPIOB, GPIO2);
        status = rx_buff[6];
        status = u8map(status, 0, 0xff, 0, 100);
        timer_set_oc_value(TIM1, TIM_OC1, status);
        status = rx_buff[7];
        status = u8map(status, 0x43, 0xbb, 0, 100);
        timer_set_oc_value(TIM1, TIM_OC2, status);
        status = rx_buff[8];
        status = u8map(status, 0x43, 0xbb, 0, 100);
        timer_set_oc_value(TIM2, TIM_OC1, status);
        status = rx_buff[9];
        status = u8map(status, 0x43, 0xbb, 0, 100);
        timer_set_oc_value(TIM2, TIM_OC2, status);        
      }
      else 
      {
        bind();
        binded = true;
      }
    }
    delay(100);
  }
}
*/

cImu IMU;
uint8_t cout;

void TestImu(void)
{
    IMU.readAccelData();  // Read the x/y/z adc values
    IMU.readGyroData();  // Read the x/y/z adc values
    IMU.Mahony_no_mag_Update();
    delay(20);
    cout++;
    if (cout==50)
    {
      IMU.quater2euler();
      cout=0;
      int16_t Angles[3];
      Angles[0] = (int16_t)(IMU.roll);
    	Angles[1] = (int16_t)(IMU.pitch);
      Angles[2] = (int16_t)(IMU.yaw);
      
      for(uint i=0; i<3; i++)
      {
        int2UART(Angles[i]);
        tx_UART(' ');  
      }
      tx_UART(0xa);
      tx_UART(0xd);       
    }   
}


int main(void)
{
  system_init();
  init_LEDs();
  init_USART();
  init_i2c();
  
  
	IMU.resetMPU9250(); // Reset registers to default in preparation for device calibration
	IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
	IMU.initMPU9250();
  IMU.deltat = 0.02;
  cout=0;
  while(1) TestImu();
}