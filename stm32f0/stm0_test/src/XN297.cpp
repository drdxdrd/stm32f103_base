#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/timer.h>
#include <XN297.h>
#include <SPI.h>
#include <system.h>


uint8_t tx_buff[BUFF_LENGTH];
uint8_t rx_buff[BUFF_LENGTH];

void init_XN297(void)
{
  delay_us(180);
  transfer_word_SPI(0x20, 0x0F);   // config wr
  delay_us(180);
  transfer_word_SPI(0x20, 0x09);   // config wr
  tx_buff[0]=0x2A;
  tx_buff[1]=0x26;
  tx_buff[2]=0xA8;
  tx_buff[3]=0x67;    
  tx_buff[4]=0x35;
  tx_buff[5]=0xCC;     
  transfer_buff(tx_buff, rx_buff, 6);
  transfer_word_SPI(0x0A, 0xFF);  // rd addr datapipe answer 26???+pulse
  tx_buff[0]=0x30;
  tx_buff[1]=0x26;
  tx_buff[2]=0xA8;
  tx_buff[3]=0x67;    
  tx_buff[4]=0x35;
  tx_buff[5]=0xCC;  
  transfer_buff(tx_buff, rx_buff, 6);
  transfer_word_SPI(0x10, 0xFF);  // rd addr datapipe answer 26???+pulse
  tx_buff[0]=0x39;
  tx_buff[1]=0x0B;
  tx_buff[2]=0xDF;
  tx_buff[3]=0xC4;    
  tx_buff[4]=0xA7;
  tx_buff[5]=0x03;    
  transfer_buff(tx_buff, rx_buff, 6);
  tx_buff[0]=0x3E;
  tx_buff[1]=0xC9;
  tx_buff[2]=0x9A;
  tx_buff[3]=0xB0;    
  tx_buff[4]=0x61;
  tx_buff[5]=0xBB;  
  tx_buff[6]=0xAB;
  tx_buff[7]=0x9C;   
  transfer_buff(tx_buff, rx_buff, 8);
  transfer_word_SPI(0x1E, 0xFF); 
  tx_buff[0]=0x3F;
  tx_buff[1]=0x4C;
  tx_buff[2]=0x84;
  tx_buff[3]=0x6F;    
  tx_buff[4]=0x9C;
  tx_buff[5]=0x20;    
  transfer_buff(tx_buff, rx_buff, 6);  
  transfer_word_SPI(0x1F, 0xFF); 
  transfer_word_SPI(0x26, 0x07);
  transfer_word_SPI(0x31, 0x0F);  
  transfer_word_SPI(0x24, 0x00);
  transfer_word_SPI(0x27, 0x70);    
  transfer_word_SPI(0x25, 0x2D);  
  transfer_word_SPI(0x23, 0x03);  
  transfer_word_SPI(0x22, 0x01);
  transfer_word_SPI(0x21, 0x00);
  transfer_word_SPI(0xE2, 0x00);   
  transfer_word_SPI(0x70, 0x73); 
  transfer_word_SPI(0x3D, 0x00); 
  transfer_word_SPI(0x3C, 0x00);
  transfer_word_SPI(0x20, 0x0F);  
  delay_us(180);
  transfer_word_SPI(0x00, 0xFF);
  //transfer_word_SPI(0x07, 0xFF);


}

void rx_pack(void)
{
  transfer_word_SPI(0x20, 0x09);  
  for(uint8_t i=0; i<16; i++) tx_buff[i]=0xFF;
  tx_buff[0]=0x61;
  transfer_buff(tx_buff, rx_buff, 16); 
  transfer_word_SPI(0x27, 0x40); 
  transfer_word_SPI(0xE2, 0x00); 
  transfer_word_SPI(0x20, 0x0F); 
}

void bind(void)
{
  void rx_pack(void);
  transfer_word_SPI(0x20, 0x09); 
  transfer_word_SPI(0x25, 0x11); 
  transfer_word_SPI(0x20, 0x0F);   
}
