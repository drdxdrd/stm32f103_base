#include <UART.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <math.h>

void init_USART(void)
{
  // only tx at PB6
  //rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);
	// Setup GPIO pins for USART transmit.  
  //gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
	//gpio_set_af(GPIOA, GPIO_AF0, GPIO6);

  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO2);
	// Setup USART parameters. 
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_enable(USART1);
}


uint8_t num2sym(uint8_t symb)
{
  symb &= 0x0F;
  if (symb<10) symb+='0';
  else         symb+='A'-10; 
  return symb;
}  

void tx_byte_UART(uint8_t c)
{
  tx_UART('0');
  tx_UART('x');
  tx_UART(num2sym(c>>4));
  tx_UART(num2sym(c));  
}

void tx_word_UART(uint16_t c)
{
  tx_UART('0');
  tx_UART('x');
  tx_UART(num2sym(c>>12));
  tx_UART(num2sym(c>>8));
  tx_UART(num2sym(c>>4));   
  tx_UART(num2sym(c));  
}

void tx_UART(uint8_t c)
{
  while ((USART_ISR(USART1) & USART_ISR_TXE)==0); // TX empty
  usart_send(USART1, c);  
}

void buff_UART(uint8_t *pointer, uint8_t length)
{
  do
  {
    uint8_t data= *pointer;
    tx_UART(num2sym(data>>4));
    tx_UART(num2sym(data));      
    tx_UART(' ');
    pointer++;
  } while(--length);
  tx_UART(0xa);
  tx_UART(0xd);
}



void int2s(uint8_t *String, int16_t Data)
{
  ldiv_t DivNum; 
  uint8_t * Pointer;
  bool sign = false;
  if(Data<0) 
  {
    sign = true;
    Data = -Data;
  }
  
  Pointer = String; 
  for(uint8_t i=0; i<5; i++) *Pointer++ = ' ';
  *Pointer = 0;
  Pointer = String+4; 
  for(uint8_t i=0; i<4; i++)
  {
    if (Data<10)
    {
      *Pointer--= (char)Data+'0'; 
      break;
    }  
    else
    {  
      DivNum = ldiv(Data, 10);    
      *Pointer-- = (char)DivNum.rem+'0'; 
      Data = DivNum.quot;
    }       
  }  
  if (sign) *Pointer = '-';
}  

void str2UART(uint8_t *pointer)
{
  uint8_t maxl=16;
  do
  {
    uint8_t data= *pointer;
    if (data==0) break;
    tx_UART(data);
    pointer++;
  } while(--maxl);
}

void int2UART(uint16_t Number)
{
  uint8_t tx_buff[16];
  int2s(tx_buff, Number);
  str2UART(tx_buff);

}