#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

/**************************************************************************************/
// define bootloader commands

#define ACK 				(uint8_t)0x79
#define NACK 				(uint8_t)0x1F
#define BOOTLOADER 			(uint8_t)0x7F
#define GET 				(uint8_t)0x00
#define GETVER_PROTSTATUS 	(uint8_t)0x01
#define GETID 				(uint8_t)0x02
#define READ_MEMORY 		(uint8_t)0x11
#define GO 					(uint8_t)0x21
#define WRITE_MEMORY 		(uint8_t)0x31
#define ERASE 				(uint8_t)0x43
#define EXTENDED_ERASE 		(uint8_t)0x44
#define WRITE_PROTECT 		(uint8_t)0x63
#define WRITE_UNPROTECT 	(uint8_t)0x73
#define READOUT_PROTECT 	(uint8_t)0x82
#define READOUT_UNPROTECT 	(uint8_t)0x92

/**************************************************************************************/
void RCC_Configuration(void)
{
  /* --------------------------- System Clocks Configuration -----------------*/
  /* USART2 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  /* GPIOA clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  /* GPIOC clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}

/**************************************************************************************/

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect USART pins to AF */
  //GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);  // USART2_TX
  //GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);  // USART2_RX
}

/**************************************************************************************/

void USART1_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure);

  USART_Cmd(USART1, ENABLE);
}
void USART1_Configuration2(void)
{
    USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_Even;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure);

  USART_Cmd(USART1, ENABLE);
}

void Ledinit(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  /* Configure led as alternate function push-pull */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void USART_Send(USART_TypeDef* USARTx, uint8_t Data)
{
	USART_SendData(USARTx,(uint16_t)Data);
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
			{
			}
}

uint16_t USART_Receive(USART_TypeDef* USARTx)
{
	while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET)
		{
		}
	return USART_ReceiveData(USARTx);
}

void Delay(uint32_t nCount)
{
  while(nCount--)
  {
  }
}

int main(void)
{
	Ledinit();

	RCC_Configuration();
	GPIO_Configuration();
	USART1_Configuration2();
	GPIO_SetBits(GPIOC,GPIO_Pin_8);
	  uint16_t ch;

	  ch = USART_Receive(USART1);

	  USART_Send(USART1,ch);

	  GPIO_SetBits(GPIOC,GPIO_Pin_9);

    while(1)
    {
    	ch = USART_Receive(USART1);
    	USART_Send(USART1,ch);
    }
}
