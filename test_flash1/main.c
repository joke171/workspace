/**
  ******************************************************************************
  * @file    main.c
  * @author  Marek Barina
  * @date    23-July-2013
  * @brief   Flashing from stm32f407 to stm32f1xx
  ******************************************************************************
  * @attention
  * Při inicializaci USARTu na stmf407 deska vyšle pulz, který způsobí chybu v nastavení Baudrate bootloaderu.
  * Proto musí být při restartu stm32f407 desky odpojeny nebo musí být u druhé desky držen reset.
  *
  * Flashování začína po stisku USER BUTTON. <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  *
  * LEDs:
  * GREEN 	- active
  * RED 	- error
  * BLUE 	- transmitting
  * ORANGE 	- complete
  ******************************************************************************
  */
#include <stdio.h>
#include <stdlib.h>

#include "stm32f4_discovery.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "LED.h"

/**************************************************************************************/
// define device flash (important just for some commands)
#define STM32F10X_MD_VL
#if defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)
 #define PAGE_SIZE                         (0x400)    /* 1 Kbyte */
 #define FLASH_SIZE                        (0x20000)  /* 128 KBytes */
#elif defined STM32F10X_CL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x40000)  /* 256 KBytes */
#elif defined STM32F10X_HD || defined (STM32F10X_HD_VL)
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x80000)  /* 512 KBytes */
#elif defined STM32F10X_XL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x100000) /* 1 MByte */
#else
 #error "Please select first the STM32 device to be used "
#endif
/**************************************************************************************/
// define bootloader commands

#define ACK 				(uint8_t)0x79	//01111001
#define NACK 				(uint8_t)0x1F	//00011111
#define BOOTLOADER 			(uint8_t)0x7F	//01111111
#define GET 				(uint8_t)0x00	//00000000
#define GETVER_PROTSTATUS 	(uint8_t)0x01	//00000001
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
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}
/**************************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect USART pins to AF */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);  // USART2_TX
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);  // USART2_RX
}
/**************************************************************************************/
void USART2_Configuration(void)
{
	  USART_InitTypeDef USART_InitStructure;
	  USART_ClockInitTypeDef USART_ClockInitStruct;

	  USART_InitStructure.USART_BaudRate = 115200;
	  USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_Even;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure);

  USART_Cmd(USART2, ENABLE);
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
	return (uint16_t)((USART_ReceiveData(USARTx)) & (uint16_t)0xFF);
}

void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

void W8_for_push_USER_BUTTON(void){
	while(GPIO_ReadInputDataBit(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN) == 0);
	GPIO_ResetBits(USER_BUTTON_GPIO_PORT,USER_BUTTON_PIN);
}

void inf_loop(void){
	while(1);
}

void is_ACK(uint16_t byte)
{
	if(byte != ACK)
		{
			STM_EVAL_LEDOn(LED5); // red LED error
			inf_loop();
		}
}

void write_memory(uint32_t address, uint16_t size, unsigned char* data)
{
	 unsigned char buf_addr[4];
	 uint16_t checksum;

	//////// write memory command
	USART_Send(USART2,WRITE_MEMORY);
	USART_Send(USART2,WRITE_MEMORY ^ 0xFF);
	is_ACK( USART_Receive(USART2) );
	//////// send address
	buf_addr[0] = address >> 24;
	buf_addr[1] = address >> 16;
	buf_addr[2] = address >> 8;
	buf_addr[3] = address;
	USART_Send(USART2,buf_addr[0]);
	USART_Send(USART2,buf_addr[1]);
	USART_Send(USART2,buf_addr[2]);
	USART_Send(USART2,buf_addr[3]);
	USART_Send(USART2,buf_addr[0]^buf_addr[1]^buf_addr[2]^buf_addr[3]);
	is_ACK( USART_Receive(USART2) );

	//blink blue LED
	STM_EVAL_LEDOn(LED6);
	Delay(1000000);
	STM_EVAL_LEDOff(LED6);

	//////// send data
	USART_Send(USART2,size - 1); // pocet nasledujicich bytu -1 (podle protokolu)
	checksum = size - 1;
	while(size > 0)
	{
		USART_Send(USART2,*data);
		checksum ^= *data;
		data++;
		size--;
	}
	USART_Send(USART2,checksum);
	is_ACK( USART_Receive(USART2) );
	//blink blue LED
	STM_EVAL_LEDOn(LED6);
	Delay(100000);
	STM_EVAL_LEDOff(LED6);
}

void full_erase(void)
{
	//////// erase command
	USART_Send(USART2,ERASE);
	USART_Send(USART2,ERASE ^ 0xFF);
	is_ACK( USART_Receive(USART2) );
	//////// full-erase command
	USART_Send(USART2,0xFF);
	USART_Send(USART2,0x00);
	is_ACK( USART_Receive(USART2) );
}
void remove_protection(void)
{
	//////// write unprotect command
	USART_Send(USART2,WRITE_UNPROTECT);
	USART_Send(USART2,WRITE_UNPROTECT ^ 0xFF);
	is_ACK( USART_Receive(USART2) );
	is_ACK( USART_Receive(USART2) );
	//after this command a system reset is called
	Delay(1000000);
	USART_Send(USART2,BOOTLOADER);	// paket pro nastavení baudrate
	is_ACK( USART_Receive(USART2) );

	STM_EVAL_LEDOn(LED6);
	Delay(100000);
	STM_EVAL_LEDOff(LED6);
}

int main(void)
{
	u32 address = 0x08000000;
	u16 data_size = LED_bin_size;
	unsigned char* uk_data = LED_bin;
	u8 GETID_Buffer[4];
	u8 RxCounter = 0;
	u16 byte;

	////////////////////// init and conf
	SystemInit();
	SystemCoreClockUpdate();
	STM_EVAL_LEDInit(LED3); //completed ORANGE
	STM_EVAL_LEDInit(LED4);	//start GREEN
	STM_EVAL_LEDInit(LED5);	//error RED
	STM_EVAL_LEDInit(LED6);	//transmitting BLUE
	STM_EVAL_PBInit(BUTTON_USER,BUTTON_MODE_GPIO);

	RCC_Configuration();

	GPIO_Configuration();

	USART2_Configuration();

	Delay(1000);
	STM_EVAL_LEDOn(LED4); //green
	/////////////////////////////////////


	W8_for_push_USER_BUTTON();

	USART_Send(USART2,BOOTLOADER);	// paket pro nastavení baudrate
	is_ACK( USART_Receive(USART2) );

	STM_EVAL_LEDOn(LED6);
	Delay(1000000);
	STM_EVAL_LEDOff(LED6);

	/* get PID of device (not needed in this case)
	W8_for_push_USER_BUTTON();

	USART_Send(USART2,GETID);
	USART_Send(USART2,GETID ^ 0xFF);
	is_ACK( USART_Receive(USART2) );
	RxCounter = 0;
	do{
			byte = USART_Receive(USART2);
			GETID_Buffer[RxCounter] = byte;
			RxCounter++;
	}while(byte != ACK);
	*/

	//////// remove write protection
	remove_protection();
	//////// full-chip erase
	full_erase();
	//////// write memory

	while(data_size > 0)
	{
		if(data_size < 256){
			//uk_data = uk_data + data_size - 1; // -1 (preteceni) chceme posledni prvek pole
			write_memory(address,data_size,uk_data);
			data_size = 0;
		}
		else{
		write_memory(address,256,uk_data);
		data_size -= 256;
		address += 256;
		uk_data += 256;
		}
	}

	STM_EVAL_LEDOn(LED3); // completed orange LED

	while(1); // Don't want to exit
}
