
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_flash.h"


uint8_t ch;


#define  RCC_APB2Periph_GPIOx     RCC_APB2Periph_GPIOA
#define  GPIO_Pin_Tx2               GPIO_Pin_2	 // PA2 sortie
#define  GPIO_Pin_Rx2               GPIO_Pin_3	   // PA3 entree
typedef enum { FAILED = 0, PASSED = 1} TestStatus;

#define countof(a)   (sizeof(a) / sizeof(*(a)))
#define RxBufferSize   countof(TxBuffer)


GPIO_InitTypeDef GPIO_InitStructure;
TestStatus TransferStatus = PASSED;
ErrorStatus HSEStartUpStatus;

/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;
u8 TxBuffer[] = "hello world";
u8 RxBuffer[RxBufferSize];
u8 TxCounter = 0, RxCounter = 0;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
TestStatus Buffercmp(u8* pBuffer1, u8* pBuffer2, u16 BufferLength);


void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure USART2 Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Tx2;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART2 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Rx2;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  // configurer Rx USART2 en entree flottante


  /* Configure led as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}


TestStatus Buffercmp(u8* pBuffer1, u8* pBuffer2, u16 BufferLength)
{
  u8 i = 0;
  pBuffer2++;
  while(BufferLength--)
  {
    if(*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }
    pBuffer1++;
    pBuffer2++;
  }


  return PASSED;
}

int main(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 // autorise GPIO
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);


  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	  // autorise USART2
 SystemInit();
 GPIO_Configuration();	 // configuration GPIO
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure USART2 */
  USART_Init(USART2, &USART_InitStructure);


  /* Enable the USART2 */
  USART_Cmd(USART2, ENABLE);

  while(RxCounter < RxBufferSize)
  {

	while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET)
    {
    }

    /* Store the received byte in RxBuffer */
    RxBuffer[RxCounter++] = (uint8_t)USART_ReceiveData(USART2);

    if (RxBuffer[RxCounter -1]=='w'){GPIO_SetBits(GPIOC,GPIO_Pin_8);}
  }
  /*RxCounter = 0;
  while(RxCounter < RxBufferSize)
  {
	uint32_t FLASH_Pot_StartingAddress=((uint32_t)0x08000ECC);

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR );
    FLASH_ProgramHalfWord(FLASH_Pot_StartingAddress, (uint16_t)RxBuffer[RxCounter++]);
    FLASH_Pot_StartingAddress=FLASH_Pot_StartingAddress+4;
    FLASH_Lock();
  }*/

  /* Check the received data with the send ones */
  TransferStatus = Buffercmp(TxBuffer, RxBuffer, RxBufferSize);
  if (TransferStatus == PASSED)
   {
	  GPIO_SetBits(GPIOC,GPIO_Pin_9);
   }
}


//=============================================================================
// End of file
//=============================================================================
