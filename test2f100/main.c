
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"


uint8_t ch;


#define  RCC_APB2Periph_GPIOx     RCC_APB2Periph_GPIOA
#define  GPIO_Pin_Tx1               GPIO_Pin_9	  // PA9  sortie
#define  GPIO_Pin_Rx1               GPIO_Pin_10	   // PA10	entree
#define  GPIO_Pin_Tx2               GPIO_Pin_2	 // PA2 sortie
#define  GPIO_Pin_Rx2               GPIO_Pin_3	   // PA3 entree
typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;


#define countof(a)   (sizeof(a) / sizeof(*(a)))
#define TxBufferSize   countof(TxBuffer)


GPIO_InitTypeDef GPIO_InitStructure;
TestStatus TransferStatus = FAILED;
ErrorStatus HSEStartUpStatus;
/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;
u8 TxBuffer[] = "hello world.c";
u8 RxBuffer[TxBufferSize];
u8 TxCounter = 0, RxCounter = 0;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);  // configuration horloge PortA et horloge Usart
void GPIO_Configuration(void);	// configuration pattes entrees/sorties
void NVIC_Configuration(void);	 // configuration interruption (ici non utilisee)
TestStatus Buffercmp(u8* pBuffer1, u8* pBuffer2, u16 BufferLength);
//u8 index = 0;


void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;


  /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Tx1;
  GPIO_Init(GPIOA, &GPIO_InitStructure);   // Tx USART1 en sortie push pull


  /* Configure USART2 Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Tx2;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Rx1;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  // configurer Rx USART1 en entree flottante

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

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	  // autorise USART1
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	  // autorise USART2
 SystemInit();	// initialiser horloge et PLLMUL systeme et USART	  //
 GPIO_Configuration();	 // configuration GPIO
  USART_InitStructure.USART_BaudRate = 115200;	   // configuration vitesse
  USART_InitStructure.USART_WordLength = USART_WordLength_8b; // configuration longueur mot
  USART_InitStructure.USART_StopBits = USART_StopBits_1;	 // bit de stop
  USART_InitStructure.USART_Parity = USART_Parity_No;	 // bit de parite
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // hardware control
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	// half duplex

  /* Configure USART1 */
  USART_Init(USART1, &USART_InitStructure);	// initialisation registres	internes (SR et BRR) USART1
  /* Configure USART2 */
  USART_Init(USART2, &USART_InitStructure);	 // initialisation registres internes (SR et BRR) USART2

  USART_Cmd(USART1, ENABLE);	// autoriser USART1 a fonctionner


  /* Enable the USART2 */
  USART_Cmd(USART2, ENABLE); 	// autoriser USART2 a fonctionner

while(TxCounter < TxBufferSize)
 {
    /* envoyer un caractere de USART1 a USART2 */
 ch=TxBuffer[TxCounter++];
 if (ch=='c'){GPIO_SetBits(GPIOC,GPIO_Pin_9);}


while (!(USART1->SR & USART_FLAG_TXE));	  // tester si registre a decalge vide (Bit TXE) registre SR
USART1->DR = ch;	  // envoyer caractere pareil que USART_SendData(USART1, TxBuffer[TxCounter++]);
// USART1->DR = (ch1 & (uint16_t)0x01FF);

while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET)
    {
    }


    /* Store the received byte in RxBuffer */
    RxBuffer[RxCounter++] = (USART_ReceiveData(USART2) & 0x7F); // why we must "&" the received value with 0x7F!!!!!
      //if (RxBuffer[RxCounter -1]=='c'){GPIO_SetBits(GPIOC,GPIO_Pin_8);}
  }
  /* Check the received data with the send ones */
   TransferStatus = Buffercmp(TxBuffer, RxBuffer, TxBufferSize);
  if (TransferStatus== PASSED)
   {GPIO_SetBits(GPIOC,GPIO_Pin_8); }
}

// si le contenu des deux buffers de reception et d'emission sont identiques c'est que
//la communication entre port serie 1USART1 et port serie 2 USART1 s'est bien faite: PASS = 1, Failed = 0
//=============================================================================
// End of file
//=============================================================================
