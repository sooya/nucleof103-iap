/*!
  * @file    arch/main.c
  * @brief   string Control API
  */

#include "common.h"

int version = 0;

void BSP_Init(void);
int app_start(void);

void __attribute__((weak)) osa_mdelay(unsigned int msec)
{
	uint32_t temp;
	SysTick->LOAD=(uint32_t)msec*(HSE_VALUE/1000);
	SysTick->VAL =0x00;		// clear Count flag
	SysTick->CTRL=0x01;
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));	// wait Count flag set
	SysTick->CTRL=0x00;
	SysTick->VAL =0X00;
}

void __attribute__((weak)) osa_udelay(unsigned int usec)
{
	uint32_t temp;
	SysTick->LOAD=(uint32_t)usec*(HSE_VALUE/1000000);
	SysTick->VAL =0x00;		// clear Count flag
	SysTick->CTRL=0x01;
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));	// wait Count flag set
	SysTick->CTRL=0x00;
	SysTick->VAL =0X00;
}

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* PA.05 as LED */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void LED_Off(void)
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_5, 0);
}

void LED_On(void)
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_5, 1);
}

static void IAP_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* Enable GPIOA and USART1 clock  */
	RCC_APB2PeriphClockCmd(DBG_COM_TX_GPIO_CLK
                           |DBG_COM_RX_GPIO_CLK | PUSH_BUTTON_GPIO_CLK
                           | RCC_APB2Periph_AFIO
                           , ENABLE );

	RCC_APB1PeriphClockCmd(DBG_COM_CLK , ENABLE);

	/* Push Button Configure PB.00 as input */
	GPIO_InitStructure.GPIO_Pin = PUSH_BUTTON_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(PUSH_BUTTON_GPIO_PORT, &GPIO_InitStructure);

	/* Configure the GPIO ports( USART1 Transmit and Receive Lines) */
	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = DBG_COM_TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DBG_COM_TX_GPIO_PORT, &GPIO_InitStructure);

	/* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = DBG_COM_RX_PIN;
	GPIO_Init(DBG_COM_RX_GPIO_PORT, &GPIO_InitStructure);

	/*-- USART configured as follow:------------------------
	  - Word Length = 8 Bits
	  - 1 Stop Bit
	  - No parity
	  - BaudRate = 115200 baud
	  - Receive and transmit enabled
	  -------------------------------------------------------*/

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(DBG_COM, &USART_InitStructure);

	/* Enable the DBG_COM */
	USART_Cmd(DBG_COM, ENABLE);

	LED_Init();	// debug LED
	LED_On();
}

static uint8_t Push_Button_Read (void)
{
	/* Return the status of the PB.9 pin = "Key" button on the STM3210B-EVAL board */
	return GPIO_ReadInputDataBit(PUSH_BUTTON_GPIO_PORT, PUSH_BUTTON_PIN);
}

typedef  void (*pFunction)(void);

extern pFunction Jump_To_Application;
extern uint32_t JumpAddress;

int __attribute__((weak)) app_start(void)
{
	SerialPutString("\r\n=              NucleoF103 ::: ");
	if (Push_Button_Read() == 0x00)
	{
		FLASH_Unlock();
		SerialPutString("IAP                 =\r\n\r\n");
		Main_Menu ();
	}
	else
	{
		/* Jump to user application */
		LED_Off();
		SerialPutString((u8*)"Start App (0x80004000)                 =\r\n\r\n");
		JumpAddress = *(vu32*) (ApplicationAddress + 4);
		Jump_To_Application = (pFunction) JumpAddress;
		/* Initialize user application's Stack Pointer */
		__set_MSP(*(vu32*) ApplicationAddress);
		Jump_To_Application();
	}
	while(1);
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
#ifdef __NO_SYSTEM_INIT
	SystemInit();
#endif
	IAP_Init();

	/* Configure the NVIC Preemption Priority Bits */
	app_start();
}
