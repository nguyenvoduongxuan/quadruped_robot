/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.8.1
  * @date    27-January-2022
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup Template_Project
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void TR_Serial(void *pvParameters);
static void UpdatePWM(void *pvParameters);
static void ReadPS2(void *pvParameters);
static void ReadSensor(void *pvParameters);
static void RW_SRAM(void *pvParameters);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
xTaskHandle Task_TR_Serial;
xTaskHandle Task_UpdatePWM;
xTaskHandle Task_ReadPS2;
xTaskHandle Task_ReadSensor;
xTaskHandle Task_RW_SRAM;

int main(void)
{ 
	CLK_Config();
	SRAM_Config();
	DMP_MPU_Config();
	
	xTaskCreate(TR_Serial, (const signed char *) "ReadWrite Serial", configMINIMAL_STACK_SIZE, NULL, 2, &Task_TR_Serial);
	xTaskCreate(UpdatePWM, (const signed char *) "Update PWM", configMINIMAL_STACK_SIZE, NULL, 2, &Task_UpdatePWM);
	xTaskCreate(ReadPS2, (const signed char *) "Read PS2", configMINIMAL_STACK_SIZE, NULL, 2, &Task_ReadPS2);
	xTaskCreate(ReadSensor, (const signed char *) "Read IMU", configMINIMAL_STACK_SIZE, NULL, 2, &Task_ReadSensor);
	xTaskCreate(RW_SRAM, (const signed char *) "ReadnWrite SRAM", configMINIMAL_STACK_SIZE, NULL, 2, &Task_RW_SRAM);
	vTaskStartScheduler();

	while(1)
	{				
	}
}

uint8_t PWM_Sign[RXB_SIZE-2];
uint8_t PWM_Current[RXB_SIZE-2];
uint8_t PWM_End[RXB_SIZE-2];
static void TR_Serial(void *pvParameters)
{
	unsigned int Frequency = 100;
	int i;
	UART_DMA_Config();
	TX_Buffer.Begin_PS2 = 'P';
	TX_Buffer.Begin_IMU = 'I';
	TX_Buffer.End_TX = 'E';
	TX_Buffer.EOL = 0x0A;
	
	for(;;)
	{
		for (i = 0; i < sizeof(TX_Buffer.TX_PS2); i++)
		{
			TX_Buffer.TX_PS2[i] = *(__IO uint32_t *) (BKPSRAM_BASE + TXB_ADDR + i);
		}
		TX_Buffer_Byte[26] = 0;
		for (i = 0; i < 4; i++)
		{
			TX_Buffer.TX_QIMU[i] = *(__IO float *) (BKPSRAM_BASE + TXB_ADDR + 8 + 4 * i);
			if (TX_Buffer.TX_QIMU[i] >= 0.0f) TX_Buffer_Byte[26] &=~(1<<i);
			else TX_Buffer_Byte[26] |= 1<<i;
		}
		struct_to_byte(TX_Buffer_Byte, TX_Buffer);
			
		while(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!= SET) // WAITING FOR RECEIVE DATA
		{
			vTaskSuspend(NULL);
		}
		if (((uint16_t)(RX_Buffer[0]&0x00FF) == 0x00FE) && ((uint16_t)(RX_Buffer[RXB_SIZE - 1]&0x00FF) == 0x00EF))
		{
			for (i = 1; i < RXB_SIZE - 1; i ++)
			{
				PWM_Sign[i-1] = (RX_Buffer[i]&0xFF00) >> 8;
				PWM_End[i-1] = RX_Buffer[i]&0x00FF;
			}
		}
		DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);

		vTaskDelay(Frequency);
	}		
}

static void UpdatePWM(void *pvParameters)
{
	unsigned int Frequency = 100;
	int i;
	uint8_t min[12];
	for (i = 0; i < 12; i=i+3)
	{
		min[i] = 0;
		min[i+1] = 40;
		min[i+2] = 70;

		PWM_Current[i] = 0;
		PWM_Current[i+1] = 40;
		PWM_Current[i+2] = 70;

		PWM_End[i] = 0;
		PWM_End[i+1] = 40;
		PWM_End[i+2] = 70;
	}
	PWM_Config();
	for(;;)
	{
		for (i = 0; i < RXB_SIZE-2; i++)
		{
			PWM_Current[i] = PWM_End[i];
			if (PWM_Current[i] < min[i]) 
			{
				PWM_Current[i] = min[i];
			}
		}
		TIM_OCInitStructure.TIM_Pulse = 120 + (PWM_Current[5] - min[5]) + 12; // 120 - 600
		TIM_OC1Init(TIM2, &TIM_OCInitStructure); // RR3
		TIM_OCInitStructure.TIM_Pulse = 600 - (PWM_Current[1] - min[1]) - 5; // 120 - 600
		TIM_OC2Init(TIM2, &TIM_OCInitStructure); // FR2
		TIM_OCInitStructure.TIM_Pulse = 120 + (PWM_Current[2] - min[2]) + 11; // 120 - 600
		TIM_OC3Init(TIM2, &TIM_OCInitStructure); // FR3 // New Servo
		
		TIM_OCInitStructure.TIM_Pulse = 120 + (PWM_Current[3] - min[3]) + 36; // 120 - 600
		TIM_OC4Init(TIM2, &TIM_OCInitStructure); // RR1
		TIM_OCInitStructure.TIM_Pulse = 600 - (PWM_Current[4] - min[4]) - 47; // 120 - 600
		TIM_OC1Init(TIM4, &TIM_OCInitStructure); // RR2
		TIM_OCInitStructure.TIM_Pulse = 600 - (PWM_Current[0] - min[0]) - 5; // 120 - 600
		TIM_OC2Init(TIM4, &TIM_OCInitStructure); // FR1
		
		TIM_OCInitStructure.TIM_Pulse = 600 - (PWM_Current[6] - min[6]) - 48; // 120 - 600
		TIM_OC3Init(TIM4, &TIM_OCInitStructure); // RL1
		TIM_OCInitStructure.TIM_Pulse = 120 + (PWM_Current[7] - min[7]) + 42; // 120 - 600
		TIM_OC4Init(TIM4, &TIM_OCInitStructure); // RL2
		TIM_OCInitStructure.TIM_Pulse = 120 + (PWM_Current[9] - min[9]) + 13; // 120 - 600
		TIM_OC1Init(TIM3, &TIM_OCInitStructure); // FL1
		
		TIM_OCInitStructure.TIM_Pulse = 600 - (PWM_Current[8] - min[8]) - 20; // 120 - 600
		TIM_OC2Init(TIM3, &TIM_OCInitStructure); // RL3
		TIM_OCInitStructure.TIM_Pulse = 120 + (PWM_Current[10] - min[10]) + 27; // 120 - 600
		TIM_OC3Init(TIM3, &TIM_OCInitStructure); // FL2
		TIM_OCInitStructure.TIM_Pulse = 600 - (PWM_Current[11] - min[11]) - 2; // 120 - 600
		TIM_OC4Init(TIM3, &TIM_OCInitStructure); // FL3 // New Servo
		
		vTaskResume(Task_TR_Serial);
		vTaskDelay(Frequency);
	}
}

uint16_t Digital_PS2;
uint8_t Analog_PS2[4];
static void ReadPS2(void *pvParameters)
{
	unsigned int Frequency = 100;
	PS2_Pin_Config(GPIOA, GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7);
	for(;;)
	{
		PS2_readGamePad();
		if (Digital())
		{
			Digital_PS2 = Digital();
		}
		
		Analog_PS2[0] = Analog(PSS_LX);
		Analog_PS2[1] = Analog(PSS_LY);
		Analog_PS2[2] = Analog(PSS_RX);
		Analog_PS2[3] = Analog(PSS_RY);

		vTaskResume(Task_TR_Serial);
		vTaskDelay(Frequency);
	}
}

static void ReadSensor(void *pvParameters)
{
	unsigned int Frequency = 100;
	
	for(;;)
	{
		mpuIntStatus = MPUgetIntStatus();
    fifoCount = MPUgetFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
		{
			MPUresetFIFO();
    } 
		else if (mpuIntStatus & 0x02)
		{
			while (fifoCount < packetSize) fifoCount = MPUgetFIFOCount();
			MPUgetFIFOBytes(fifoBuffer, packetSize);
			fifoCount -= packetSize;
		
			MPUdmpGetQuaternion(&q, fifoBuffer);
			MPUdmpGetEuler(euler, &q);			
		}
		GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
		vTaskResume(Task_TR_Serial);

		vTaskDelay(Frequency);
	}
}

static void RW_SRAM(void *pvParameters)
{
	unsigned int Frequency = 100;
	uint8_t i;
	for(;;)
	{ 
		// Max: 0xF00 bytes
		// Write to Backup SRAM with 32-Bit Data
		// Write Transfer Data to SRAM
		*(__IO uint32_t *) (BKPSRAM_BASE + TXB_ADDR + 0) = ((Digital_PS2 & 0xFF) >> 4) + 0x60;
		*(__IO uint32_t *) (BKPSRAM_BASE + TXB_ADDR + 1) = (Digital_PS2 >> 8) & 0xFF;
		*(__IO uint32_t *) (BKPSRAM_BASE + TXB_ADDR + 2) = 0x0;
    *(__IO uint32_t *) (BKPSRAM_BASE + TXB_ADDR + 3) = 0x0;

		for (i = 0; i < 4; i++) 
		{
			*(__IO uint32_t *) (BKPSRAM_BASE + TXB_ADDR + 4 + i) = Analog_PS2[i] & 0xFF;
		}
		
		*(__IO float *) (BKPSRAM_BASE + TXB_ADDR + 8 + 4 * 0) = q.x;		
		*(__IO float *) (BKPSRAM_BASE + TXB_ADDR + 8 + 4 * 1) = q.y;			
		*(__IO float *) (BKPSRAM_BASE + TXB_ADDR + 8 + 4 * 2) = q.z;			
		*(__IO float *) (BKPSRAM_BASE + TXB_ADDR + 8 + 4 * 3) = q.w;			
		
		// Write Receive Data to SRAM
		if (((uint16_t)(RX_Buffer[0]&0x00FF) == 0x00FE) && ((uint16_t)(RX_Buffer[RXB_SIZE - 1]&0x00FF) == 0xEF))
		{
			for (i = 1; i < RXB_SIZE-1; i++) 
			{
				*(__IO uint32_t *) (BKPSRAM_BASE + RXB_ADDR + (i - 1)*sizeof(uint16_t)) = RX_Buffer[i];	
			}		
		}
		vTaskResume(Task_TR_Serial);
		vTaskDelay(Frequency);
	}
}

#ifdef  USE_FULL_ASSERT
 
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


