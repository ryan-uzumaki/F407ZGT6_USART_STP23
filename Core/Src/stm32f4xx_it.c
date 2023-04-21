/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern LiDARFrameTypeDef Pack_Data;//�״���յ����ݴ������������֮��
extern uint16_t receive_cnt;
extern uint16_t distance;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	static uint8_t state = 0;//״̬λ	
	static uint8_t crc = 0;//У���
	static uint8_t cnt = 0;//����һ֡12����ļ���
	uint8_t temp_data;
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET) //���յ�����
  {
    temp_data = (uint8_t)(huart1.Instance->DR & 0xFF);

    //if(state == 1)receive_cnt = temp_data;
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
		if (state > 5)
		{
			if(state < 42)
			{
				if(state%3 == 0)//һ֡�����е����Ϊ6,9.....39�����ݣ�����ֵ��8λ
				{
					Pack_Data.point[cnt].distance = (uint16_t)temp_data;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
				else if(state%3 == 1)//һ֡�����е����Ϊ7,10.....40�����ݣ�����ֵ��8λ
				{
					Pack_Data.point[cnt].distance = ((uint16_t)temp_data<<8)+Pack_Data.point[cnt].distance;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
				else//һ֡�����е����Ϊ8,11.....41�����ݣ����Ŷ�
				{
					Pack_Data.point[cnt].intensity = temp_data;
					cnt++;	
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
			}
			else 
			{
				switch(state)
				{
					case 42:
						Pack_Data.end_angle = (uint16_t)temp_data;//�����Ƕȵ�8λ
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 43:
						Pack_Data.end_angle = ((uint16_t)temp_data<<8)+Pack_Data.end_angle;//�����Ƕȸ�8λ
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 44:
						Pack_Data.timestamp = (uint16_t)temp_data;//ʱ�����8λ
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 45:
						Pack_Data.timestamp = ((uint16_t)temp_data<<8)+Pack_Data.timestamp;//ʱ�����8λ
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 46:
						Pack_Data.crc8 = temp_data;//�״ﴫ����У���
						if(Pack_Data.crc8 == crc)//У����ȷ
						{
							data_process();//���յ�һ֡��У����ȷ���Խ������ݴ���
							receive_cnt++;//������յ���ȷ���ݵĴ���
						}
						else
						{
						//У�鲻��ȷ
						}
							//memset(&Pack_Data,0,sizeof(Pack_Data)*);//����
						crc = 0;
						state = 0;
						cnt = 0;//��λ
					default: break;
				}
			}
		}
		else 
		{
			switch(state)
			{
				case 0:
					if(temp_data == HEADER)//ͷ�̶�
					{
						Pack_Data.header = temp_data;
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];//��ʼ����У��
					} else state = 0,crc = 0;
					break;
				case 1:
					//receive_cnt = temp_data;
					if(temp_data == VERLEN)//�����ĵ�����Ŀǰ�̶�
					{
						//receive_cnt++;
						Pack_Data.ver_len = temp_data;
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
					} else state = 0,crc = 0;
					break;
				case 2:
					Pack_Data.temperature = (uint16_t)temp_data;//�¶ȵ�8λ��һ��16λADC��0--4096��������
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 3:
					Pack_Data.temperature = ((uint16_t)temp_data<<8)+Pack_Data.temperature;//�¶ȸ�8λ
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 4:
					Pack_Data.start_angle = (uint16_t)temp_data;//��ʼ�Ƕȵ�8λ���Ŵ���100��
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 5:
					Pack_Data.start_angle = ((uint16_t)temp_data<<8)+Pack_Data.start_angle;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				default: break;
			}
		}
	}				
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
