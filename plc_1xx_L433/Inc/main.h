/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "math.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define RS485_USART2_TX_Pin GPIO_PIN_2
#define RS485_USART2_TX_GPIO_Port GPIOA
#define RS485_USART2_RX_Pin GPIO_PIN_3
#define RS485_USART2_RX_GPIO_Port GPIOA
#define ICP_ADC1_IN11_Pin GPIO_PIN_6
#define ICP_ADC1_IN11_GPIO_Port GPIOA
#define _4_20_ADC1_IN12_Pin GPIO_PIN_7
#define _4_20_ADC1_IN12_GPIO_Port GPIOA
#define POWER_SUPPLY_ADC1_IN15_Pin GPIO_PIN_0
#define POWER_SUPPLY_ADC1_IN15_GPIO_Port GPIOB
#define Button_Pin GPIO_PIN_8
#define Button_GPIO_Port GPIOB
#define Discrete_in_Pin GPIO_PIN_9
#define Discrete_in_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define ADC_CHANNEL_NUMBER 2
#define ADC_BUFFER_SIZE 3200
#define RAW_ADC_BUFFER_SIZE (ADC_CHANNEL_NUMBER * ADC_BUFFER_SIZE)
#define QUEUE_LENGHT 16 //2 сек.


#define FILTER_MODE filter_mode_icp

#define COEF_TRANSFORM_icp (range_icp / (4096 - 2170))
#define COEF_TRANSFORM_4_20 (range_420 / 4096)
#define COEF_TRANSFORM_SUPPLY (24.0 / 2900)

#define REG_COUNT 155  
#define PAGE 100  
#define PAGE_ADDR (0x8000000 + (PAGE * 2048))

#define SLAVE_ADR slave_adr
#define START_REG_ADR_MB_MASTER 148

#define HART_UART_Tx_TIMEOUT 5000

void convert_float_and_swap(float32_t float_in, uint16_t* int_out);
float32_t convert_hex_to_float(uint16_t* in, uint8_t index);

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
