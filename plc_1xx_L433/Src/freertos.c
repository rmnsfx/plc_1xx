/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "arm_math.h"
#include "math.h"
#include <stdint.h>
#include "Task_manager.h"
#include "main.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
osThreadId myTask06Handle;

/* USER CODE BEGIN Variables */



uint16_t raw_adc_value[RAW_ADC_BUFFER_SIZE];
float32_t float_adc_value_1[ADC_BUFFER_SIZE];
float32_t float_adc_value_2[ADC_BUFFER_SIZE];
//float32_t filter_value[ADC_BUFFER_SIZE];
//float32_t integr_low_filter_value[ADC_BUFFER_SIZE];
float32_t rms_out = 0.0;

float32_t all_rms = 0;
float32_t all_rms2 = 0;
uint8_t queue_count;
float32_t qrms;
float32_t qrms_array[QUEUE_LENGHT];
uint64_t xTimeBefore, xTotalTimeSuspended;

//float32_t source_integral[ADC_BUFFER_SIZE];
//float32_t destination_integral[ADC_BUFFER_SIZE];
//float32_t filter_destination_integral[ADC_BUFFER_SIZE];

xQueueHandle queue;

xSemaphoreHandle Semaphore1, Semaphore2, Semaphore3, Semaphore4, Semaphore5;

arm_biquad_casd_df1_inst_f32 filter_instance_float;
float32_t pStates_float[8];

arm_biquad_casd_df1_inst_f32 filter_instance_lowpass;
float32_t pStates_lowpass[8];
		
float32_t settings[REG_COUNT];



	
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void GetADC_Task(void const * argument);
void Filter_Task(void const * argument);
void RMS_Task(void const * argument);
void AverageBufferQueue_Task(void const * argument);
void Integrate_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void FilterInit(void);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	
	queue = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));	
	
	vSemaphoreCreateBinary(Semaphore1);
	vSemaphoreCreateBinary(Semaphore2);
	vSemaphoreCreateBinary(Semaphore3);
	vSemaphoreCreateBinary(Semaphore4);
	vSemaphoreCreateBinary(Semaphore5);
	
	
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, GetADC_Task, osPriorityNormal, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, Filter_Task, osPriorityNormal, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, RMS_Task, osPriorityNormal, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, AverageBufferQueue_Task, osPriorityNormal, 0, 128);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* definition and creation of myTask06 */
  osThreadDef(myTask06, Integrate_Task, osPriorityNormal, 0, 128);
  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	
	Task_manager_Init();
	
  for(;;)
  {
		Task_manager_LoadCPU();
		
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* GetADC_Task function */
void GetADC_Task(void const * argument)
{
  /* USER CODE BEGIN GetADC_Task */
  /* Infinite loop */
  for(;;)
  {
    xSemaphoreTake( Semaphore1, portMAX_DELAY );

		for (uint16_t i=0; i<ADC_BUFFER_SIZE-1; i++)
		{
				float_adc_value_1[i] = (float32_t) raw_adc_value[i*2];				
				float_adc_value_2[i] = (float32_t) raw_adc_value[i*2+1];
		}
		
		xSemaphoreGive( Semaphore2 );
  }
  /* USER CODE END GetADC_Task */
}

/* Filter_Task function */
void Filter_Task(void const * argument)
{
  /* USER CODE BEGIN Filter_Task */
	FilterInit();
	
  /* Infinite loop */
  for(;;)
  {
    xSemaphoreTake( Semaphore2, portMAX_DELAY );
		
		arm_biquad_cascade_df1_f32(&filter_instance_float, (float32_t*) &float_adc_value_1[0], (float32_t*) &float_adc_value_1[0], ADC_BUFFER_SIZE);
		arm_biquad_cascade_df1_f32(&filter_instance_float, (float32_t*) &float_adc_value_2[0], (float32_t*) &float_adc_value_2[0], ADC_BUFFER_SIZE);
				
		xSemaphoreGive( Semaphore3 );
		//xSemaphoreGive( Semaphore5 );
  }
  /* USER CODE END Filter_Task */
}

/* RMS_Task function */
void RMS_Task(void const * argument)
{
  /* USER CODE BEGIN RMS_Task */
  /* Infinite loop */
  for(;;)
  {
    xSemaphoreTake( Semaphore3, portMAX_DELAY );
			
		arm_rms_f32( (float32_t*)&float_adc_value_1[0], ADC_BUFFER_SIZE, (float32_t*)&all_rms );				
		arm_rms_f32( (float32_t*)&float_adc_value_2[0], ADC_BUFFER_SIZE, (float32_t*)&all_rms2 );				
				
		xQueueSend(queue, (void*)&all_rms, 0);				
		
								
		all_rms = 0;		
		

		xSemaphoreGive( Semaphore4 );
  }
  /* USER CODE END RMS_Task */
}

/* AverageBufferQueue_Task function */
void AverageBufferQueue_Task(void const * argument)
{
  /* USER CODE BEGIN AverageBufferQueue_Task */
  /* Infinite loop */
  for(;;)
  {			
		
   		xSemaphoreTake( Semaphore4, portMAX_DELAY );
			
			queue_count = uxQueueMessagesWaiting(queue);		
			
			if (queue_count == QUEUE_LENGHT)
			{			
					rms_out = 0.0;		
								
					for (uint16_t i=0; i<QUEUE_LENGHT; i++)
					{
							xQueueReceive(queue, (void *) &qrms, 0);		
							qrms_array[i] = qrms;												
					}
					
					arm_rms_f32((float32_t*) &qrms_array, QUEUE_LENGHT, (float32_t*)&rms_out);
					
					xTotalTimeSuspended = xTaskGetTickCount() - xTimeBefore;
					xTimeBefore = xTaskGetTickCount();	
			}
  }
  /* USER CODE END AverageBufferQueue_Task */
}

/* Integrate_Task function */
void Integrate_Task(void const * argument)
{
  /* USER CODE BEGIN Integrate_Task */
	
	static float32_t coef_lowpass_gain[] = { 1*0.9972270499044702, -2*0.9972270499044702, 1*0.9972270499044702, 1.9944464105419268, -0.99446178907595384};				
	arm_biquad_cascade_df1_init_f32(&filter_instance_lowpass, 2, (float32_t *) &coef_lowpass_gain[0], &pStates_lowpass[0]);	
	
	
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore5, portMAX_DELAY );
						
		//for (uint16_t i=1; i<ADC_BUFFER_SIZE; i++) filter_value[i] = filter_value[i]+ filter_value[i-1];	
		
		//arm_biquad_cascade_df1_f32(&filter_instance_lowpass, (float32_t*) &filter_value[0], (float32_t*) &integr_low_filter_value[0], ADC_BUFFER_SIZE);
		
    
  }
  /* USER CODE END Integrate_Task */
}

/* USER CODE BEGIN Application */

void FilterInit(void)
{

/////////////////////////////////////////////////////////////////////////////
//SOS Matrix:                                                  
//1  0  -1  1  -1.9722335009416523  0.9726187287542114         
//1  0  -1  1   0.4569532855558438  0.21172935334109441        
//                                                             
//Scale Values:                                                
//0.64137714128839884                                          
//0.64137714128839884

		//float32_t gain = 0.64137714128839884;
	
		//static float32_t coef_f32[] = 
		//{
		//	1, 0, -1, 1.9722335009416523, -0.9726187287542114,         
		//	1, 0, -1, -0.4569532855558438, -0.21172935334109441 
		//};
	

		
		static float32_t coef_f32_gain[] = { 
			
			1 * 0.64137714128839884, 0 * 0.64137714128839884, -1 * 0.64137714128839884, 1.9722335009416523, -0.9726187287542114,         
			1 * 0.64137714128839884, 0 * 0.64137714128839884, -1 * 0.64137714128839884, -0.4569532855558438, -0.21172935334109441 
		};

		arm_biquad_cascade_df1_init_f32(&filter_instance_float, 2, (float32_t *) &coef_f32_gain[0], &pStates_float[0]);	



/////////////////////////////////////////////////////////////////////////////	
//SOS Matrix:                                                  
//1  -2  1  1  -1.9944464105419268  0.99446178907595384        
//																															 
//Scale Values:                                                
//0.9972270499044702      

//		static float32_t coef_lowpass_gain[] = { 1*0.9972270499044702, -2*0.9972270499044702, 1*0.9972270499044702, 1.9944464105419268, -0.99446178907595384};		
//		
//		arm_biquad_cascade_df1_init_f32(&filter_instance_lowpass, 2, (float32_t *) &coef_lowpass_gain[0], &pStates_lowpass[0]);	
		
		
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
