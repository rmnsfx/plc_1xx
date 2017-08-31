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
#include "adc.h"
#include "usart.h"
#include "dac.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
osThreadId myTask06Handle;
osThreadId myTask07Handle;
osThreadId myTask08Handle;
osThreadId myTask09Handle;
osThreadId myTask10Handle;
osThreadId myTask11Handle;
osThreadId myTask12Handle;

/* USER CODE BEGIN Variables */

xSemaphoreHandle 	Semaphore1, Semaphore2,
									Semaphore_Acceleration, Semaphore_Velocity, Semaphore_Displacement,
									Q_Semaphore_Acceleration, Q_Semaphore_Velocity, Q_Semaphore_Displacement;

//float32_t sinus[ADC_BUFFER_SIZE];
uint16_t raw_adc_value[RAW_ADC_BUFFER_SIZE];
float32_t float_adc_value_ICP[ADC_BUFFER_SIZE];
float32_t float_adc_value_4_20[ADC_BUFFER_SIZE];

float32_t power_supply_voltage = 0.0;
float32_t dac_voltage = 0.0;

float32_t rms_acceleration_icp = 0.0;
float32_t rms_acceleration_4_20 = 0.0;
float32_t rms_velocity_icp = 0.0;
float32_t rms_velocity_4_20 = 0.0;
float32_t rms_displacement_icp = 0.0;
float32_t rms_displacement_4_20 = 0.0;

float32_t max_acceleration_icp = 0.0;
float32_t min_acceleration_4_20 = 0.0;
float32_t max_velocity_icp = 0.0;
float32_t min_velocity_4_20 = 0.0;
float32_t max_displacement_icp = 0.0;
float32_t min_displacement_4_20 = 0.0;

uint64_t xTimeBefore, xTotalTimeSuspended;

float32_t Q_A_rms_array_icp[QUEUE_LENGHT];
float32_t Q_V_rms_array_icp[QUEUE_LENGHT];
float32_t Q_D_rms_array_icp[QUEUE_LENGHT];
float32_t Q_A_rms_array_4_20[QUEUE_LENGHT];
float32_t Q_V_rms_array_4_20[QUEUE_LENGHT];
float32_t Q_D_rms_array_4_20[QUEUE_LENGHT];

xQueueHandle acceleration_queue_icp;
xQueueHandle velocity_queue_icp;
xQueueHandle displacement_queue_icp;
xQueueHandle acceleration_queue_4_20;
xQueueHandle velocity_queue_4_20;
xQueueHandle displacement_queue_4_20;
	
uint8_t queue_count_A_icp;
uint8_t queue_count_A_4_20;
uint8_t queue_count_V_icp;
uint8_t queue_count_V_4_20;
uint8_t queue_count_D_icp;
uint8_t queue_count_D_4_20;

arm_biquad_casd_df1_inst_f32 filter_main_high_icp;
float32_t pStates_main_high_icp[8];

arm_biquad_casd_df1_inst_f32 filter_main_high_4_20;
float32_t pStates_main_high_4_20[8];

arm_biquad_casd_df1_inst_f32 filter_main_low_icp;
float32_t pStates_main_low_icp[8];

arm_biquad_casd_df1_inst_f32 filter_main_low_4_20;
float32_t pStates_main_low_4_20[8];

arm_biquad_casd_df1_inst_f32 filter_lowpass_instance_float_icp;
float32_t pStates_lowpass_float_icp[8];

arm_biquad_casd_df1_inst_f32 filter_lowpass_instance_float_4_20;
float32_t pStates_lowpass_float_4_20[8];

arm_biquad_casd_df1_inst_f32 filter_instance_highpass_1_icp;
float32_t pStates_highpass_1_icp[8];

arm_biquad_casd_df1_inst_f32 filter_instance_highpass_1_4_20;
float32_t pStates_highpass_1_4_20[8];

arm_biquad_casd_df1_inst_f32 filter_instance_highpass_2_icp;
float32_t pStates_highpass_2_icp[8];

arm_biquad_casd_df1_inst_f32 filter_instance_highpass_2_4_20;
float32_t pStates_highpass_2_4_20[8];

		




	
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void Acceleration_Task(void const * argument);
void Velocity_Task(void const * argument);
void Displacement_Task(void const * argument);
void Q_Average_A(void const * argument);
void Q_Average_V(void const * argument);
void Q_Average_D(void const * argument);
void ADC_supply_voltage(void const * argument);
void Usart_Task(void const * argument);
void DAC_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void FilterInit(void);
void Integrate(float32_t* input, float32_t* output, uint32_t size, arm_biquad_casd_df1_inst_f32 filter_instance);
extern void write_flash(uint32_t page, uint32_t* data, uint32_t size);
extern uint32_t read_flash(uint32_t addr);
extern uint16_t crc16(uint8_t *adr_buffer, uint32_t byte_cnt);
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
	

	
	acceleration_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));	
	velocity_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	displacement_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));	
	acceleration_queue_4_20 = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));	
	velocity_queue_4_20 = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	displacement_queue_4_20 = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	
	vSemaphoreCreateBinary(Semaphore1);
	vSemaphoreCreateBinary(Semaphore2);
	vSemaphoreCreateBinary(Semaphore_Acceleration);
	vSemaphoreCreateBinary(Semaphore_Velocity);
	vSemaphoreCreateBinary(Semaphore_Displacement);
	vSemaphoreCreateBinary(Q_Semaphore_Acceleration);
	vSemaphoreCreateBinary(Q_Semaphore_Velocity);
	vSemaphoreCreateBinary(Q_Semaphore_Displacement);
	
	FilterInit();
	
	
	
//	for(int i = 0; i<3200; i++)
//	sinus[i] = (float32_t) sin(2*3.1415*80*i/25600)*10;

	
       
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

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, Acceleration_Task, osPriorityNormal, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, Velocity_Task, osPriorityNormal, 0, 128);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* definition and creation of myTask06 */
  osThreadDef(myTask06, Displacement_Task, osPriorityNormal, 0, 128);
  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

  /* definition and creation of myTask07 */
  osThreadDef(myTask07, Q_Average_A, osPriorityNormal, 0, 128);
  myTask07Handle = osThreadCreate(osThread(myTask07), NULL);

  /* definition and creation of myTask08 */
  osThreadDef(myTask08, Q_Average_V, osPriorityNormal, 0, 128);
  myTask08Handle = osThreadCreate(osThread(myTask08), NULL);

  /* definition and creation of myTask09 */
  osThreadDef(myTask09, Q_Average_D, osPriorityNormal, 0, 128);
  myTask09Handle = osThreadCreate(osThread(myTask09), NULL);

  /* definition and creation of myTask10 */
  osThreadDef(myTask10, ADC_supply_voltage, osPriorityNormal, 0, 128);
  myTask10Handle = osThreadCreate(osThread(myTask10), NULL);

  /* definition and creation of myTask11 */
  osThreadDef(myTask11, Usart_Task, osPriorityNormal, 0, 128);
  myTask11Handle = osThreadCreate(osThread(myTask11), NULL);

  /* definition and creation of myTask12 */
  osThreadDef(myTask12, DAC_Task, osPriorityNormal, 0, 128);
  myTask12Handle = osThreadCreate(osThread(myTask12), NULL);

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

/* Acceleration_Task function */
void Acceleration_Task(void const * argument)
{
  /* USER CODE BEGIN Acceleration_Task */
	
	//float32_t* float_adc_value_ICP = pvPortMalloc(sizeof(float32_t)*ADC_BUFFER_SIZE);	
	//float32_t* float_adc_value_4_20 = pvPortMalloc(sizeof(float32_t)*ADC_BUFFER_SIZE);	

	float32_t temp_rms_acceleration_icp = 0.0;
	float32_t temp_rms_acceleration_4_20 = 0.0;
	
	float32_t temp_max_acceleration_icp = 0.0;
	float32_t temp_max_acceleration_4_20 = 0.0;
	
	float32_t temp_min_acceleration_icp = 0.0;
	float32_t temp_min_acceleration_4_20 = 0.0;
	
	uint32_t index;
	
  /* Infinite loop */
  for(;;)
  {		

		xSemaphoreTake( Semaphore_Acceleration, portMAX_DELAY );	
		


		//Получаем данные
		for (uint16_t i=0; i<ADC_BUFFER_SIZE; i++)
		{			
			float_adc_value_ICP[i] = (float32_t) raw_adc_value[i*2];					
			float_adc_value_4_20[i] = (float32_t) raw_adc_value[i*2+1];			
			//float_adc_value_ICP[i] = sinus[i];
			//float_adc_value_4_20[i] = sinus[i];	
		}		

		//Фильтр НЧ
		arm_biquad_cascade_df1_f32(&filter_main_low_icp, (float32_t*) &float_adc_value_ICP[0], (float32_t*) &float_adc_value_ICP[0], ADC_BUFFER_SIZE);								
		arm_biquad_cascade_df1_f32(&filter_main_low_4_20, (float32_t*) &float_adc_value_4_20[0], (float32_t*) &float_adc_value_4_20[0], ADC_BUFFER_SIZE);			
		
		//Фильтр ВЧ
		arm_biquad_cascade_df1_f32(&filter_main_high_icp, (float32_t*) &float_adc_value_ICP[0], (float32_t*) &float_adc_value_ICP[0], ADC_BUFFER_SIZE);		
		arm_biquad_cascade_df1_f32(&filter_main_high_4_20, (float32_t*) &float_adc_value_4_20[0], (float32_t*) &float_adc_value_4_20[0], ADC_BUFFER_SIZE);

		//СКЗ
		arm_rms_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_rms_acceleration_icp );
		arm_rms_f32( (float32_t*)&float_adc_value_4_20[0], ADC_BUFFER_SIZE, (float32_t*)&temp_rms_acceleration_4_20 );
		
		//Max
		arm_max_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_max_acceleration_icp, &index );
		arm_max_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_max_acceleration_4_20, &index );
		
		//Min
		arm_min_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_min_acceleration_icp, &index );
		arm_min_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_min_acceleration_4_20, &index );
		
		xQueueSend(acceleration_queue_icp, (void*)&temp_rms_acceleration_icp, 0);				
		xQueueSend(acceleration_queue_4_20, (void*)&temp_rms_acceleration_4_20, 0);		
		
		//vPortFree(float_adc_value_ICP);
		//vPortFree(float_adc_value_4_20);		
		
		xSemaphoreGive( Semaphore_Velocity );
		xSemaphoreGive( Q_Semaphore_Acceleration );		
		
		
  }
  /* USER CODE END Acceleration_Task */
}

/* Velocity_Task function */
void Velocity_Task(void const * argument)
{
  /* USER CODE BEGIN Velocity_Task */
	
	float32_t temp_rms_velocity_icp = 0.0;
	float32_t temp_rms_velocity_4_20 = 0.0;
	
	float32_t temp_max_velocity_icp = 0.0;
	float32_t temp_max_velocity_4_20 = 0.0;
	
	float32_t temp_min_velocity_icp = 0.0;
	float32_t temp_min_velocity_4_20 = 0.0;
			
	uint32_t index;
	
  
	/* Infinite loop */
  for(;;)
  {
    xSemaphoreTake( Semaphore_Velocity, portMAX_DELAY );
		
		//Копируем данные
		for (uint16_t i=0; i<ADC_BUFFER_SIZE; i++)
		{			
			float_adc_value_ICP[i] = (float32_t) raw_adc_value[i*2];
			float_adc_value_4_20[i] = (float32_t) raw_adc_value[i*2+1];		
			//float_adc_value_ICP[i] = (float32_t) sinus[i];
			//float_adc_value_4_20[i] = (float32_t) sinus[i];					
		}	
		
		//Фильтр НЧ
		arm_biquad_cascade_df1_f32(&filter_main_low_icp, (float32_t*) &float_adc_value_ICP[0], (float32_t*) &float_adc_value_ICP[0], ADC_BUFFER_SIZE);								
		arm_biquad_cascade_df1_f32(&filter_main_low_4_20, (float32_t*) &float_adc_value_4_20[0], (float32_t*) &float_adc_value_4_20[0], ADC_BUFFER_SIZE);					

		//Интегратор
		Integrate( (float32_t*)&float_adc_value_ICP[0], (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, filter_instance_highpass_1_icp );
		Integrate( (float32_t*)&float_adc_value_4_20[0], (float32_t*)&float_adc_value_4_20[0], ADC_BUFFER_SIZE, filter_instance_highpass_1_4_20 );
				
		//Фильтр ВЧ
		arm_biquad_cascade_df1_f32(&filter_instance_highpass_1_icp, (float32_t*) &float_adc_value_ICP[0], (float32_t*) &float_adc_value_ICP[0], ADC_BUFFER_SIZE);		
		arm_biquad_cascade_df1_f32(&filter_instance_highpass_1_4_20, (float32_t*) &float_adc_value_4_20[0], (float32_t*) &float_adc_value_4_20[0], ADC_BUFFER_SIZE);
		
		//СКЗ
		arm_rms_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_rms_velocity_icp );								
		arm_rms_f32( (float32_t*)&float_adc_value_4_20[0], ADC_BUFFER_SIZE, (float32_t*)&temp_rms_velocity_4_20 );	

		//Max
		arm_max_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_max_velocity_icp, &index );
		arm_max_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_max_velocity_4_20, &index );
		
		//Min
		arm_min_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_min_velocity_icp, &index );
		arm_min_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_min_velocity_4_20, &index );		
		
		xQueueSend(velocity_queue_icp, (void*)&temp_rms_velocity_icp, 0);
		xQueueSend(velocity_queue_4_20, (void*)&temp_rms_velocity_4_20, 0);
		
		
		xSemaphoreGive( Semaphore_Displacement );
		xSemaphoreGive( Q_Semaphore_Velocity );		

		
  }
  /* USER CODE END Velocity_Task */
}

/* Displacement_Task function */
void Displacement_Task(void const * argument)
{
  /* USER CODE BEGIN Displacement_Task */
	
	float32_t temp_rms_displacement_icp = 0.0;
	float32_t temp_rms_displacement_4_20 = 0.0;
	
	float32_t temp_max_displacement_icp = 0.0;
	float32_t temp_max_displacement_4_20 = 0.0;
	
	float32_t temp_min_displacement_icp = 0.0;
	float32_t temp_min_displacement_4_20 = 0.0;
			
	uint32_t index;
	
	
	
  /* Infinite loop */
  for(;;)
  {
    xSemaphoreTake( Semaphore_Displacement, portMAX_DELAY );				
				
		//Интегратор			
		Integrate( (float32_t*)&float_adc_value_ICP[0], (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, filter_instance_highpass_2_icp );		
		Integrate( (float32_t*)&float_adc_value_4_20[0], (float32_t*)&float_adc_value_4_20[0], ADC_BUFFER_SIZE, filter_instance_highpass_2_4_20 );		
		
		//Фильтр ВЧ
		arm_biquad_cascade_df1_f32(&filter_instance_highpass_2_icp, (float32_t*) &float_adc_value_ICP[0], (float32_t*) &float_adc_value_ICP[0], ADC_BUFFER_SIZE);		
		arm_biquad_cascade_df1_f32(&filter_instance_highpass_2_4_20, (float32_t*) &float_adc_value_4_20[0], (float32_t*) &float_adc_value_4_20[0], ADC_BUFFER_SIZE);		
		
		//СКЗ
		arm_rms_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_rms_displacement_icp );								
		arm_rms_f32( (float32_t*)&float_adc_value_4_20[0], ADC_BUFFER_SIZE, (float32_t*)&temp_rms_displacement_4_20 );	

		//Max
		arm_max_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_max_displacement_icp, &index );
		arm_max_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_max_displacement_4_20, &index );
		
		//Min
		arm_min_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_min_displacement_icp, &index );
		arm_min_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_min_displacement_4_20, &index );		
		
								
		xQueueSend(displacement_queue_icp, (void*)&temp_rms_displacement_icp, 0);
		xQueueSend(displacement_queue_4_20, (void*)&temp_rms_displacement_4_20, 0);

		
		xSemaphoreGive( Q_Semaphore_Displacement );
		
  }
  /* USER CODE END Displacement_Task */
}

/* Q_Average_A function */
void Q_Average_A(void const * argument)
{
  /* USER CODE BEGIN Q_Average_A */
  /* Infinite loop */
  for(;;)
  {		
			xSemaphoreTake( Q_Semaphore_Acceleration, portMAX_DELAY );
				
			
			queue_count_A_icp = uxQueueMessagesWaiting(acceleration_queue_icp);	
				
			if (queue_count_A_icp == QUEUE_LENGHT)
			{						
					rms_acceleration_icp = 0.0;		
								
					for (uint16_t i=0; i<QUEUE_LENGHT; i++)
					{
							xQueueReceive(acceleration_queue_icp, (void *) &Q_A_rms_array_icp[i], 0);										
					}
					
					arm_rms_f32((float32_t*) &Q_A_rms_array_icp, QUEUE_LENGHT, (float32_t*)&rms_acceleration_icp);	
					
					rms_acceleration_icp *= (float32_t) COEF_TRANSFORM_icp;

			}
				
				
				
			queue_count_A_4_20 = uxQueueMessagesWaiting(acceleration_queue_4_20);		

			if (queue_count_A_4_20 == QUEUE_LENGHT)
			{						
					rms_acceleration_4_20 = 0.0;		
								
					for (uint16_t i=0; i<QUEUE_LENGHT; i++)
					{
							xQueueReceive(acceleration_queue_4_20, (void *) &Q_A_rms_array_4_20[i], 0);										
					}
					
					arm_rms_f32((float32_t*) &Q_A_rms_array_4_20, QUEUE_LENGHT, (float32_t*)&rms_acceleration_4_20);	
					
					rms_acceleration_4_20 *= (float32_t) COEF_TRANSFORM_4_20;
					
			}

				
  }
  /* USER CODE END Q_Average_A */
}

/* Q_Average_V function */
void Q_Average_V(void const * argument)
{
  /* USER CODE BEGIN Q_Average_V */
  /* Infinite loop */
  for(;;)
  {
			xSemaphoreTake( Q_Semaphore_Velocity, portMAX_DELAY );
			
			queue_count_V_icp = uxQueueMessagesWaiting(velocity_queue_icp);		
			
			if (queue_count_V_icp == QUEUE_LENGHT)
			{						
					rms_velocity_icp = 0.0;		
								
					for (uint16_t i=0; i<QUEUE_LENGHT; i++)
					{
							xQueueReceive(velocity_queue_icp, (void *) &Q_V_rms_array_icp[i], 0);										
					}
					
					arm_rms_f32((float32_t*) &Q_V_rms_array_icp, QUEUE_LENGHT, (float32_t*)&rms_velocity_icp);
						
					rms_velocity_icp *= (float32_t) COEF_TRANSFORM_icp;
					
			}
			
			
			queue_count_V_4_20 = uxQueueMessagesWaiting(velocity_queue_4_20);		

			if (queue_count_V_4_20 == QUEUE_LENGHT)
			{						
				
					rms_velocity_4_20 = 0.0;		
								
					for (uint16_t i=0; i<QUEUE_LENGHT; i++)
					{
							xQueueReceive(velocity_queue_4_20, (void *) &Q_V_rms_array_4_20[i], 0);										
					}
					
					arm_rms_f32((float32_t*) &Q_V_rms_array_4_20, QUEUE_LENGHT, (float32_t*)&rms_velocity_4_20);		

					rms_velocity_4_20 *= (float32_t) COEF_TRANSFORM_4_20;					
			}

  }
  /* USER CODE END Q_Average_V */
}

/* Q_Average_D function */
void Q_Average_D(void const * argument)
{
  /* USER CODE BEGIN Q_Average_D */
  /* Infinite loop */
  for(;;)
  {
			xSemaphoreTake( Q_Semaphore_Displacement, portMAX_DELAY );
				
			queue_count_D_icp = uxQueueMessagesWaiting(displacement_queue_icp);		
			
			if (queue_count_D_icp == QUEUE_LENGHT)
			{						
					rms_displacement_icp = 0.0;		
								
					for (uint16_t i=0; i<QUEUE_LENGHT; i++)
					{
							xQueueReceive(displacement_queue_icp, (void *) &Q_D_rms_array_icp[i], 0);										
					}
					
					arm_rms_f32((float32_t*) &Q_D_rms_array_icp, QUEUE_LENGHT, (float32_t*)&rms_displacement_icp);

					rms_displacement_icp *= (float32_t) COEF_TRANSFORM_icp;					
					
			}
				
				
				
			queue_count_D_4_20 = uxQueueMessagesWaiting(displacement_queue_4_20);		

			if (queue_count_D_4_20 == QUEUE_LENGHT)
			{						
					rms_displacement_4_20 = 0.0;		
								
					for (uint16_t i=0; i<QUEUE_LENGHT; i++)
					{
							xQueueReceive(displacement_queue_4_20, (void *) &Q_D_rms_array_4_20[i], 0);										
					}
					
					arm_rms_f32((float32_t*) &Q_D_rms_array_4_20, QUEUE_LENGHT, (float32_t*)&rms_displacement_4_20);			

					rms_displacement_4_20 *= (float32_t) COEF_TRANSFORM_4_20;	
					
					xTotalTimeSuspended = xTaskGetTickCount() - xTimeBefore;
					xTimeBefore = xTaskGetTickCount();
					
			}				
				
				
  }
  /* USER CODE END Q_Average_D */
}

/* ADC_supply_voltage function */
void ADC_supply_voltage(void const * argument)
{
  /* USER CODE BEGIN ADC_supply_voltage */
	
	uint16_t supply_voltage = 0;
  /* Infinite loop */
  for(;;)
  {
		HAL_ADCEx_InjectedStart(&hadc1);
		HAL_ADCEx_InjectedPollForConversion(&hadc1, 100);
		supply_voltage = HAL_ADCEx_InjectedGetValue(&hadc1, 1);
		HAL_ADCEx_InjectedStop(&hadc1);
	
		power_supply_voltage = (float32_t) supply_voltage * COEF_TRANSFORM_SUPPLY;
		
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		
    osDelay(100);
  }
  /* USER CODE END ADC_supply_voltage */
}

/* Usart_Task function */
void Usart_Task(void const * argument)
{
  /* USER CODE BEGIN Usart_Task */
	uint8_t transmitBuffer[32];
	uint8_t receiveBuffer[32];
	
	
	
  /* Infinite loop */
  for(;;)
  {
		
		for (unsigned char i = 0; i < 32; i++)
		{
			transmitBuffer[i] = i;
			receiveBuffer[i] = 0;
		}
 
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		
		//HAL_UART_Transmit(&huart1, transmitBuffer, 32, 1000);	
		//HAL_UART_Receive_IT(&huart1, receiveBuffer, 32);	
		
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
		//__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
		//__HAL_UART_ENABLE_IT(&huart2, UART_IT_TXE);
		
    osDelay(1000);
  }
  /* USER CODE END Usart_Task */
}

/* DAC_Task function */
void DAC_Task(void const * argument)
{
  /* USER CODE BEGIN DAC_Task */
	uint32_t out_dac = 0.0;
  /* Infinite loop */
  for(;;)
  {
						
		out_dac = (uint32_t) (dac_voltage * 4096) / 3.3;
		
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, out_dac);
		
    osDelay(100);
  }
  /* USER CODE END DAC_Task */
}

/* USER CODE BEGIN Application */

void Integrate(float32_t* input, float32_t* output, uint32_t size, arm_biquad_casd_df1_inst_f32 filter_instance)
{	
	
	input[0] /= (float32_t) 25.6;
	
	for (uint16_t i=1; i < size; i++)
	{
		output[i] = ( input[i] / (float32_t) 25.6 ) + input[i-1];						
	}			
	
	//arm_biquad_cascade_df1_f32(&filter_instance, (float32_t*) &output[0], (float32_t*) &output[0], size);	
	
}

void FilterInit(void)
{

		//Butterworth 3 Order, LowPass 1000 Hz
		static float32_t coef_main_low_gain[] = {
			1*0.013361128677806023,  2*0.013361128677806023,  1*0.013361128677806023,  1.729897146458744,    -0.78334166116996795,        
			1*0.10979617017302817,  1*0.10979617017302817,  0*0.10979617017302817,  0.78040765965394354,  0		
		};
		
		arm_biquad_cascade_df1_init_f32(&filter_main_low_icp, 2, (float32_t *) &coef_main_low_gain[0], &pStates_main_low_icp[0]);							
		arm_biquad_cascade_df1_init_f32(&filter_main_low_4_20, 2, (float32_t *) &coef_main_low_gain[0], &pStates_main_low_4_20[0]);	
		
		
		//Butterworth 3 Order, HighPass 2 Hz
		static float32_t coef_main_highpass_2Hz_gain[] = {		
			1*0.99975456308379129,  -2*0.99975456308379129,  1*0.99975456308379129,    1.9995090057185783,   -0.99950924661658691,       
			1*0.99975462329351572,  -1*0.99975462329351572,  0,    0.99950924658703155,  0                         
		};
		
		//Butterworth 3 Order, HighPass 5 Hz
		static float32_t coef_main_highpass_5Hz_gain[] = {
			1*0.99938640783871391,  -2*0.99938640783871391,  1*0.99938640783871391,  1.9987720631482098,   -0.99877356820664565,       
			1*0.99938678387259139,  -1*0.99938678387259139,  0*0.99938678387259139,  0.99877356774518267, 0		
		};
		
		//Butterworth 3 Order, HighPass 10 Hz
		static float32_t coef_main_highpass_10Hz_gain[] = {		                                           
			1*0.99877281659950468,  -2*0.99877281659950468,  1*0.99877281659950468,    1.997542624927988,    -0.99754864147003097,       
			1*0.99877431889142487,  -1*0.99877431889142487,  0,    0.99754863778284986,  0                         
		};
		

		if (FILTER_MODE == 1)		
		{
			arm_biquad_cascade_df1_init_f32(&filter_main_high_icp, 2, (float32_t *) &coef_main_highpass_2Hz_gain[0], &pStates_main_high_icp[0]);				
			arm_biquad_cascade_df1_init_f32(&filter_main_high_4_20, 2, (float32_t *) &coef_main_highpass_2Hz_gain[0], &pStates_main_high_4_20[0]);	
			
			//Фильтр для интегратора			
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_1_icp, 2, (float32_t *) &coef_main_highpass_2Hz_gain[0], &pStates_highpass_1_icp[0]);							
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_1_4_20, 2, (float32_t *) &coef_main_highpass_2Hz_gain[0], &pStates_highpass_1_4_20[0]);	
				
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_2_icp, 2, (float32_t *) &coef_main_highpass_2Hz_gain[0], &pStates_highpass_2_icp[0]);				
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_2_4_20, 2, (float32_t *) &coef_main_highpass_2Hz_gain[0], &pStates_highpass_2_4_20[0]);	
		}
		else
		if (FILTER_MODE == 2)		
		{
			arm_biquad_cascade_df1_init_f32(&filter_main_high_icp, 2, (float32_t *) &coef_main_highpass_5Hz_gain[0], &pStates_main_high_icp[0]);				
			arm_biquad_cascade_df1_init_f32(&filter_main_high_4_20, 2, (float32_t *) &coef_main_highpass_5Hz_gain[0], &pStates_main_high_4_20[0]);	
			
			//Фильтр для интегратора			
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_1_icp, 2, (float32_t *) &coef_main_highpass_5Hz_gain[0], &pStates_highpass_1_icp[0]);							
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_1_4_20, 2, (float32_t *) &coef_main_highpass_5Hz_gain[0], &pStates_highpass_1_4_20[0]);	
				
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_2_icp, 2, (float32_t *) &coef_main_highpass_5Hz_gain[0], &pStates_highpass_2_icp[0]);				
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_2_4_20, 2, (float32_t *) &coef_main_highpass_5Hz_gain[0], &pStates_highpass_2_4_20[0]);	
		}
		else
		if (FILTER_MODE == 3)		
		{
			arm_biquad_cascade_df1_init_f32(&filter_main_high_icp, 2, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_main_high_icp[0]);				
			arm_biquad_cascade_df1_init_f32(&filter_main_high_4_20, 2, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_main_high_4_20[0]);	
			
			//Фильтр для интегратора			
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_1_icp, 2, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_highpass_1_icp[0]);							
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_1_4_20, 2, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_highpass_1_4_20[0]);	
				
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_2_icp, 2, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_highpass_2_icp[0]);				
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_2_4_20, 2, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_highpass_2_4_20[0]);	
		}	
		
		
}



/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
