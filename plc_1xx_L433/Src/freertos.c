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


//float32_t sinus[ADC_BUFFER_SIZE];
uint16_t raw_adc_value[RAW_ADC_BUFFER_SIZE];
float32_t float_adc_value_ICP[ADC_BUFFER_SIZE];
float32_t float_adc_value_4_20[ADC_BUFFER_SIZE];


float32_t temp_rms_acceleration_icp = 0.0;
float32_t temp_rms_acceleration_4_20 = 0.0;
float32_t temp_rms_velocity_icp = 0.0;
float32_t temp_rms_velocity_4_20 = 0.0;
float32_t temp_rms_displacement_icp = 0.0;
float32_t temp_rms_displacement_4_20 = 0.0;


uint8_t queue_count;
float32_t qrms;
float32_t qrms_array[QUEUE_LENGHT];
uint64_t xTimeBefore, xTotalTimeSuspended;

xQueueHandle queue;
xQueueHandle queue2;

xSemaphoreHandle Semaphore1, Semaphore2, Semaphore_Acceleration, Semaphore_Velocity, Semaphore_Displacement;

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


		
float32_t settings[REG_COUNT];



	
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void GetADC_Task(void const * argument);
void Filter_Task(void const * argument);
void Acceleration_Task(void const * argument);
void Velocity_Task(void const * argument);
void Displacement_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void FilterInit(void);
void Integrate(float32_t* input, float32_t* output, uint32_t size, arm_biquad_casd_df1_inst_f32 filter_instance);
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
	queue2 = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	
	vSemaphoreCreateBinary(Semaphore1);
	vSemaphoreCreateBinary(Semaphore2);
	vSemaphoreCreateBinary(Semaphore_Acceleration);
	vSemaphoreCreateBinary(Semaphore_Velocity);
	vSemaphoreCreateBinary(Semaphore_Displacement);
	
	FilterInit();
	
//	for(int i = 0; i<3200; i++)
//	sinus[i] = (float32_t) sin(2*3.1415*300*i/25600)*10;
       
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
  osThreadDef(myTask04, Acceleration_Task, osPriorityNormal, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, Velocity_Task, osPriorityNormal, 0, 128);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* definition and creation of myTask06 */
  osThreadDef(myTask06, Displacement_Task, osPriorityNormal, 0, 128);
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

		for (uint16_t i=0; i<ADC_BUFFER_SIZE; i++)
		{			
			float_adc_value_ICP[i] = (float32_t) raw_adc_value[i*2] * COEF_TRANSFORM;						
			float_adc_value_4_20[i] = (float32_t) raw_adc_value[i*2+1] * COEF_TRANSFORM;								
		}
		
		xSemaphoreGive( Semaphore2 );
		
  }
  /* USER CODE END GetADC_Task */
}

/* Filter_Task function */
void Filter_Task(void const * argument)
{
  /* USER CODE BEGIN Filter_Task */

  /* Infinite loop */
  for(;;)
  {		
		
    xSemaphoreTake( Semaphore2, portMAX_DELAY );
				
		arm_biquad_cascade_df1_f32(&filter_main_high_icp, (float32_t*) &float_adc_value_ICP[0], (float32_t*) &float_adc_value_ICP[0], ADC_BUFFER_SIZE);		
		arm_biquad_cascade_df1_f32(&filter_main_high_4_20, (float32_t*) &float_adc_value_4_20[0], (float32_t*) &float_adc_value_4_20[0], ADC_BUFFER_SIZE);

		arm_biquad_cascade_df1_f32(&filter_main_low_icp, (float32_t*) &float_adc_value_ICP[0], (float32_t*) &float_adc_value_ICP[0], ADC_BUFFER_SIZE);								
		arm_biquad_cascade_df1_f32(&filter_main_low_4_20, (float32_t*) &float_adc_value_4_20[0], (float32_t*) &float_adc_value_4_20[0], ADC_BUFFER_SIZE);			
		
		
		xSemaphoreGive( Semaphore_Acceleration );		
  }
  /* USER CODE END Filter_Task */
}

/* Acceleration_Task function */
void Acceleration_Task(void const * argument)
{
  /* USER CODE BEGIN Acceleration_Task */
  /* Infinite loop */
  for(;;)
  {

		xSemaphoreTake( Semaphore_Acceleration, portMAX_DELAY );		

		arm_rms_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_rms_acceleration_icp );								
		arm_rms_f32( (float32_t*)&float_adc_value_4_20[0], ADC_BUFFER_SIZE, (float32_t*)&temp_rms_acceleration_4_20 );						
		
		xSemaphoreGive( Semaphore_Velocity );		
  }
  /* USER CODE END Acceleration_Task */
}

/* Velocity_Task function */
void Velocity_Task(void const * argument)
{
  /* USER CODE BEGIN Velocity_Task */
	
	
	
  /* Infinite loop */
  for(;;)
  {
    xSemaphoreTake( Semaphore_Velocity, portMAX_DELAY );
						
		Integrate( (float32_t*)&float_adc_value_ICP[0], (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, filter_instance_highpass_1_icp );
		arm_rms_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_rms_velocity_icp );								
				
		Integrate( (float32_t*)&float_adc_value_4_20[0], (float32_t*)&float_adc_value_4_20[0], ADC_BUFFER_SIZE, filter_instance_highpass_1_4_20 );
		arm_rms_f32( (float32_t*)&float_adc_value_4_20[0], ADC_BUFFER_SIZE, (float32_t*)&temp_rms_velocity_4_20 );								
		
		
		xSemaphoreGive( Semaphore_Displacement );
		
  }
  /* USER CODE END Velocity_Task */
}

/* Displacement_Task function */
void Displacement_Task(void const * argument)
{
  /* USER CODE BEGIN Displacement_Task */
  /* Infinite loop */
  for(;;)
  {
    xSemaphoreTake( Semaphore_Displacement, portMAX_DELAY );
				
		Integrate( (float32_t*)&float_adc_value_ICP[0], (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, filter_instance_highpass_2_icp );
		arm_rms_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_rms_displacement_icp );								
				
		
		Integrate( (float32_t*)&float_adc_value_4_20[0], (float32_t*)&float_adc_value_4_20[0], ADC_BUFFER_SIZE, filter_instance_highpass_2_4_20 );
		arm_rms_f32( (float32_t*)&float_adc_value_4_20[0], ADC_BUFFER_SIZE, (float32_t*)&temp_rms_displacement_4_20 );					
		
  }
  /* USER CODE END Displacement_Task */
}

/* USER CODE BEGIN Application */

void Integrate(float32_t* input, float32_t* output, uint32_t size, arm_biquad_casd_df1_inst_f32 filter_instance)
{
	input[0] /= (float32_t) 25.6;
	
	for (uint16_t i=1; i < size; i++)
	{
		output[i] = ( input[i] / (float32_t) 25.6 ) + input[i-1];				
	}
	
	arm_biquad_cascade_df1_f32(&filter_instance, (float32_t*) &output[0], (float32_t*) &output[0], size);
	
}

void FilterInit(void)
{

		//Butterworth 3 Order, LowPass 1000 Hz
		static float32_t coef_main_low_gain[] = {
			1*0.013361128677806023,  2*0.013361128677806023,  1*0.013361128677806023,  1.729897146458744,    -0.78334166116996795,        
			1*0.10979617017302817,  1*0.10979617017302817,  0*0.10979617017302817,  0.78040765965394354,  -0		
		};
		
		arm_biquad_cascade_df1_init_f32(&filter_main_low_icp, 2, (float32_t *) &coef_main_low_gain[0], &pStates_main_low_icp[0]);							
		arm_biquad_cascade_df1_init_f32(&filter_main_low_4_20, 2, (float32_t *) &coef_main_low_gain[0], &pStates_main_low_4_20[0]);	
		
		

		//Butterworth 3 Order, HighPass 5 Hz
		static float32_t coef_main_high_gain[] = {
			1*0.99938640783871391,  -2*0.99938640783871391,  1*0.99938640783871391,  1.9987720631482098,   -0.99877356820664565,       
			1*0.99938678387259139,  -1*0.99938678387259139,  0*0.99938678387259139,  0.99877356774518267, -0		
		};

			
		arm_biquad_cascade_df1_init_f32(&filter_main_high_icp, 2, (float32_t *) &coef_main_high_gain[0], &pStates_main_high_icp[0]);				
		arm_biquad_cascade_df1_init_f32(&filter_main_high_4_20, 2, (float32_t *) &coef_main_high_gain[0], &pStates_main_high_4_20[0]);	
		
				
		//Butterworth 4 Order, HighPass 3 Hz
		static float32_t coef_highpass_gain[] = {
		1*0.99971816996354446, -2*0.99971816996354446,  1*0.99971816996354446,    1.999436068926681,   -0.99943661092749692,        
		1*0.99932006460003497, -2*0.99932006460003497,  1*0.99932006460003497,    1.9986398583075793,  -0.99864040009256083        	
		};

				
		arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_1_icp, 2, (float32_t *) &coef_highpass_gain[0], &pStates_highpass_1_icp[0]);							
		arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_1_4_20, 2, (float32_t *) &coef_highpass_gain[0], &pStates_highpass_1_4_20[0]);	
			
		arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_2_icp, 2, (float32_t *) &coef_highpass_gain[0], &pStates_highpass_2_icp[0]);				
		arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_2_4_20, 2, (float32_t *) &coef_highpass_gain[0], &pStates_highpass_2_4_20[0]);	

		
		
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
