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

#include "fonts.h"
#include "ssd1306.h"

#include "stm32l4xx_it.h"
#include "modbus_reg_map.h"
#include "Flash_manager.h"

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
osThreadId myTask13Handle;
osThreadId myTask14Handle;
osThreadId myTask15Handle;
osThreadId myTask16Handle;
osThreadId myTask17Handle;
osThreadId myTask18Handle;
osThreadId myTask19Handle;
osThreadId myTask20Handle;
osThreadId myTask21Handle;
osThreadId myTask22Handle;
osThreadId myTask23Handle;
osThreadId myTask24Handle;

/* USER CODE BEGIN Variables */

xSemaphoreHandle 	Semaphore1, Semaphore2,
									Semaphore_Acceleration, Semaphore_Velocity, Semaphore_Displacement,
									Q_Semaphore_Acceleration, Q_Semaphore_Velocity, Q_Semaphore_Displacement,
									Semaphore_Modbus_Rx, Semaphore_Modbus_Tx, 
									Semaphore_Master_Modbus_Rx, Semaphore_Master_Modbus_Tx,
									Semaphore_Relay_1, Semaphore_Relay_2,
									Semaphore_HART_Receive, Semaphore_HART_Transmit;

//float32_t sinus[ADC_BUFFER_SIZE];
uint16_t raw_adc_value[RAW_ADC_BUFFER_SIZE];
float32_t float_adc_value_ICP[ADC_BUFFER_SIZE];
float32_t float_adc_value_4_20[ADC_BUFFER_SIZE];

float32_t dac_voltage = 0.0;

float32_t mean_4_20 = 0.0;

//float32_t rms_velocity_4_20 = 0.0;
//float32_t rms_acceleration_4_20 = 0.0;
//float32_t rms_displacement_4_20 = 0.0;

//float32_t max_acceleration_icp = 0.0;
//float32_t min_acceleration_4_20 = 0.0;
//float32_t max_velocity_icp = 0.0;
//float32_t min_velocity_4_20 = 0.0;
//float32_t max_displacement_icp = 0.0;
//float32_t min_displacement_4_20 = 0.0;

uint64_t xTimeBefore, xTotalTimeSuspended;

float32_t Q_A_rms_array_icp[QUEUE_LENGHT];
float32_t Q_V_rms_array_icp[QUEUE_LENGHT];
float32_t Q_D_rms_array_icp[QUEUE_LENGHT];
float32_t Q_A_mean_array_4_20[QUEUE_LENGHT];
float32_t Q_V_rms_array_4_20[QUEUE_LENGHT];
float32_t Q_D_rms_array_4_20[QUEUE_LENGHT];

xQueueHandle acceleration_queue_icp;
xQueueHandle velocity_queue_icp;
xQueueHandle displacement_queue_icp;
xQueueHandle queue_4_20;
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

arm_biquad_casd_df1_inst_f32 filter_instance_highpass_1_icp;
float32_t pStates_highpass_1_icp[8];

arm_biquad_casd_df1_inst_f32 filter_instance_highpass_1_4_20;
float32_t pStates_highpass_1_4_20[8];

arm_biquad_casd_df1_inst_f32 filter_instance_highpass_2_icp;
float32_t pStates_highpass_2_icp[8];

arm_biquad_casd_df1_inst_f32 filter_instance_highpass_2_4_20;
float32_t pStates_highpass_2_4_20[8];

arm_biquad_casd_df1_inst_f32 filter_instance_highpass_3_icp;
float32_t pStates_highpass_3_icp[8];

arm_biquad_casd_df1_inst_f32 filter_instance_highpass_4_icp;
float32_t pStates_highpass_4_icp[8];

extern uint16_t settings[REG_COUNT]; //массив настроек 

uint8_t button_state = 0;

uint8_t transmitBuffer[REG_COUNT*2+5];
uint8_t receiveBuffer[16];
uint8_t master_transmitBuffer[8];
uint8_t master_receiveBuffer[255];
uint8_t HART_receiveBuffer[16];
uint8_t HART_transmitBuffer[REG_COUNT*2+5];
//ICP
float32_t icp_voltage = 0.0;
float32_t lo_warning_icp = 0.0;
float32_t hi_warning_icp = 0.0;
float32_t lo_emerg_icp = 0.0;
float32_t hi_emerg_icp = 0.0;
uint8_t break_sensor_icp = 0;
float32_t break_level_icp = 0.0;
float32_t coef_ampl_icp = 0.0;
float32_t coef_offset_icp = 0.0;
float32_t range_icp = 0.0;
uint8_t filter_mode_icp = 0;
float32_t rms_acceleration_icp = 0.0;
float32_t rms_velocity_icp = 0.0;
float32_t rms_displacement_icp = 0.0;

//4-20
float32_t current_4_20 = 0.0; //ток входного канала 4-20
float32_t out_required_current = 0.0; //ток для выдачи в выходной канал 4-20
float32_t lo_warning_420 = 0.0;
float32_t hi_warning_420 = 0.0;
float32_t lo_emerg_420 = 0.0;
float32_t hi_emerg_420 = 0.0;
uint8_t break_sensor_420 = 0;
float32_t break_level_420 = 0.0;
float32_t coef_ampl_420 = 0.0;
float32_t coef_offset_420 = 0.0;
float32_t range_420 = 0.0;
float32_t calculated_value_420 = 0.0;

//485
uint8_t slave_adr_mb_master = 0;
float32_t mb_master_BaudRate = 0.0;
uint16_t mb_master_timeout = 0;
uint16_t slave_reg_mb_master = 0;
uint8_t slave_func_mb_master = 0;
float32_t mb_master_recieve_data = 0.0;
uint16_t quantity_reg_mb_master = 0;
float32_t lo_warning_485 = 0.0;
float32_t hi_warning_485 = 0.0;
float32_t lo_emerg_485 = 0.0;
float32_t hi_emerg_485 = 0.0;
uint16_t coef_A_mb_master = 0;

uint16_t mb_master_numreg_1 = 0;
uint16_t mb_master_numreg_2 = 0;
uint16_t mb_master_numreg_3 = 0;
uint16_t mb_master_numreg_4 = 0;
float32_t mb_master_recieve_data_1 = 0.0;
uint16_t mb_master_recieve_data_2 = 0;
uint16_t mb_master_recieve_data_3 = 0;
uint16_t mb_master_recieve_data_4 = 0;
float32_t mb_master_recieve_value_1 = 0.0;
float32_t mb_master_recieve_value_2 = 0.0;
float32_t mb_master_recieve_value_3 = 0.0;
float32_t mb_master_recieve_value_4 = 0.0;

float32_t mb_master_lo_warning_485_1 = 0.0;
float32_t mb_master_hi_warning_485_1 = 0.0;
float32_t mb_master_lo_emerg_485_1 = 0.0;
float32_t mb_master_hi_emerg_485_1 = 0.0;
float32_t mb_master_lo_warning_485_2 = 0.0;
float32_t mb_master_hi_warning_485_2 = 0.0;
float32_t mb_master_lo_emerg_485_2 = 0.0;
float32_t mb_master_hi_emerg_485_2 = 0.0;
float32_t mb_master_lo_warning_485_3 = 0.0;
float32_t mb_master_hi_warning_485_3 = 0.0;
float32_t mb_master_lo_emerg_485_3 = 0.0;
float32_t mb_master_hi_emerg_485_3 = 0.0;
float32_t mb_master_lo_warning_485_4 = 0.0;
float32_t mb_master_hi_warning_485_4 = 0.0;
float32_t mb_master_lo_emerg_485_4 = 0.0;
float32_t mb_master_hi_emerg_485_4 = 0.0;


//Реле
uint8_t state_emerg_relay = 0;
uint8_t state_warning_relay = 0;
uint8_t mode_relay = 0;
uint8_t source_signal_relay = 0;
uint16_t delay_relay = 0;

//Выход 4-20
uint8_t source_signal_out420 = 0;
float32_t coef_1 = 0.0;
volatile float32_t range_out_420 = 0.0;
	
	

//Дискретный вход
uint8_t bin_input_state = 0;

//Общие
extern float32_t cpu_float;
float32_t power_supply_voltage = 0.0;
uint16_t slave_adr = 0;	
uint16_t warming_up = 0;
uint8_t warming_flag = 1;
float32_t power_supply_warning_lo = 0.0;
float32_t power_supply_warning_hi = 0.0;

//Кнопки
uint8_t button_left = 0;
uint8_t button_right = 0;
uint8_t button_up = 0;
uint8_t button_down = 0;
uint8_t button_center = 0;

uint8_t button_left_pressed_in = 0;
uint8_t button_right_pressed_in = 0;
uint8_t button_up_pressed_in = 0;
uint8_t button_down_pressed_in = 0;
uint8_t button_center_pressed_in_short = 0;
uint8_t button_center_pressed_in_long = 0;

extern FontDef font_7x12_RU;
extern FontDef font_7x12;
extern FontDef font_8x15_RU;
extern FontDef font_8x14;
extern FontDef font_5x10_RU;
extern FontDef font_5x10;

uint16_t menu_index_pointer = 0;



volatile int temp_var_1 = 0;
volatile int temp_var_2 = 0;

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
void Lights_Task(void const * argument);
void DAC_Task(void const * argument);
void Display_Task(void const * argument);
void Button_Task(void const * argument);
void Modbus_Receive_Task(void const * argument);
void Modbus_Transmit_Task(void const * argument);
void Master_Modbus_Receive(void const * argument);
void Master_Modbus_Transmit(void const * argument);
void Data_Storage_Task(void const * argument);
void TiggerLogic_Task(void const * argument);
void Relay_1_Task(void const * argument);
void Relay_2_Task(void const * argument);
void HART_Receive_Task(void const * argument);
void HART_Transmit_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void FilterInit(void);
void Integrate(float32_t* input, float32_t* output, uint32_t size, arm_biquad_casd_df1_inst_f32 filter_instance);
extern void write_flash(uint32_t page, uint32_t* data, uint32_t size);
extern uint32_t read_flash(uint32_t addr);
extern uint16_t crc16(uint8_t *adr_buffer, uint32_t byte_cnt);
uint16_t crc_calculating(unsigned char* puchMsg, unsigned short usDataLen);
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

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
	queue_4_20 = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));	
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
	vSemaphoreCreateBinary(Semaphore_Modbus_Rx);
	vSemaphoreCreateBinary(Semaphore_Modbus_Tx);
	vSemaphoreCreateBinary(Semaphore_Master_Modbus_Rx);
	vSemaphoreCreateBinary(Semaphore_Master_Modbus_Tx);
	vSemaphoreCreateBinary(Semaphore_Relay_1);
	vSemaphoreCreateBinary(Semaphore_Relay_2);
	vSemaphoreCreateBinary(Semaphore_HART_Receive);
	//vSemaphoreCreateBinary(Semaphore_HART_Transmit);
	Semaphore_HART_Transmit = xSemaphoreCreateCounting( 10, 0 );
	
	
	
	FilterInit();
	
	
	
//	for(int i = 0; i<3200; i++)
//	sinus[i] = (float32_t) sin(2*3.1415*80*i/25600)*15;

	
       
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
  osThreadDef(myTask11, Lights_Task, osPriorityNormal, 0, 128);
  myTask11Handle = osThreadCreate(osThread(myTask11), NULL);

  /* definition and creation of myTask12 */
  osThreadDef(myTask12, DAC_Task, osPriorityNormal, 0, 128);
  myTask12Handle = osThreadCreate(osThread(myTask12), NULL);

  /* definition and creation of myTask13 */
  osThreadDef(myTask13, Display_Task, osPriorityNormal, 0, 128);
  myTask13Handle = osThreadCreate(osThread(myTask13), NULL);

  /* definition and creation of myTask14 */
  osThreadDef(myTask14, Button_Task, osPriorityNormal, 0, 128);
  myTask14Handle = osThreadCreate(osThread(myTask14), NULL);

  /* definition and creation of myTask15 */
  osThreadDef(myTask15, Modbus_Receive_Task, osPriorityNormal, 0, 128);
  myTask15Handle = osThreadCreate(osThread(myTask15), NULL);

  /* definition and creation of myTask16 */
  osThreadDef(myTask16, Modbus_Transmit_Task, osPriorityNormal, 0, 128);
  myTask16Handle = osThreadCreate(osThread(myTask16), NULL);

  /* definition and creation of myTask17 */
  osThreadDef(myTask17, Master_Modbus_Receive, osPriorityNormal, 0, 128);
  myTask17Handle = osThreadCreate(osThread(myTask17), NULL);

  /* definition and creation of myTask18 */
  osThreadDef(myTask18, Master_Modbus_Transmit, osPriorityNormal, 0, 128);
  myTask18Handle = osThreadCreate(osThread(myTask18), NULL);

  /* definition and creation of myTask19 */
  osThreadDef(myTask19, Data_Storage_Task, osPriorityNormal, 0, 128);
  myTask19Handle = osThreadCreate(osThread(myTask19), NULL);

  /* definition and creation of myTask20 */
  osThreadDef(myTask20, TiggerLogic_Task, osPriorityAboveNormal, 0, 128);
  myTask20Handle = osThreadCreate(osThread(myTask20), NULL);

  /* definition and creation of myTask21 */
  osThreadDef(myTask21, Relay_1_Task, osPriorityNormal, 0, 128);
  myTask21Handle = osThreadCreate(osThread(myTask21), NULL);

  /* definition and creation of myTask22 */
  osThreadDef(myTask22, Relay_2_Task, osPriorityNormal, 0, 128);
  myTask22Handle = osThreadCreate(osThread(myTask22), NULL);

  /* definition and creation of myTask23 */
  osThreadDef(myTask23, HART_Receive_Task, osPriorityNormal, 0, 128);
  myTask23Handle = osThreadCreate(osThread(myTask23), NULL);

  /* definition and creation of myTask24 */
  osThreadDef(myTask24, HART_Transmit_Task, osPriorityNormal, 0, 128);
  myTask24Handle = osThreadCreate(osThread(myTask24), NULL);

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
	float32_t temp_mean_acceleration_4_20 = 0.0;	
	float32_t temp_max_acceleration_icp = 0.0;	
	float32_t temp_min_acceleration_icp = 0.0;	
	uint32_t index;	
	float32_t constant_voltage;
	
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

		//Усредняем постоянку ICP
		arm_rms_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&constant_voltage );

		
		//Фильтр НЧ
		arm_biquad_cascade_df1_f32(&filter_main_low_icp, (float32_t*) &float_adc_value_ICP[0], (float32_t*) &float_adc_value_ICP[0], ADC_BUFFER_SIZE);								
		arm_biquad_cascade_df1_f32(&filter_main_low_4_20, (float32_t*) &float_adc_value_4_20[0], (float32_t*) &float_adc_value_4_20[0], ADC_BUFFER_SIZE);			
		
		//Фильтр ВЧ
		arm_biquad_cascade_df1_f32(&filter_main_high_icp, (float32_t*) &float_adc_value_ICP[0], (float32_t*) &float_adc_value_ICP[0], ADC_BUFFER_SIZE);		
				
		//СКЗ
		arm_rms_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_rms_acceleration_icp );
		arm_rms_f32( (float32_t*)&float_adc_value_4_20[0], ADC_BUFFER_SIZE, (float32_t*)&temp_mean_acceleration_4_20 );
		
		//Max
		arm_max_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_max_acceleration_icp, &index );
				
		//Min
		arm_min_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_min_acceleration_icp, &index );
		
		
		xQueueSend(acceleration_queue_icp, (void*)&temp_rms_acceleration_icp, 0);				
		xQueueSend(queue_4_20, (void*)&temp_mean_acceleration_4_20, 0);		
		

		//Детектор обрыва ICP
		if ( constant_voltage > 4000 && (icp_voltage * 0.001) <  break_level_icp ) break_sensor_icp = 0;
		else break_sensor_icp = 1;

		//Детектор обрыва 4-20
		if ( (temp_mean_acceleration_4_20 * COEF_TRANSFORM_4_20 * coef_ampl_420 + coef_offset_420) < break_level_420 ) break_sensor_420 = 0;
		else break_sensor_420 = 1;
		
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
	float32_t temp_max_velocity_icp = 0.0;	
	float32_t temp_min_velocity_icp = 0.0;

			
	uint32_t index;
	
  
	/* Infinite loop */
  for(;;)
  {
    xSemaphoreTake( Semaphore_Velocity, portMAX_DELAY );
		
				
		//Интегратор
		Integrate( (float32_t*)&float_adc_value_ICP[0], (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, filter_instance_highpass_1_icp );
								
		//Фильтр ВЧ (highpass)
		arm_biquad_cascade_df1_f32(&filter_instance_highpass_1_icp, (float32_t*) &float_adc_value_ICP[0], (float32_t*) &float_adc_value_ICP[0], ADC_BUFFER_SIZE);		
		
				
		//СКЗ
		arm_rms_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_rms_velocity_icp );
		
		//Max
		arm_max_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_max_velocity_icp, &index );				
		//Min
		arm_min_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_min_velocity_icp, &index );
		
		
		
		xQueueSend(velocity_queue_icp, (void*)&temp_rms_velocity_icp, 0);		
		
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
	float32_t temp_max_displacement_icp = 0.0;		
	float32_t temp_min_displacement_icp = 0.0;	
			
	uint32_t index;	
	
  /* Infinite loop */
  for(;;)
  {
    xSemaphoreTake( Semaphore_Displacement, portMAX_DELAY );				
		
		
		//Интегратор			
		Integrate( (float32_t*)&float_adc_value_ICP[0], (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, filter_instance_highpass_2_icp );		
				
		//Фильтр ВЧ
		arm_biquad_cascade_df1_f32(&filter_instance_highpass_2_icp, (float32_t*) &float_adc_value_ICP[0], (float32_t*) &float_adc_value_ICP[0], ADC_BUFFER_SIZE);		
		
		
		//СКЗ
		arm_rms_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_rms_displacement_icp );								
		
		//Max
		arm_max_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_max_displacement_icp, &index );
				
		//Min
		arm_min_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_min_displacement_icp, &index );
				
								
		xQueueSend(displacement_queue_icp, (void*)&temp_rms_displacement_icp, 0);
		
		
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
					
					icp_voltage = rms_acceleration_icp * 0.001;
					
					rms_acceleration_icp *= (float32_t) COEF_TRANSFORM_icp * coef_ampl_icp + coef_offset_icp;

			}
				
				
				
			queue_count_A_4_20 = uxQueueMessagesWaiting(queue_4_20);		

			if (queue_count_A_4_20 == QUEUE_LENGHT_4_20)
			{						
					mean_4_20 = 0.0;		
								
					for (uint16_t i=0; i<QUEUE_LENGHT_4_20; i++)
					{
							xQueueReceive(queue_4_20, (void *) &Q_A_mean_array_4_20[i], 0);										
					}
					
					arm_rms_f32((float32_t*) &Q_A_mean_array_4_20, QUEUE_LENGHT_4_20, (float32_t*)&mean_4_20);	
															
					mean_4_20 = (float32_t) (mean_4_20 * COEF_TRANSFORM_4_20 * coef_ampl_420 + coef_offset_420);
					
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
						
					rms_velocity_icp *= (float32_t) COEF_TRANSFORM_icp * coef_ampl_icp + coef_offset_icp;
					
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

					rms_displacement_icp *= (float32_t) COEF_TRANSFORM_icp * coef_ampl_icp + coef_offset_icp;					
			}				
  }
  /* USER CODE END Q_Average_D */
}

/* ADC_supply_voltage function */
void ADC_supply_voltage(void const * argument)
{
  /* USER CODE BEGIN ADC_supply_voltage */
	
	volatile uint16_t supply_voltage = 0;
  /* Infinite loop */
  for(;;)
  {
		HAL_ADCEx_InjectedStart(&hadc1);
		HAL_ADCEx_InjectedPollForConversion(&hadc1, 100);
		supply_voltage = HAL_ADCEx_InjectedGetValue(&hadc1, 1);
		HAL_ADCEx_InjectedStop(&hadc1);
	
		power_supply_voltage = (float32_t) supply_voltage * COEF_TRANSFORM_SUPPLY;
								
    osDelay(100);
  }
  /* USER CODE END ADC_supply_voltage */
}

/* Lights_Task function */
void Lights_Task(void const * argument)
{
  /* USER CODE BEGIN Lights_Task */
  /* Infinite loop */
  for(;;)
  {
		//Прогрев
		if (warming_flag == 1) 
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
			osDelay(150);						
		}
		else
		{	
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0)
			{					
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			}
						
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 1 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0)
			{								
				//Синий 
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				
				osDelay(500);
				
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				
				osDelay(500);
				
			}			
			
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 1)
			{				
				//Красный 
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				
				osDelay(200);
				
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				
				osDelay(200);
			}
			
			
			if (break_sensor_icp == 0 || break_sensor_420 == 0)
			{				
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				
				osDelay(500);
				
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				
				osDelay(500);
				
			}	
			
			osDelay(100);			
		}

	

  }
  /* USER CODE END Lights_Task */
}

/* DAC_Task function */
void DAC_Task(void const * argument)
{
  /* USER CODE BEGIN DAC_Task */
	uint32_t out_dac = 0.0;
	float32_t a_to_v = 0.0;
	float32_t variable_485 = 0.0;
	
  /* Infinite loop */
  for(;;)
  {
		
		
		//Источник сигнала ICP
		if (settings[89] == 0)
		{
			out_required_current = rms_velocity_icp / (range_out_420 / 16.0) + 4;		
		}
		
		//Источник сигнала 4-20
		if (settings[89] == 1)
		{
			out_required_current = mean_4_20;
		}
		
		//Источник сигнала 485
		if (settings[89] == 2)
		{
			//variable_485 = convert_hex_to_float(&settings[0], 71); 	
			//out_required_current = variable_485 / (range_out_420 / 16.0) + 4;		
			out_required_current = mb_master_recieve_data / (range_out_420 / 16.0) + 4;		
			
		}		
		
		a_to_v = (float32_t) out_required_current * (3.3 / 21.45); 
		
		out_dac = a_to_v * 4096 / 3.3;
		
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, out_dac);
		
    osDelay(100);
  }
  /* USER CODE END DAC_Task */
}

/* Display_Task function */
void Display_Task(void const * argument)
{
  /* USER CODE BEGIN Display_Task */
	uint8_t temp_stat = 0;
	char buffer[64];
	// CS# (This pin is the chip select input. (active LOW))
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	
	ssd1306_Init();
	
  /* Infinite loop */
  for(;;)
  {		
		
			if (button_down_pressed_in == 1 && menu_index_pointer < 3) 
			{				
				menu_index_pointer ++; 
				button_down_pressed_in = 0;
				osDelay(300);
			}
				
			if (button_left_pressed_in == 1 && menu_index_pointer > 0) 
			{				
				menu_index_pointer --;
				button_left_pressed_in = 0;
				osDelay(300);
			}				
			
			if (menu_index_pointer > 10 && (button_down_pressed_in || button_left_pressed_in)) menu_index_pointer = 0;
			
			
			if (menu_index_pointer == 0)
			{
				ssd1306_Fill(0);
				ssd1306_SetCursor(0,0);
				ssd1306_WriteString("Ускоре",font_8x15_RU,1);
				ssd1306_WriteString("-",font_8x14,1);
				ssd1306_SetCursor(0,15);				
				ssd1306_WriteString("ние",font_8x15_RU,1);
				ssd1306_WriteString(" ICP",font_8x14,1);
				
				ssd1306_SetCursor(0,33);				
				snprintf(buffer, sizeof buffer, "%0.03f", rms_acceleration_icp);				
				ssd1306_WriteString(buffer,font_8x14,1);	
				ssd1306_UpdateScreen();				
			}
			
			if (menu_index_pointer == 1)
			{
				ssd1306_Fill(0);
				ssd1306_SetCursor(0,0);
				ssd1306_WriteString("Скоро",font_8x15_RU,1);
				ssd1306_WriteString("-",font_8x14,1);
				ssd1306_SetCursor(0,15);				
				ssd1306_WriteString("сть",font_8x15_RU,1);
				ssd1306_WriteString(" ICP",font_8x14,1);
				
				ssd1306_SetCursor(0,33);				
				snprintf(buffer, sizeof buffer, "%f", rms_velocity_icp);
				ssd1306_WriteString(buffer,font_8x14,1);	
				ssd1306_UpdateScreen();								
			}
			
			if (menu_index_pointer == 2)
			{
				ssd1306_Fill(0);
				ssd1306_SetCursor(0,0);
				ssd1306_WriteString("Переме",font_8x15_RU,1);
				ssd1306_WriteString("-",font_8x14,1);
				ssd1306_SetCursor(0,15);				
				ssd1306_WriteString("щен",font_8x15_RU,1);
				ssd1306_WriteString(".ICP",font_8x14,1);
				
				ssd1306_SetCursor(0,33);				
				snprintf(buffer, sizeof buffer, "%f", rms_displacement_icp);
				ssd1306_WriteString(buffer,font_8x14,1);	
				ssd1306_UpdateScreen();								
			}
			
			if (menu_index_pointer == 3)
			{
				ssd1306_Fill(0);
				ssd1306_SetCursor(0,0);
				ssd1306_WriteString("Загруз",font_8x15_RU,1);
				ssd1306_WriteString("-",font_8x14,1);
				ssd1306_SetCursor(0,15);				
				ssd1306_WriteString("ка",font_8x15_RU,1);
				ssd1306_WriteString(" CPU",font_8x14,1);
				
				ssd1306_SetCursor(0,33);				
				snprintf(buffer, sizeof buffer, "%0.02f %%", cpu_float);
				ssd1306_WriteString(buffer,font_8x14,1);	
				ssd1306_UpdateScreen();								
			}
			
			if (button_center_pressed_in_short == 1)
			{
				ssd1306_Fill(0);
				ssd1306_SetCursor(0,0);
				ssd1306_WriteString("Ввод",font_8x15_RU,1);				
								
				ssd1306_UpdateScreen();
				
				menu_index_pointer = 98;								
			}
			
			if (button_center_pressed_in_long == 1)
			{				
					ssd1306_Fill(0);					
					ssd1306_SetCursor(0,0);
					ssd1306_WriteString("Настр",font_8x15_RU,1);				
					ssd1306_WriteString(".",font_8x14,1);					
					ssd1306_SetCursor(0,15);												
					ssd1306_WriteString("сохра",font_8x15_RU,1);				
					ssd1306_WriteString("-",font_8x14,1);
					ssd1306_SetCursor(0,30);				
					ssd1306_WriteString("нены",font_8x15_RU,1);					
					ssd1306_UpdateScreen();
										
					menu_index_pointer = 99;											
			}
			
			if ( (break_sensor_icp == 0 || break_sensor_420 == 0) && warming_flag == 0)
			{				
				ssd1306_Fill(0);
				ssd1306_SetCursor(0,0);
				ssd1306_WriteString("Обрыв",font_8x15_RU,1);				
				ssd1306_SetCursor(0,15);				
				ssd1306_WriteString("датчика",font_8x15_RU,1);
				ssd1306_SetCursor(0,30);	
				//ssd1306_WriteString("ICP 4-20",font_8x14,1);
				ssd1306_UpdateScreen();
				
				menu_index_pointer = 100;
			}
			
	
			osDelay(100);
  }
  /* USER CODE END Display_Task */
}

/* Button_Task function */
void Button_Task(void const * argument)
{
  /* USER CODE BEGIN Button_Task */
	uint8_t prev_state = 0;
  /* Infinite loop */
  for(;;)
  {
					
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0)
		{
			button_left ++;
			
			if ( button_left > 7 ) 
			{
				button_left_pressed_in = 1;				
				button_right_pressed_in = 0;
				button_up_pressed_in = 0;
				button_down_pressed_in = 0;
				button_center_pressed_in_short = 0;
				button_center_pressed_in_long = 0;
				button_center = 0;
				
				button_left = 0;
			}			
		}		
		
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == 0)
		{
			button_right ++;
			
			if ( button_right > 7 ) 
			{
				button_left_pressed_in = 0;				
				button_right_pressed_in = 1;
				button_up_pressed_in = 0;
				button_down_pressed_in = 0;
				button_center_pressed_in_short = 0;
				button_center_pressed_in_long = 0;
				button_center = 0;
				
				button_right = 0;
			}			
		}		
		
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0)
		{
			button_up ++;
			
			if ( button_up > 7 ) 
			{
				button_left_pressed_in = 0;				
				button_right_pressed_in = 0;
				button_up_pressed_in = 1;
				button_down_pressed_in = 0;
				button_center_pressed_in_short = 0;
				button_center_pressed_in_long = 0;
				button_center = 0;
				
				button_up = 0;
			}
		}		
		
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0)
		{
			button_down ++;
			
			if ( button_down > 7 ) 
			{
				button_left_pressed_in = 0;				
				button_right_pressed_in = 0;
				button_up_pressed_in = 0;
				button_down_pressed_in = 1;
				button_center_pressed_in_short = 0;
				button_center_pressed_in_long = 0;
				button_center = 0;
				
				button_down = 0;
			}
		}		
		
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0)
		{
			button_center ++;	
			
			if ( button_center > 70 ) 
			{
				button_left_pressed_in = 0;				
				button_right_pressed_in = 0;
				button_up_pressed_in = 0;
				button_down_pressed_in = 0;
				button_center_pressed_in_short = 0;
				button_center_pressed_in_long = 1;
				
				button_center = 0;
			}
		}	
		
		if ( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 1 && button_center > 5 && button_center < 30 && button_center_pressed_in_long == 0) 
		{
				button_left_pressed_in = 0;				
				button_right_pressed_in = 0;
				button_up_pressed_in = 0;
				button_down_pressed_in = 0;
				button_center_pressed_in_short = 1;
				button_center_pressed_in_long = 0;
				
				button_center = 0;
		}		
		
    osDelay(20);
  }
  /* USER CODE END Button_Task */
}

/* Modbus_Receive_Task function */
void Modbus_Receive_Task(void const * argument)
{
  /* USER CODE BEGIN Modbus_Receive_Task */
		
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_Modbus_Rx, portMAX_DELAY );					
				
		__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_IDLEF); 				
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
		
		HAL_UART_DMAStop(&huart2); 
		
		HAL_UART_Receive_DMA(&huart2, receiveBuffer, 16);					
		
		
		
		xSemaphoreGive( Semaphore_Modbus_Tx );
    
  }
  /* USER CODE END Modbus_Receive_Task */
}

/* Modbus_Transmit_Task function */
void Modbus_Transmit_Task(void const * argument)
{
  /* USER CODE BEGIN Modbus_Transmit_Task */
	uint16_t crc = 0;
	volatile uint16_t count_registers = 0;
	volatile uint16_t adr_of_registers = 0;
	volatile uint16_t recieve_calculated_crc = 0;
	volatile uint16_t recieve_actual_crc = 0;
	volatile uint16_t outer_register = 0;
	
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_Modbus_Tx, portMAX_DELAY );
		
		//for (int i = 0; i < REG_COUNT*2+5; i++) transmitBuffer[i] = 0;
		
		if (receiveBuffer[0] == SLAVE_ADR)
		{		
				recieve_calculated_crc = crc16(receiveBuffer, 6);
				recieve_actual_crc = (receiveBuffer[7] << 8) + receiveBuffer[6];
				
				//Если 16 функция, другая длина пакета
				if (receiveBuffer[1] == 0x10) 
				{
					recieve_calculated_crc = crc16(receiveBuffer, 11);
					recieve_actual_crc = (receiveBuffer[12] << 8) + receiveBuffer[11];
				}
				
				//Проверяем crc
				if (recieve_calculated_crc == recieve_actual_crc) 
				{	
						transmitBuffer[0] = receiveBuffer[0]; //адрес устр-ва			
						transmitBuffer[1] = receiveBuffer[1]; //номер функции						
					
						adr_of_registers = (receiveBuffer[2] << 8) + receiveBuffer[3];//получаем адрес регистра				
						count_registers = (receiveBuffer[4] << 8) + receiveBuffer[5]; //получаем кол-во регистров из запроса
						outer_register = adr_of_registers + count_registers; //крайний регистр
						
						transmitBuffer[2] = count_registers*2; //количество байт	(в два раза больше чем регистров)	
					
					
						//Проверяем номер регистра
						if (adr_of_registers > REG_COUNT) 
						{
									if (transmitBuffer[1] == 0x3) transmitBuffer[1] = 0x83; //Function Code in Exception Response
									if (transmitBuffer[1] == 0x4) transmitBuffer[1] = 0x84; //Function Code in Exception Response
									
									transmitBuffer[2] = 0x02; //Exception "Illegal Data Address"		
									
									crc = crc16(transmitBuffer, 3);
							
									transmitBuffer[3] = crc;
									transmitBuffer[4] = crc >> 8;		 
							
									HAL_UART_Transmit_DMA(&huart2, transmitBuffer, 5);									
						}					
						
						if (receiveBuffer[1] == 0x03 || receiveBuffer[1] == 0x04) //Holding Register (FC=03) or Input Register (FC=04)
						{		
									if (adr_of_registers < 125) //если кол-во регистров больше 125 (255 байт макс.), опрос идет несколькими запросами 
									{							
											for (uint16_t i=adr_of_registers; i < outer_register; i++)
											{
												transmitBuffer[i*2+3] = settings[i] >> 8; //значение регистра Lo 		
												transmitBuffer[i*2+4] = settings[i] & 0x00FF; //значение регистра Hi		
											}
									
											crc = crc16(transmitBuffer, count_registers*2+3);				
									
											transmitBuffer[count_registers*2+3] = crc;
											transmitBuffer[count_registers*2+3+1] = crc >> 8;		
																				
												
											HAL_UART_Transmit_DMA(&huart2, transmitBuffer, count_registers*2+5);
									}
									else
									{
											for (uint16_t i=0; i < count_registers; i++)
											{
												transmitBuffer[i*2+3] = settings[adr_of_registers + i] >> 8; //значение регистра Lo 		
												transmitBuffer[i*2+4] = settings[adr_of_registers + i] & 0x00FF; //значение регистра Hi		
											}
									
											crc = crc16(transmitBuffer, count_registers*2+3);				
									
											transmitBuffer[count_registers*2+3] = crc;
											transmitBuffer[count_registers*2+3+1] = crc >> 8;		
																				
															
											HAL_UART_Transmit_DMA(&huart2, transmitBuffer, count_registers*2+5);					
									}
						}							
						else if (receiveBuffer[1] == 0x06) //Preset Single Register (FC=06)
						{									
							
									settings[adr_of_registers] = (receiveBuffer[4] << 8) + receiveBuffer[5]; 										

									transmitBuffer[2] = receiveBuffer[2];
									transmitBuffer[3] = receiveBuffer[3];
							
									transmitBuffer[4] = receiveBuffer[4];
									transmitBuffer[5] = receiveBuffer[5];
							
									crc = crc16(transmitBuffer, 6);				
							
									transmitBuffer[6] = crc;
									transmitBuffer[7] = crc >> 8;		
																		
							
									HAL_UART_Transmit_DMA(&huart2, transmitBuffer, 8);						
						}				
						else if (receiveBuffer[1] == 0x10) //Preset Multiply Registers (FC=16)
						{									
							
									settings[adr_of_registers] = (receiveBuffer[7] << 8) + receiveBuffer[8]; 										
									settings[adr_of_registers+1] = (receiveBuffer[9] << 8) + receiveBuffer[10];
									

									transmitBuffer[2] = receiveBuffer[2];//адрес первого регистра
									transmitBuffer[3] = receiveBuffer[3];
							
									transmitBuffer[4] = receiveBuffer[4];//кол-во регистров	
									transmitBuffer[5] = receiveBuffer[5];
								
							
									crc = crc16(transmitBuffer, 6);				
							
									transmitBuffer[6] = crc;
									transmitBuffer[7] = crc >> 8;		
									
							
									HAL_UART_Transmit_DMA(&huart2, transmitBuffer, 8);						
						}
						else
						{							
									transmitBuffer[1] = 0x81; //Function Code in Exception Response
									transmitBuffer[2] = 0x01; //Exception "Illegal function"			
									
									crc = crc16(transmitBuffer, 3);
							
									transmitBuffer[3] = crc;
									transmitBuffer[4] = crc >> 8;		 
							
									HAL_UART_Transmit_DMA(&huart2, transmitBuffer, 5);
						}					
						
				}
		}
		
    
  }
  /* USER CODE END Modbus_Transmit_Task */
}

/* Master_Modbus_Receive function */
void Master_Modbus_Receive(void const * argument)
{
  /* USER CODE BEGIN Master_Modbus_Receive */
	uint16_t f_number = 0;
	volatile uint16_t byte_number = 0;
	uint16_t temp_data[2];
	uint16_t calculated_crc = 0;
	uint16_t actual_crc = 0;
	
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_Master_Modbus_Rx, portMAX_DELAY );			
		
		mb_master_recieve_data = 0.0;
				
		__HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_IDLEF); 				
		__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
		
		HAL_UART_DMAStop(&huart3); 
		
		HAL_UART_Receive_DMA(&huart3, master_receiveBuffer, quantity_reg_mb_master*2+5); //// !!!! Ожидаемое (DMA) количество байт (13 - для опроса 4 регистров) 				
		
		if (master_receiveBuffer[0] == slave_adr_mb_master)
		{						
				f_number = master_receiveBuffer[1]; //номер функции	
				byte_number = master_receiveBuffer[2];//кол-во байт
						
			
				//считаем crc
				calculated_crc = crc16(master_receiveBuffer, 3 + byte_number);										
				actual_crc = master_receiveBuffer[3 + byte_number];
				actual_crc += master_receiveBuffer[3 + byte_number + 1] << 8;
				
				if (calculated_crc == actual_crc)
				{
						if (master_receiveBuffer[1] == 0x03 || master_receiveBuffer[1] == 0x04) //Holding Register (FC=03)
						{	
							
//								for (int i = 3; i < byte_number+2; i=i+2)
//								{
//									settings[START_REG_ADR_MB_MASTER+i-3] = ( master_receiveBuffer[i] << 8 ) + master_receiveBuffer[i+1];																									
//								}
							
									temp_data[0] = ( master_receiveBuffer[mb_master_numreg_1*2+3] << 8 ) + master_receiveBuffer[mb_master_numreg_1*2+4];
									temp_data[1] = ( master_receiveBuffer[mb_master_numreg_1*2+5] << 8 ) + master_receiveBuffer[mb_master_numreg_1*2+6];							
									mb_master_recieve_data_1 = convert_hex_to_float(&temp_data[0], 0);
							
									mb_master_recieve_data_2 = ( master_receiveBuffer[mb_master_numreg_2*2+3] << 8 ) + master_receiveBuffer[mb_master_numreg_2*2+4];
									mb_master_recieve_data_3 = ( master_receiveBuffer[mb_master_numreg_3*2+3] << 8 ) + master_receiveBuffer[mb_master_numreg_3*2+4];		
									mb_master_recieve_data_4 = ( master_receiveBuffer[mb_master_numreg_4*2+3] << 8 ) + master_receiveBuffer[mb_master_numreg_4*2+4];
							
								
						}				

				}
			
		}
			
    
  }
  /* USER CODE END Master_Modbus_Receive */
}

/* Master_Modbus_Transmit function */
void Master_Modbus_Transmit(void const * argument)
{
  /* USER CODE BEGIN Master_Modbus_Transmit */
	uint16_t crc = 0;
  /* Infinite loop */
  for(;;)
  {
		
		master_transmitBuffer[0] = slave_adr_mb_master;
		master_transmitBuffer[1] = slave_func_mb_master;
		master_transmitBuffer[2] = slave_reg_mb_master >> 8;
		master_transmitBuffer[3] = slave_reg_mb_master & 0x00FF;
		master_transmitBuffer[4] = quantity_reg_mb_master >> 8;
		master_transmitBuffer[5] = quantity_reg_mb_master & 0x00FF;		
		
		crc = crc16(master_transmitBuffer, 6);				
					
		master_transmitBuffer[6] = crc;
		master_transmitBuffer[7] = crc >> 8;	
		
		HAL_UART_Transmit_DMA(&huart3, master_transmitBuffer, 8);
		
    osDelay(mb_master_timeout);
  }
  /* USER CODE END Master_Modbus_Transmit */
}

/* Data_Storage_Task function */
void Data_Storage_Task(void const * argument)
{
  /* USER CODE BEGIN Data_Storage_Task */
	uint16_t temp[2];
	volatile uint8_t st_flash = 0;
  /* Infinite loop */
  for(;;)
  {
		
		convert_float_and_swap(icp_voltage, &temp[0]);				
		settings[0] = temp[0];
		settings[1] = temp[1];

		settings[10] = break_sensor_icp;
		
		convert_float_and_swap(rms_acceleration_icp, &temp[0]);		
		settings[22] = temp[0];
		settings[23] = temp[1];			
		convert_float_and_swap(rms_velocity_icp, &temp[0]);
		settings[24] = temp[0];
		settings[25] = temp[1];	
		convert_float_and_swap(rms_displacement_icp, &temp[0]);
		settings[26] = temp[0];
		settings[27] = temp[1];

		
		convert_float_and_swap(mean_4_20, &temp[0]);		
		settings[36] = temp[0];
		settings[37] = temp[1];
		
		settings[46] = break_sensor_420;
		
//		convert_float_and_swap(mb_master_recieve_data, &temp[0]);		
//		settings[71] = temp[0];
//		settings[72] = temp[1];

		settings[82] = state_warning_relay;
		settings[83] = state_emerg_relay;

		convert_float_and_swap(power_supply_voltage, &temp[0]);		
		settings[98] = temp[0];
		settings[99] = temp[1];

		convert_float_and_swap(cpu_float, &temp[0]);		
		settings[103] = temp[0];
		settings[104] = temp[1];

		convert_float_and_swap(mb_master_recieve_data_1, &temp[0]);		
		settings[116] = temp[0];
		settings[117] = temp[1];
		
		settings[128] = mb_master_recieve_data_2;
		settings[140] = mb_master_recieve_data_3;
		settings[152] = mb_master_recieve_data_4;		

		mb_master_recieve_value_1 = mb_master_recieve_data_1;
		mb_master_recieve_value_2 = (float32_t) mb_master_recieve_data_2 / 100;			
		mb_master_recieve_value_3 = (float32_t) mb_master_recieve_data_3 / 100;			
		mb_master_recieve_value_4 = (float32_t) mb_master_recieve_data_4 / 100;	


		//Применение/запись настроек
		if (settings[107] == 0xABCD)
		{
			settings[107] = 0x0;
						
			st_flash = write_registers_to_flash(settings);			
			
			NVIC_SystemReset();			
		}
		
		//Сброс настроек
		if (settings[108] == 0xDCBA)
		{
			settings[108] = 0x0;
			
			for(int i=0; i< REG_COUNT; i++)
				settings[i] = default_settings[i];
			
						
			st_flash = write_registers_to_flash(settings);	

			NVIC_SystemReset();	
		}

    osDelay(100);
  }
  /* USER CODE END Data_Storage_Task */
}

/* TiggerLogic_Task function */
void TiggerLogic_Task(void const * argument)
{
  /* USER CODE BEGIN TiggerLogic_Task */
	
	
	osDelay(warming_up);
	warming_flag = 0;
	
  /* Infinite loop */
  for(;;)
  {
		//Режим работы "без памяти"
		if (mode_relay == 0 && warming_flag == 0)
		{			
				//Источник сигнала ICP
				if (source_signal_relay == 0)
				{
						//Предупр. реле
						if ( rms_velocity_icp >= lo_warning_icp && rms_velocity_icp < hi_warning_icp ) 
						{							
							state_warning_relay = 1;							
							xSemaphoreGive( Semaphore_Relay_1 );
						}
						else
						{							
							state_warning_relay = 0;							
							xSemaphoreGive( Semaphore_Relay_1 );
						}
						
						//Авар. реле
						if ( rms_velocity_icp >= lo_emerg_icp && rms_velocity_icp <= hi_emerg_icp ) 
						{								
							state_emerg_relay = 1;							
							xSemaphoreGive( Semaphore_Relay_2 );
						}
						else
						{							
							state_emerg_relay = 0;							
							xSemaphoreGive( Semaphore_Relay_2 );							
						}
				}
				
				//Источник сигнала 4-20
				if (source_signal_relay == 1)
				{							
						if ( mean_4_20 >= lo_warning_420 && mean_4_20 < hi_warning_420 ) 
						{							
							state_warning_relay = 1;							
							xSemaphoreGive( Semaphore_Relay_1 );
						}
						else
						{							
							state_warning_relay = 0;
							xSemaphoreGive( Semaphore_Relay_1 );
						}
						
						if ( mean_4_20 >= lo_emerg_420 && mean_4_20 <= hi_emerg_420 ) 
						{							
							state_emerg_relay = 1;							
							xSemaphoreGive( Semaphore_Relay_2 );							
						}
						else
						{							
							state_emerg_relay = 0;
							xSemaphoreGive( Semaphore_Relay_2 );							
						}
				}
				
				//Источник сигнала 485 (Modbus)
				if (source_signal_relay == 2)
				{					
						if ( 	(mb_master_recieve_value_1 >= mb_master_lo_warning_485_1 && mb_master_recieve_value_1 < mb_master_hi_warning_485_1) ||
									(mb_master_recieve_value_2 >= mb_master_lo_warning_485_2 && mb_master_recieve_value_2 < mb_master_hi_warning_485_2) ||
									(mb_master_recieve_value_3 >= mb_master_lo_warning_485_3 && mb_master_recieve_value_3 < mb_master_hi_warning_485_3) ||
									(mb_master_recieve_value_4 >= mb_master_lo_warning_485_4 && mb_master_recieve_value_4 < mb_master_hi_warning_485_4) ) 
						{							
							state_warning_relay = 1;							
							xSemaphoreGive( Semaphore_Relay_1 );							
						}
						else
						{							
							state_warning_relay = 0;
							xSemaphoreGive( Semaphore_Relay_1 );							
						}
						
						if ( 	(mb_master_recieve_value_1 >= mb_master_lo_emerg_485_1 && mb_master_recieve_value_1 < mb_master_hi_emerg_485_1) ||
									(mb_master_recieve_value_2 >= mb_master_lo_emerg_485_2 && mb_master_recieve_value_2 < mb_master_hi_emerg_485_2) ||
									(mb_master_recieve_value_3 >= mb_master_lo_emerg_485_3 && mb_master_recieve_value_3 < mb_master_hi_emerg_485_3) ||
									(mb_master_recieve_value_4 >= mb_master_lo_emerg_485_4 && mb_master_recieve_value_4 < mb_master_hi_emerg_485_4) ) 									  
						{							
							state_emerg_relay = 1;							
							xSemaphoreGive( Semaphore_Relay_2 );							
						}
						else
						{						
							state_emerg_relay = 0;
							xSemaphoreGive( Semaphore_Relay_2 );
						}
				}
		}
		
		
		//Режим работы "с памятью"
		if (mode_relay == 1 && warming_flag == 0)
		{			
				//Источник сигнала ICP
				if (source_signal_relay == 0)
				{
					
						if ( rms_velocity_icp >= lo_warning_icp && rms_velocity_icp < hi_warning_icp ) 
						{							
							state_warning_relay = 1;														
							xSemaphoreGive( Semaphore_Relay_1 );							
						}

						
						if ( rms_velocity_icp >= lo_emerg_icp && rms_velocity_icp <= hi_emerg_icp ) 
						{							
							state_emerg_relay = 1;							
							xSemaphoreGive( Semaphore_Relay_2 );							
						}

				}
				
				//Источник сигнала 4-20
				if (source_signal_relay == 1)
				{							
						if ( mean_4_20 >= lo_warning_420 && mean_4_20 < hi_warning_420 ) 
						{							
							state_warning_relay = 1;							
							xSemaphoreGive( Semaphore_Relay_1 );							
						}

						
						if ( mean_4_20 >= lo_emerg_420 && mean_4_20 <= hi_emerg_420 ) 
						{							
							state_emerg_relay = 1;							
							xSemaphoreGive( Semaphore_Relay_2 );							
						}

				}
				
				//Источник сигнала 485
				if (source_signal_relay == 2)
				{							
						if ( 	(mb_master_recieve_value_1 >= mb_master_lo_warning_485_1 && mb_master_recieve_value_1 < mb_master_hi_warning_485_1) ||
									(mb_master_recieve_value_2 >= mb_master_lo_warning_485_2 && mb_master_recieve_value_2 < mb_master_hi_warning_485_2) ||
									(mb_master_recieve_value_3 >= mb_master_lo_warning_485_3 && mb_master_recieve_value_3 < mb_master_hi_warning_485_3) ||
									(mb_master_recieve_value_4 >= mb_master_lo_warning_485_4 && mb_master_recieve_value_4 < mb_master_hi_warning_485_4) ) 
						{							
							state_warning_relay = 1;							
							xSemaphoreGive( Semaphore_Relay_1 );							
						}

						if ( 	(mb_master_recieve_value_1 >= mb_master_lo_emerg_485_1 && mb_master_recieve_value_1 < mb_master_hi_emerg_485_1) ||
									(mb_master_recieve_value_2 >= mb_master_lo_emerg_485_2 && mb_master_recieve_value_2 < mb_master_hi_emerg_485_2) ||
									(mb_master_recieve_value_3 >= mb_master_lo_emerg_485_3 && mb_master_recieve_value_3 < mb_master_hi_emerg_485_3) ||
									(mb_master_recieve_value_4 >= mb_master_lo_emerg_485_4 && mb_master_recieve_value_4 < mb_master_hi_emerg_485_4) ) 	  
						{							
							state_emerg_relay = 1;							
							xSemaphoreGive( Semaphore_Relay_2 );							
						}

				}
		}
		
		
		//Квитирование
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0 || settings[96] == 1) 
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			state_warning_relay = 0;
			
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
			state_emerg_relay = 0;
			
			settings[96] = 0;
		}
		
		//Контроль напряжения питания ПЛК (+24 )
		if (power_supply_voltage < power_supply_warning_lo || power_supply_voltage > power_supply_warning_hi)
		{
			state_warning_relay = 1;							
			xSemaphoreGive( Semaphore_Relay_1 );	
		}
		
		
    osDelay(50);
  }
  /* USER CODE END TiggerLogic_Task */
}

/* Relay_1_Task function */
void Relay_1_Task(void const * argument)
{
  /* USER CODE BEGIN Relay_1_Task */
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_Relay_1, portMAX_DELAY );		
		
    osDelay(delay_relay);
		
		if (warming_flag == 0 && state_warning_relay == 1)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);										
		}
		
		if (state_warning_relay == 0)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		}
		
  }
  /* USER CODE END Relay_1_Task */
}

/* Relay_2_Task function */
void Relay_2_Task(void const * argument)
{
  /* USER CODE BEGIN Relay_2_Task */
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_Relay_2, portMAX_DELAY );		
		
    osDelay(delay_relay);
		
		if (warming_flag == 0 && state_emerg_relay == 1)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);										
		}
		
		if (state_emerg_relay == 0)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		}   
		
  }
  /* USER CODE END Relay_2_Task */
}

/* HART_Receive_Task function */
void HART_Receive_Task(void const * argument)
{
  /* USER CODE BEGIN HART_Receive_Task */
	volatile uint8_t sta = 0;
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_HART_Receive, portMAX_DELAY );		
		
		__HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_IDLEF); 				
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		
		HAL_UART_DMAStop(&huart1); 				

		sta = HAL_UART_Receive_DMA(&huart1, HART_receiveBuffer, 16);							
		
		
		
		xSemaphoreGive( Semaphore_HART_Transmit );
		
    
  }
  /* USER CODE END HART_Receive_Task */
}

/* HART_Transmit_Task function */
void HART_Transmit_Task(void const * argument)
{
  /* USER CODE BEGIN HART_Transmit_Task */
	volatile uint8_t sta = 0;
	volatile uint16_t crc = 0;
	volatile uint16_t count_registers = 0;
	volatile uint16_t adr_of_registers = 0;
	volatile uint16_t recieve_calculated_crc = 0;
	volatile uint16_t recieve_actual_crc = 0;
	volatile uint16_t outer_register = 0;
	
  /* Infinite loop */
  for(;;)
  {   
		xSemaphoreTake( Semaphore_HART_Transmit, portMAX_DELAY );
		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);		
		osDelay(20);
				
		
		
		if (HART_receiveBuffer[0] == SLAVE_ADR)
		{		
				recieve_calculated_crc = crc16(HART_receiveBuffer, 6);
				recieve_actual_crc = (HART_receiveBuffer[7] << 8) + HART_receiveBuffer[6];
				
				//Если 16 функция, другая длина пакета
				if (HART_receiveBuffer[1] == 0x10) 
				{
					recieve_calculated_crc = crc16(HART_receiveBuffer, 11);
					recieve_actual_crc = (HART_receiveBuffer[12] << 8) + HART_receiveBuffer[11];
				}
				
				//Проверяем crc
				if (recieve_calculated_crc == recieve_actual_crc) 
				{	
						HART_transmitBuffer[0] = HART_receiveBuffer[0]; //адрес устр-ва			
						HART_transmitBuffer[1] = HART_receiveBuffer[1]; //номер функции						
					
						adr_of_registers = (HART_receiveBuffer[2] << 8) + HART_receiveBuffer[3];//получаем адрес регистра				
						count_registers = (HART_receiveBuffer[4] << 8) + HART_receiveBuffer[5]; //получаем кол-во регистров из запроса
						outer_register = adr_of_registers + count_registers; //крайний регистр
						
						HART_transmitBuffer[2] = count_registers*2; //количество байт	(в два раза больше чем регистров)	
					
					
						//Проверяем номер регистра
						if (adr_of_registers > REG_COUNT) 
						{
									if (HART_transmitBuffer[1] == 0x3) HART_transmitBuffer[1] = 0x83; //Function Code in Exception Response
									if (HART_transmitBuffer[1] == 0x4) HART_transmitBuffer[1] = 0x84; //Function Code in Exception Response
									
									HART_transmitBuffer[2] = 0x02; //Exception "Illegal Data Address"		
									
									crc = crc16(HART_transmitBuffer, 3);
							
									HART_transmitBuffer[3] = crc;
									HART_transmitBuffer[4] = crc >> 8;		 
							
									HAL_UART_Transmit_DMA(&huart2, HART_transmitBuffer, 5);									
						}					
						
						if (HART_receiveBuffer[1] == 0x03 || HART_receiveBuffer[1] == 0x04) //Holding Register (FC=03) or Input Register (FC=04)
						{		
									if (adr_of_registers < 125) 
									{							
											for (uint16_t i=adr_of_registers; i < outer_register; i++)
											{
												HART_transmitBuffer[i*2+3] = settings[i] >> 8; //значение регистра Lo 		
												HART_transmitBuffer[i*2+4] = settings[i] & 0x00FF; //значение регистра Hi		
											}
									
											crc = crc16(HART_transmitBuffer, count_registers*2+3);				
									
											HART_transmitBuffer[count_registers*2+3] = crc;
											HART_transmitBuffer[count_registers*2+3+1] = crc >> 8;		
																				
												
											HAL_UART_Transmit_DMA(&huart1, HART_transmitBuffer, count_registers*2+5);
											

									}									
									else //если кол-во регистров больше 125 (255 байт макс.), опрос идет несколькими запросами 
									{
											
											for (uint16_t i=0; i < count_registers; i++)
											{
												HART_transmitBuffer[i*2+3] = settings[adr_of_registers + i] >> 8; //значение регистра Lo 		
												HART_transmitBuffer[i*2+4] = settings[adr_of_registers + i] & 0x00FF; //значение регистра Hi		
											}
									
											crc = crc16(HART_transmitBuffer, count_registers*2+3);				
									
											HART_transmitBuffer[count_registers*2+3] = crc;
											HART_transmitBuffer[count_registers*2+3+1] = crc >> 8;		
																				
											//while (huart1.gState != HAL_UART_STATE_READY);
											
																							
											HAL_UART_Transmit_DMA(&huart1, HART_transmitBuffer, count_registers*2+5);					
											
											
									}
						}							
						else if (HART_receiveBuffer[1] == 0x06) //Preset Single Register (FC=06)
						{									
							
									settings[adr_of_registers] = (HART_receiveBuffer[4] << 8) + HART_receiveBuffer[5]; 										

									HART_transmitBuffer[2] = HART_receiveBuffer[2];
									HART_transmitBuffer[3] = HART_receiveBuffer[3];
							
									HART_transmitBuffer[4] = HART_receiveBuffer[4];
									HART_transmitBuffer[5] = HART_receiveBuffer[5];
							
									crc = crc16(HART_transmitBuffer, 6);				
							
									HART_transmitBuffer[6] = crc;
									HART_transmitBuffer[7] = crc >> 8;		
																		
							
									HAL_UART_Transmit_DMA(&huart1, HART_transmitBuffer, 8);			
									
									osDelay(count_registers*20);							
						}				
						else if (HART_receiveBuffer[1] == 0x10) //Preset Multiply Registers (FC=16)
						{									
							
									settings[adr_of_registers] = (HART_receiveBuffer[7] << 8) + HART_receiveBuffer[8]; 										
									settings[adr_of_registers+1] = (HART_receiveBuffer[9] << 8) + HART_receiveBuffer[10];
									

									HART_transmitBuffer[2] = HART_receiveBuffer[2];//адрес первого регистра
									HART_transmitBuffer[3] = HART_receiveBuffer[3];
							
									HART_transmitBuffer[4] = HART_receiveBuffer[4];//кол-во регистров	
									HART_transmitBuffer[5] = HART_receiveBuffer[5];
								
							
									crc = crc16(HART_transmitBuffer, 6);				
							
									HART_transmitBuffer[6] = crc;
									HART_transmitBuffer[7] = crc >> 8;		
									
							
									HAL_UART_Transmit_DMA(&huart1, HART_transmitBuffer, 8);				

									osDelay(count_registers*20);							
						}
						else
						{							
									HART_transmitBuffer[1] = 0x81; //Function Code in Exception Response
									HART_transmitBuffer[2] = 0x01; //Exception "Illegal function"			
									
									crc = crc16(HART_transmitBuffer, 3);
							
									HART_transmitBuffer[3] = crc;
									HART_transmitBuffer[4] = crc >> 8;		 
							
									HAL_UART_Transmit_DMA(&huart1, HART_transmitBuffer, 5);
						}					
						
				}
		}
		
		if (count_registers < 4) osDelay(200);
		else osDelay(count_registers*20);
				
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);	
		
		
		
  }
  /* USER CODE END HART_Transmit_Task */
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
		
		//Butterworth 3 Order, LowPass 100 Hz
		static float32_t coef_main_low_100_gain[] = {
                
			1*0.00014876521137360051,  2*0.00014876521137360051,  1*0.00014876521137360051,  1.9751611962490403,   -0.97575625709453451,        
			1*0.012123675033811735,  1*0.012123675033811735,  0*0.012123675033811735,  0.97575264993237654,  0                          
		};
		
		arm_biquad_cascade_df1_init_f32(&filter_main_low_4_20, 2, (float32_t *) &coef_main_low_100_gain[0], &pStates_main_low_4_20[0]);	
		
		//Butterworth 3 Order, HighPass 2 Hz
		static float32_t coef_main_highpass_2Hz_gain[] = {		
			1*0.99975456308379129,  -2*0.99975456308379129,  1*0.99975456308379129,    1.9995090057185783,   -0.99950924661658691,       
			1*0.99975462329351572,  -1*0.99975462329351572,  0*0.99975462329351572,    0.99950924658703155,  0                         
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
		
//		//Butterworth 4 Order, HighPass 30 Hz
//		static float32_t coef_main_highpass_30Hz_gain[] = {		                                           
//		
//			1*0.99717668761063039,  -2*0.99717668761063039,  1*0.99717668761063039,  1.9943263438323484,  -0.99438040661017335,        
//			1*0.99322993689598427,  -2*0.99322993689598427,  1*0.99322993689598427,  1.9864329493912707,  -0.98648679819266638        			
//		};
//		
//		//Butterworth 4 Order, HighPass 300 Hz
//		static float32_t coef_main_highpass_300Hz_gain[] = {		                                           
//			1*0.97130121510244805,  -2*0.97130121510244805,  1*0.97130121510244805,    1.9399670771829451,  -0.94523778322684693,        
//			1*0.9350918994360039,  -2*0.9350918994360039,  1*0.9350918994360039,    1.8676466896574162,  -0.87272090808659941  			
//		};
		

		if (FILTER_MODE == 1 || FILTER_MODE == 0) 		
		{
			arm_biquad_cascade_df1_init_f32(&filter_main_high_icp, 2, (float32_t *) &coef_main_highpass_2Hz_gain[0], &pStates_main_high_icp[0]);				
			arm_biquad_cascade_df1_init_f32(&filter_main_high_4_20, 2, (float32_t *) &coef_main_highpass_2Hz_gain[0], &pStates_main_high_4_20[0]);	
			
			
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
			
				
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_1_icp, 2, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_highpass_1_icp[0]);							
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_1_4_20, 2, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_highpass_1_4_20[0]);	
				
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_2_icp, 2, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_highpass_2_icp[0]);				
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_2_4_20, 2, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_highpass_2_4_20[0]);	
		}	
		
//		arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_3_icp, 2, (float32_t *) &coef_main_highpass_30Hz_gain[0], &pStates_highpass_3_icp[0]);							
//		arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_4_icp, 2, (float32_t *) &coef_main_highpass_300Hz_gain[0], &pStates_highpass_4_icp[0]);							
		
}



/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
