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
#include <string.h>


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
									Semaphore_HART_Receive, Semaphore_HART_Transmit,
									Mutex_Setting;

//float32_t sinus[ADC_BUFFER_SIZE];
uint16_t raw_adc_value[RAW_ADC_BUFFER_SIZE];
float32_t float_adc_value_ICP[ADC_BUFFER_SIZE];
float32_t float_adc_value_4_20[ADC_BUFFER_SIZE];

float32_t dac_voltage = 0.0;

float32_t mean_4_20 = 0.0;

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


float32_t min_4_20 = 0.0;
float32_t max_4_20 = 0.0;

float32_t Q_A_peak_array_icp[QUEUE_LENGHT];
float32_t Q_V_peak_array_icp[QUEUE_LENGHT];
float32_t Q_D_peak_array_icp[QUEUE_LENGHT];
float32_t Q_A_2peak_array_icp[QUEUE_LENGHT];
float32_t Q_V_2peak_array_icp[QUEUE_LENGHT];
float32_t Q_D_2peak_array_icp[QUEUE_LENGHT];

float32_t Q_peak_array_4_20[QUEUE_LENGHT];
float32_t Q_2peak_array_4_20[QUEUE_LENGHT];

xQueueHandle acceleration_peak_queue_icp;
xQueueHandle velocity_peak_queue_icp;
xQueueHandle displacement_peak_queue_icp;
xQueueHandle acceleration_2peak_queue_icp;
xQueueHandle velocity_2peak_queue_icp;
xQueueHandle displacement_2peak_queue_icp;

xQueueHandle queue_peak_4_20;
xQueueHandle queue_2peak_4_20;


arm_biquad_casd_df1_inst_f32 filter_main_high_icp;
float32_t pStates_main_high_icp[16];

arm_biquad_casd_df1_inst_f32 filter_main_low_icp;
float32_t pStates_main_low_icp[16];

arm_biquad_casd_df1_inst_f32 filter_instance_highpass_1_icp;
float32_t pStates_highpass_1_icp[16];

arm_biquad_casd_df1_inst_f32 filter_instance_highpass_2_icp;
float32_t pStates_highpass_2_icp[16];


arm_biquad_casd_df1_inst_f32 filter_main_low_4_20;
float32_t pStates_main_low_4_20[8];

arm_biquad_casd_df1_inst_f32 filter_main_high_4_20;
float32_t pStates_main_high_4_20[8];



int16_t settings[REG_COUNT]; //массив настроек 

uint8_t button_state = 0;

uint8_t transmitBuffer[REG_COUNT*2+5];
uint8_t receiveBuffer[16];
uint8_t boot_receiveBuffer[128];
uint8_t master_transmitBuffer[8];
uint8_t master_receiveBuffer[255];
uint8_t HART_receiveBuffer[16];
uint8_t HART_transmitBuffer[8];

uint8_t hart_switch_on = 0;
uint16_t hart_slave_address = 0;
uint16_t hart_slave_numreg = 0;
uint8_t hart_func = 0;
uint16_t hart_regs_qty = 0;
uint16_t hart_timeout_transmit = 0;
uint16_t hart_time_poll = 0;
uint16_t hart_value = 0.0;


//ICP
float32_t icp_voltage = 0.0;
float32_t lo_warning_icp = 0.0;
float32_t hi_warning_icp = 0.0;
float32_t lo_emerg_icp = 0.0;
float32_t hi_emerg_icp = 0.0;
uint8_t break_sensor_icp = 0;
float32_t break_level_icp = 0.0;
//float32_t coef_ampl_icp = 0.0;
//float32_t coef_offset_icp = 0.0;
float32_t range_icp = 0.0;
uint8_t filter_mode_icp = 0;
float32_t rms_acceleration_icp = 0.0;
float32_t rms_velocity_icp = 0.0;
float32_t rms_displacement_icp = 0.0;

float32_t icp_coef_K = 0.0;
float32_t icp_coef_B = 0.0;

//Амплитуда и размах (ПИК, ПИК-ПИК)
float32_t max_acceleration_icp = 0.0;
float32_t min_acceleration_icp = 0.0;
float32_t max_velocity_icp = 0.0;
float32_t min_velocity_icp = 0.0;
float32_t max_displacement_icp = 0.0;
float32_t min_displacement_icp = 0.0;


//4-20
float32_t current_4_20 = 0.0; //ток входного канала 4-20
float32_t out_required_current = 0.0; //ток для выдачи в выходной канал 4-20
float32_t lo_warning_420 = 0.0;
float32_t hi_warning_420 = 0.0;
float32_t lo_emerg_420 = 0.0;
float32_t hi_emerg_420 = 0.0;
uint8_t break_sensor_420 = 0;
float32_t coef_ampl_420 = 0.0;
float32_t coef_offset_420 = 0.0;

float32_t up_user_range_4_20 = 0.0;
float32_t down_user_range_4_20 = 0.0;
float32_t calculated_value_4_20 = 0.0;

//485
uint16_t slave_adr_mb_master = 0;
//float32_t mb_master_BaudRate = 0.0;
uint16_t mb_master_timeout = 0;
uint16_t slave_reg_mb_master = 0;
uint16_t slave_func_mb_master = 0;
float32_t mb_master_recieve_data = 0.0;
uint16_t quantity_reg_mb_master = 0;
uint8_t break_sensor_485 = 0;


struct mb_master
{	
	uint8_t master_on;
	uint8_t master_addr;
	uint8_t master_func;
	uint16_t master_numreg;
	uint8_t master_type;
	float32_t master_coef_A;
	float32_t master_coef_B;
	float32_t master_value;
	float32_t master_warning_set;
	float32_t master_emergency_set;	
	uint16_t request_timeout;
};

struct mb_master master_array[REG_485_QTY];
uint8_t master_transmit_buffer[8];
uint8_t master_receive_buffer[9];
uint8_t master_response_received_id = 0;

static TaskHandle_t xTask18 = NULL;

volatile uint64_t mb_master_timeout_error = 0;
volatile float32_t mb_master_timeout_error_percent = 0;
volatile uint64_t mb_master_crc_error = 0;
volatile float32_t mb_master_crc_error_percent = 0;
volatile uint64_t mb_master_request = 0;
volatile uint64_t mb_master_response = 0;

volatile TickType_t xTimeOutBefore, xTotalTimeOutSuspended;

uint16_t trigger_485_event_attribute_warning = 0;
uint16_t trigger_485_event_attribute_emerg = 0;


//Реле
uint8_t state_emerg_relay = 0;
uint8_t state_warning_relay = 0;
uint16_t mode_relay = 0;
uint8_t source_signal_relay = 0;
uint16_t delay_relay = 0;
uint16_t delay_relay_exit = 0;
uint8_t flag_for_delay_relay_exit = 0;
uint16_t warning_relay_counter = 0;
uint16_t emerg_relay_counter = 0;
uint16_t test_relay = 0;

//Выход 4-20
uint8_t source_signal_out420 = 0;
float32_t variable_for_out_4_20 = 0.0;	
float32_t out_4_20_coef_K = 0.0;	
float32_t out_4_20_coef_B = 0.0;	

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
uint16_t button_center = 0;

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
extern FontDef Font_11x18;
extern FontDef Font_16x26;

uint16_t menu_index = 0;
uint16_t menu_index_array[7];
uint16_t menu_index_pointer = 0;
uint16_t menu_vertical = 0;
uint16_t menu_horizontal = 0;
float32_t baud_rate_uart_2 = 0; //slave
float32_t baud_rate_uart_3 = 0; //master
uint8_t bootloader_state = 0;
extern uint32_t boot_timer_counter;	
uint16_t trigger_event_attribute = 0;


uint16_t channel_ICP_ON = 0;
uint16_t channel_4_20_ON = 0;
uint16_t channel_485_ON = 0;

volatile int temp_var_1 = 0;
volatile int temp_var_2 = 0;

extern uint16_t timer_485_counter;

volatile uint8_t temp_str = 0; //Скроллинг (промотка) строки в меню

uint8_t menu_edit_mode = 0; //Режим редактирования
volatile uint8_t digit_rank = 0; //Разряд числа (для редактирования)
volatile float32_t fractpart = 0.0;

uint8_t temp_stat_1 = 0;
float32_t temp_stat_2 = 0;
uint8_t horizont_menu_lenght = 0;
uint8_t vertical_menu_lenght = 0;
double intpart;	
char buffer[64];
uint8_t config_mode = 0; //Режим конфигурации контроллера

volatile uint16_t number_of_items_in_the_menu = 0;
const uint8_t items_menu_icp = 1;
const uint8_t items_menu_4_20 = 2; 
const uint8_t items_menu_485 = 3;
const uint8_t items_menu_relay = 4;
const uint8_t items_menu_common = 5;
const uint8_t items_menu_info = 6;
const uint8_t items_menu_config = 7;

const uint32_t baudrate_array[] = {1200, 2400, 4800, 9600, 14900, 19200, 38400, 56000, 57600, 115200, 128000, 230400, 256000, 460800, 921600};
volatile uint8_t iter = 0;

uint8_t icp_home_screen_option = 0;

uint16_t reset_to_default = 0;

int16_t icp_menu_points_for_showing = 0;
int16_t menu_485_points_for_showing = 0;
uint8_t menu_edit_settings_mode = 0;




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
uint32_t rtc_read_backup_reg(uint32_t BackupRegister);
void rtc_write_backup_reg(uint32_t BackupRegister, uint32_t data);
void string_scroll(char* msg, uint8_t len);
void string_scroll_with_number(char* msg, uint8_t len, uint8_t number);
void edit_mode(float32_t *var);
void edit_mode_int(int16_t *var); 
void edit_mode_int8(uint8_t *var); 
void init_menu(uint8_t where_from);
void save_settings(void);
void edit_mode_from_list(float32_t *var, uint32_t* list);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

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

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	

	
	acceleration_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));	
	velocity_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	displacement_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));	
	queue_4_20 = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));	
	velocity_queue_4_20 = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	displacement_queue_4_20 = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	
	acceleration_peak_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	velocity_peak_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	displacement_peak_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	acceleration_2peak_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	velocity_2peak_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	displacement_2peak_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));	
	queue_peak_4_20 = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	queue_2peak_4_20 = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	
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
	vSemaphoreCreateBinary(Semaphore_HART_Transmit);
	Mutex_Setting = xSemaphoreCreateMutex();
	
		
	
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
	float32_t temp_max_acceleration_4_20 = 0.0;
	float32_t temp_min_acceleration_4_20 = 0.0;	
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
		arm_max_f32( (float32_t*)&float_adc_value_4_20[0], ADC_BUFFER_SIZE, (float32_t*)&temp_max_acceleration_4_20, &index );
				
		//Min
		arm_min_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&temp_min_acceleration_icp, &index );
		arm_min_f32( (float32_t*)&float_adc_value_4_20[0], ADC_BUFFER_SIZE, (float32_t*)&temp_min_acceleration_4_20, &index );
		
		
		xQueueSend(acceleration_queue_icp, (void*)&temp_rms_acceleration_icp, 0);				
		xQueueSend(queue_4_20, (void*)&temp_mean_acceleration_4_20, 0);		
		
		xQueueSend(acceleration_peak_queue_icp, (void*)&temp_max_acceleration_icp, 0);				
		xQueueSend(acceleration_2peak_queue_icp, (void*)&temp_min_acceleration_icp, 0);			

		xQueueSend(queue_peak_4_20, (void*)&temp_max_acceleration_4_20, 0);				
		xQueueSend(queue_2peak_4_20, (void*)&temp_min_acceleration_4_20, 0);
		
		
		

		//Детектор обрыва ICP (0 - нет обрыва, 1 - обрыв)
		if ( constant_voltage > 64000 ) break_sensor_icp = 1;
		else break_sensor_icp = 0;

		//Детектор обрыва 4-20 (0 - нет обрыва, 1 - обрыв)
		if ( (temp_mean_acceleration_4_20 * coef_ampl_420 + coef_offset_420) > break_level_4_20 ) break_sensor_420 = 0;
		else break_sensor_420 = 1;
		
		
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

		xQueueSend(velocity_peak_queue_icp, (void*)&temp_max_velocity_icp, 0);	
		xQueueSend(velocity_2peak_queue_icp, (void*)&temp_min_velocity_icp, 0);	
		
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
		
		xQueueSend(displacement_peak_queue_icp, (void*)&temp_max_displacement_icp, 0);
		xQueueSend(displacement_2peak_queue_icp, (void*)&temp_min_displacement_icp, 0);		
		
		xSemaphoreGive( Q_Semaphore_Displacement );
		
  }
  /* USER CODE END Displacement_Task */
}

/* Q_Average_A function */
void Q_Average_A(void const * argument)
{
  /* USER CODE BEGIN Q_Average_A */
	uint32_t index;
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
					
					icp_voltage  = rms_acceleration_icp * icp_coef_K + icp_coef_B;
					
					//rms_acceleration_icp = (float32_t) COEF_TRANSFORM_icp_acceleration * icp_voltage;
					//rms_acceleration_icp = (float32_t) (icp_range_volt / icp_range_a) * icp_voltage;
					rms_acceleration_icp = icp_voltage;
					
					
//					//Вычисление разницы времени между проходами
//					xTotalTimeSuspended = xTaskGetTickCount() - xTimeBefore;
//					xTimeBefore = xTaskGetTickCount();	
					
					
					max_acceleration_icp = 0.0;
					min_acceleration_icp = 0.0;
					for (uint16_t i=0; i<QUEUE_LENGHT; i++)
					{
							xQueueReceive(acceleration_peak_queue_icp, (void *) &Q_A_peak_array_icp[i], 0);										
							xQueueReceive(acceleration_2peak_queue_icp, (void *) &Q_A_2peak_array_icp[i], 0);										
					}
					arm_max_f32( (float32_t*)&Q_A_peak_array_icp[0], QUEUE_LENGHT, (float32_t*)&max_acceleration_icp, &index );
					arm_min_f32( (float32_t*)&Q_A_2peak_array_icp[0], QUEUE_LENGHT, (float32_t*)&min_acceleration_icp, &index );
					max_acceleration_icp *= (float32_t) icp_coef_K + icp_coef_B;
					min_acceleration_icp *= (float32_t) icp_coef_K + icp_coef_B;
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
						
					//Усредненное значение тока
					mean_4_20 = (float32_t) (mean_4_20 * coef_ampl_420 + coef_offset_420);

					//Пересчет тока в пользовательский диапазон
					//calculated_value_4_20 =  (mean_4_20 - 4.0) * (16.0 / (up_user_range_4_20 - down_user_range_4_20));
					calculated_value_4_20 =  down_user_range_4_20 + (up_user_range_4_20 - down_user_range_4_20) * ((mean_4_20 - 4.0) / (20.0 - 4.0));
					
					
					max_4_20 = 0.0;
					min_4_20 = 0.0;					
					for (uint16_t i=0; i<QUEUE_LENGHT; i++)
					{
							xQueueReceive(queue_peak_4_20, (void *) &Q_peak_array_4_20[i], 0);										
							xQueueReceive(queue_2peak_4_20, (void *) &Q_2peak_array_4_20[i], 0);										
					}
					arm_max_f32( (float32_t*)&Q_peak_array_4_20[0], QUEUE_LENGHT, (float32_t*)&max_4_20, &index );
					arm_min_f32( (float32_t*)&Q_2peak_array_4_20[0], QUEUE_LENGHT, (float32_t*)&min_4_20, &index );
					max_4_20 = (float32_t) max_4_20 * coef_ampl_420 + coef_offset_420;
					min_4_20 = (float32_t) min_4_20 * coef_ampl_420 + coef_offset_420;					
			}

				
  }
  /* USER CODE END Q_Average_A */
}

/* Q_Average_V function */
void Q_Average_V(void const * argument)
{
  /* USER CODE BEGIN Q_Average_V */
	uint32_t index;
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
										
					//rms_velocity_icp = (float32_t) (icp_range_volt / icp_range_a*2) * (rms_velocity_icp * icp_coef_K + icp_coef_B) / 2;		
					rms_velocity_icp = (float32_t) (rms_velocity_icp * icp_coef_K + icp_coef_B);		
						
			}
			


			
			max_velocity_icp = 0.0;
			min_velocity_icp = 0.0;
			for (uint16_t i=0; i<QUEUE_LENGHT; i++)
			{
					xQueueReceive(velocity_peak_queue_icp, (void *) &Q_V_peak_array_icp[i], 0);										
					xQueueReceive(velocity_2peak_queue_icp, (void *) &Q_V_2peak_array_icp[i], 0);										
			}
			arm_max_f32( (float32_t*)&Q_V_peak_array_icp[0], QUEUE_LENGHT, (float32_t*)&max_velocity_icp, &index );
			arm_min_f32( (float32_t*)&Q_V_2peak_array_icp[0], QUEUE_LENGHT, (float32_t*)&min_velocity_icp, &index );
			max_velocity_icp = (float32_t) (max_velocity_icp * icp_coef_K + icp_coef_B) / 2;
			min_velocity_icp = (float32_t) (min_velocity_icp * icp_coef_K + icp_coef_B) / 2;


  }
  /* USER CODE END Q_Average_V */
}

/* Q_Average_D function */
void Q_Average_D(void const * argument)
{
  /* USER CODE BEGIN Q_Average_D */
	uint32_t index;
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

					rms_displacement_icp = (float32_t) (rms_displacement_icp * icp_coef_K + icp_coef_B) / 2;					
			}


			max_displacement_icp = 0.0;
			min_displacement_icp = 0.0;
			for (uint16_t i=0; i<QUEUE_LENGHT; i++)
			{
					xQueueReceive(displacement_peak_queue_icp, (void *) &Q_D_peak_array_icp[i], 0);										
					xQueueReceive(displacement_2peak_queue_icp, (void *) &Q_D_2peak_array_icp[i], 0);										
			}
			arm_max_f32( (float32_t*)&Q_D_peak_array_icp[0], QUEUE_LENGHT, (float32_t*)&max_displacement_icp, &index );
			arm_min_f32( (float32_t*)&Q_D_2peak_array_icp[0], QUEUE_LENGHT, (float32_t*)&min_displacement_icp, &index );
			max_displacement_icp = (float32_t) (max_displacement_icp  * icp_coef_K + icp_coef_B) / 2;
			min_displacement_icp = (float32_t) (min_displacement_icp * icp_coef_K + icp_coef_B) / 2;
			
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
			osDelay(200);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
			osDelay(200);						
		}
		else
		{	

			//Если реле не сработали и нет обрыва(по любому из каналов) и канал включен, то зажигаем зеленый
		
			if ( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 1 || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0 || 
			 (break_sensor_icp == 1 && channel_ICP_ON == 1) || (break_sensor_420 == 1 && channel_4_20_ON == 1) || (break_sensor_485 == 1 && channel_485_ON == 1) )
			{
				//Горит красный
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			}
			else 
			{				
				//Горит зеленый
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);	
			}
			
						
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 1 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 1)
			{								
				//Мигает Синий 
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				
				osDelay(500);
				
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				
				osDelay(500);
				
			}			
			
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0)
			{				
				//Мигает Красный 
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				
				osDelay(200);
				
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				
				osDelay(200);
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
		
		//Источник сигнала "калибровочный регистр"
		if (settings[89] == 0)
		{
			variable_for_out_4_20 = convert_hex_to_float(&settings[0], 62);					
			out_required_current = variable_for_out_4_20;
		}		
		
		//Источник сигнала ICP
		if (settings[89] == 1)
		{
			out_required_current = (rms_velocity_icp / (16.0 / 20.0)) + 4;		
		}
		
		//Источник сигнала 4-20
		if (settings[89] == 2)
		{
			out_required_current = mean_4_20;
		}
		
		//Источник сигнала 485
		if (settings[89] == 3)
		{
			//variable_485 = convert_hex_to_float(&settings[0], 71); 	
			//out_required_current = variable_485 / (range_out_420 / 16.0) + 4;		
			//out_required_current = mb_master_recieve_data / (range_out_420 / 16.0) + 4;		
			
		}		
		
		//a_to_v = (float32_t) out_required_current * (3.3 / 20.00); 
	
		a_to_v = (out_required_current * (3.3 / 20.00)) * out_4_20_coef_K  + out_4_20_coef_B;
		
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

	char msg[30];
	uint16_t temp_buf[2];
	
	// CS# (This pin is the chip select input. (active LOW))
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	
	ssd1306_Init();
	ssd1306_Fill(1);
	check_logo();
	ssd1306_UpdateScreen();
	osDelay(500);
	

	init_menu(1);
	
  /* Infinite loop */
  for(;;)
  {		
		
			if (warming_flag == 1) 
			{				
				ssd1306_Fill(0);				
				
				logo();
				
				ssd1306_UpdateScreen();
			}
			else 
			{							
					//Навигация по горизонтальному меню							
					if (menu_index_pointer == 1) //ICP
					{
						if (menu_edit_settings_mode == 0) horizont_menu_lenght = 9; 
						else horizont_menu_lenght = 6;
					}
					
					if (menu_index_pointer == 2) //4-20
					{						
						if (menu_edit_settings_mode == 0) horizont_menu_lenght = 1; 
						else horizont_menu_lenght = 8;
					}
					
					if (menu_index_pointer == 3) //485
					{
						if (menu_edit_settings_mode == 0) horizont_menu_lenght = REG_485_QTY;
						else horizont_menu_lenght = 2;	
					}
					
					if (menu_index_pointer == 4) //Реле
					{						
						if (menu_edit_settings_mode == 0) horizont_menu_lenght = 3;
						else horizont_menu_lenght = 4;	
					}
					
					if (menu_index_pointer == 5) horizont_menu_lenght = 4; //Настройки
					if (menu_index_pointer == 6) horizont_menu_lenght = 3; //Информация
					if (menu_index_pointer == 7) horizont_menu_lenght = 3; //Конфигурация
					
				
					if (button_left_pressed_in == 1 && menu_horizontal > 0 && menu_edit_mode == 0) 
					{				
						menu_horizontal--;
						button_left_pressed_in = 0;
						button_center_pressed_in_short = 0;				
						digit_rank = 0;						
					}						
					
					if (button_right_pressed_in == 1 && menu_horizontal < horizont_menu_lenght && menu_edit_mode == 0) 					
					{				
						menu_horizontal++;
						button_right_pressed_in = 0;
						button_center_pressed_in_short = 0;
						digit_rank = 0;						
					}	
					
					//Навигация по вертикальному меню
					if (button_up_pressed_in == 1 && menu_index_pointer > 0 && button_center_pressed_in_short == 0 && menu_edit_mode == 0) 										
					{				
						
						if (menu_index_pointer == 3 && menu_horizontal != 0) //меню 485
						{
								if (menu_vertical > 0) menu_vertical--;
								button_up_pressed_in = 0;
								digit_rank = 0;
						}
						else
						{						
								if (menu_index > 0) menu_index--;					
								menu_index_pointer = menu_index_array[menu_index];
								
								button_up_pressed_in = 0;						
								menu_horizontal = 0;					
								digit_rank = 0;						
						}
					}						
						
					if (button_down_pressed_in == 1 && button_center_pressed_in_short == 0 && menu_edit_mode == 0) 					
					{						

						if (menu_index_pointer == 3 && menu_horizontal != 0) //меню 485
						{
									if (menu_vertical < REG_485_QTY && menu_vertical < 10) menu_vertical++;
									button_down_pressed_in = 0;
									digit_rank = 0;
						}						
						else
						{
								if (menu_index < number_of_items_in_the_menu-1) menu_index++;
								menu_index_pointer = menu_index_array[menu_index];		
								
								button_down_pressed_in = 0;																		
								menu_horizontal = 0;						
								digit_rank = 0;						
						}
					}	
					
					//При коротком нажатии включаем/выключаем режим редактирования, но не в гл.меню
					if (button_center_pressed_in_short == 1 && menu_horizontal != 0) 
					{
						menu_edit_mode = !menu_edit_mode;	
						button_center_pressed_in_short = 0;						
					}					
					//При коротком нажатии в гл.меню включаем/выключаем настроечный режим  
					else if (button_center_pressed_in_short == 1 && menu_horizontal == 0) 
					{
						menu_edit_settings_mode = !menu_edit_settings_mode;	
						button_center_pressed_in_short = 0;						
					}
					
					//Переход между разрядами числа в режиме редактирования
					if (button_left_pressed_in == 1 && menu_edit_mode == 1) 
					{				
						if (digit_rank > 0) digit_rank--;
						else digit_rank = 0;
						
						button_left_pressed_in = 0;	
					}						
					
					if (button_right_pressed_in == 1 && menu_edit_mode == 1) 					
					{				
						if (digit_rank < 1) digit_rank++;
						else digit_rank = 1;
						
						button_right_pressed_in = 0;						
					}	
					
					//Сохранение настроек на флеш
					if (button_center_pressed_in_long == 1 && menu_horizontal != 0)
					{
						save_settings();
						button_center_pressed_in_long = 0;
						button_center_pressed_in_short = 0;
					}
					
					
					
//////////ICP menu					
					if (channel_ICP_ON == 1)
					{	
							if (menu_index_pointer == 1 && menu_horizontal == 0)
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);		
								
								ssd1306_WriteString("ICP",font_8x14,1);										
														
								if (break_sensor_icp == 1) //Если обрыв
								{									
										if (temp_stat_1 == 0) 
										{
											ssd1306_SetCursor(0,15);											
											ssd1306_WriteString("ОБРЫВ",font_8x15_RU,1);
											ssd1306_SetCursor(0,30);	
											ssd1306_WriteString("ДАТЧИКА",font_8x15_RU,1);
										}
										else ssd1306_WriteString(" ",font_8x14,1);
								}
								else
								{

									if (menu_edit_settings_mode == 0) //Режим просмотра вибропараметров ">"
									{
										ssd1306_Fill(0);
										ssd1306_SetCursor(0,0);												
										ssd1306_WriteString("ICP",font_8x14,1);										
										ssd1306_SetCursor(28,0);																														
										triangle_right(55,2);								
										ssd1306_SetCursor(0,15);																									
									}
									else if (menu_edit_settings_mode == 1) //Режим настройки канала ">>"
									{
										ssd1306_Fill(0);
										ssd1306_SetCursor(0,0);												
										ssd1306_WriteString("ICP",font_8x14,1);										
										ssd1306_SetCursor(28,0);																														
										triangle_right(55,2);								
										triangle_right(59,2);		
										ssd1306_SetCursor(0,15);
									}										
									

									if (icp_menu_points_for_showing == 1)	
									{
										strncpy(msg,"СКЗ виброускорения", 18);
										string_scroll(msg, 18);								
										ssd1306_SetCursor(0,30);				
										snprintf(buffer, sizeof buffer, "%.03f", rms_acceleration_icp);
										ssd1306_WriteString(buffer,font_8x14,1);											
									}
									
									if (icp_menu_points_for_showing == 2)	
									{
										strncpy(msg,"СКЗ виброскорости", 17);
										string_scroll(msg, 17);									
										ssd1306_SetCursor(0,30);				
										snprintf(buffer, sizeof buffer, "%.03f", rms_velocity_icp);
										ssd1306_WriteString(buffer,font_8x14,1);		
									}										

									if (icp_menu_points_for_showing == 3)	
									{
										strncpy(msg,"СКЗ виброперемещения", 20);
										string_scroll(msg, 20);								
										ssd1306_SetCursor(0,30);				
										snprintf(buffer, sizeof buffer, "%.03f", rms_displacement_icp);
										ssd1306_WriteString(buffer,font_8x14,1);											
									}			

									if (icp_menu_points_for_showing == 4)	
									{
										strncpy(msg,"Амплитуда виброускорения", 20);
										string_scroll(msg, 20);								
										ssd1306_SetCursor(0,30);				
										snprintf(buffer, sizeof buffer, "%.03f", max_acceleration_icp);
										ssd1306_WriteString(buffer,font_8x14,1);											
									}	
									
									if (icp_menu_points_for_showing == 5)	
									{
										strncpy(msg,"Амплитуда виброскорости", 20);
										string_scroll(msg, 20);								
										ssd1306_SetCursor(0,30);				
										snprintf(buffer, sizeof buffer, "%.03f", max_velocity_icp);
										ssd1306_WriteString(buffer,font_8x14,1);											
									}										

									if (icp_menu_points_for_showing == 6)	
									{
										strncpy(msg,"Амплитуда виброперемещения", 20);
										string_scroll(msg, 20);								
										ssd1306_SetCursor(0,30);				
										snprintf(buffer, sizeof buffer, "%.03f", max_displacement_icp);
										ssd1306_WriteString(buffer,font_8x14,1);											
									}		
									
									
									if (icp_menu_points_for_showing == 7)	
									{
										strncpy(msg,"Размах виброускорения", 20);
										string_scroll(msg, 20);								
										ssd1306_SetCursor(0,30);				
										snprintf(buffer, sizeof buffer, "%.03f", max_acceleration_icp - min_acceleration_icp);
										ssd1306_WriteString(buffer,font_8x14,1);											
									}	
									
									if (icp_menu_points_for_showing == 8)	
									{
										strncpy(msg,"Размах виброскорости", 20);
										string_scroll(msg, 20);								
										ssd1306_SetCursor(0,30);				
										snprintf(buffer, sizeof buffer, "%.03f", max_velocity_icp - min_velocity_icp);
										ssd1306_WriteString(buffer,font_8x14,1);											
									}										

									if (icp_menu_points_for_showing == 9)	
									{
										strncpy(msg,"Размах виброперемещения", 20);
										string_scroll(msg, 20);								
										ssd1306_SetCursor(0,30);				
										snprintf(buffer, sizeof buffer, "%.03f", max_displacement_icp - min_displacement_icp);
										ssd1306_WriteString(buffer,font_8x14,1);											
									}
								}								
																
								//ssd1306_UpdateScreen();				
								
								menu_edit_mode = 0 ; //Запрещаем редактирование
							}

							
							if (menu_index_pointer == 1 && menu_horizontal == 1 && menu_edit_settings_mode == 0)
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);										
								ssd1306_SetCursor(28,0);						
								triangle_left(48,2);						
								triangle_right(55,2);								
								ssd1306_SetCursor(0,15);											

								strncpy(msg,"СКЗ виброускорения", 18);
								string_scroll(msg, 18);
								
								ssd1306_SetCursor(0,30);				
								snprintf(buffer, sizeof buffer, "%.03f", rms_acceleration_icp);
								ssd1306_WriteString(buffer,font_8x14,1);							
								
								//ssd1306_UpdateScreen();			

								menu_edit_mode = 0 ; //Запрещаем редактирование								
							}						

							

							if (menu_index_pointer == 1 && menu_horizontal == 2 && menu_edit_settings_mode == 0)
							{								
									
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);										
								ssd1306_SetCursor(28,0);																		
								triangle_left(48,2);						
								triangle_right(55,2);								
								ssd1306_SetCursor(0,15);																									
									
								strncpy(msg,"СКЗ виброскорости", 17);
								string_scroll(msg, 17);
									
								ssd1306_SetCursor(0,30);				
								snprintf(buffer, sizeof buffer, "%.03f", rms_velocity_icp);
								ssd1306_WriteString(buffer,font_8x14,1);		
								
								//ssd1306_UpdateScreen();			

								menu_edit_mode = 0 ; //Запрещаем редактирование		
							}			

					
							if (menu_index_pointer == 1 && menu_horizontal == 3 && menu_edit_settings_mode == 0)
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);										
								ssd1306_SetCursor(28,0);						
								triangle_left(48,2);						
								triangle_right(55,2);								
								ssd1306_SetCursor(0,15);											

								strncpy(msg,"СКЗ виброперемещения", 20);
								string_scroll(msg, 20);
								
								ssd1306_SetCursor(0,30);				
								snprintf(buffer, sizeof buffer, "%.03f", rms_displacement_icp);
								ssd1306_WriteString(buffer,font_8x14,1);							
								
								//ssd1306_UpdateScreen();			

								menu_edit_mode = 0 ; //Запрещаем редактирование																
							}
							
							if (menu_index_pointer == 1 && menu_horizontal == 4 && menu_edit_settings_mode == 0) //Амплитуда ускорения
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(48,2);						
								triangle_right(55,2);																	
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Амплитуда виброускорения", 24);						
								string_scroll(msg, 24);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.03f", max_acceleration_icp);
								ssd1306_WriteString(buffer,font_8x14,1); 		
														
								//ssd1306_UpdateScreen();

								menu_edit_mode = 0 ; //Запрещаем редактирование									
							}							
							
							if (menu_index_pointer == 1 && menu_horizontal == 5 && menu_edit_settings_mode == 0) //Амплитуда скорости
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(48,2);						
								triangle_right(55,2);																	
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Амплитуда виброскорости", 23);						
								string_scroll(msg, 23);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.03f", max_velocity_icp);
								ssd1306_WriteString(buffer,font_8x14,1); 		
														
								//ssd1306_UpdateScreen();

								menu_edit_mode = 0 ; //Запрещаем редактирование									
							}

							if (menu_index_pointer == 1 && menu_horizontal == 6 && menu_edit_settings_mode == 0) //Амплитуда перемещения
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(48,2);						
								triangle_right(55,2);															
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Амплитуда виброперемещения", 26);						
								string_scroll(msg, 26);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.03f", max_displacement_icp);
								ssd1306_WriteString(buffer,font_8x14,1); 		
														
								//ssd1306_UpdateScreen();

								menu_edit_mode = 0 ; //Запрещаем редактирование									
							}

							if (menu_index_pointer == 1 && menu_horizontal == 7 && menu_edit_settings_mode == 0) //Размах ускорения
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(48,2);						
								triangle_right(55,2);																
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Размах виброускорения", 21);						
								string_scroll(msg, 21);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.03f", max_acceleration_icp - min_acceleration_icp);
								ssd1306_WriteString(buffer,font_8x14,1); 		
														
								//ssd1306_UpdateScreen();

								menu_edit_mode = 0 ; //Запрещаем редактирование									
							}						

							
							if (menu_index_pointer == 1 && menu_horizontal == 8 && menu_edit_settings_mode == 0) //Размах скорости
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(48,2);						
								triangle_right(55,2);															
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Размах виброскорости", 20);						
								string_scroll(msg, 20);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.03f", max_velocity_icp - min_velocity_icp);
								ssd1306_WriteString(buffer,font_8x14,1); 		
														
								//ssd1306_UpdateScreen();

								menu_edit_mode = 0 ; //Запрещаем редактирование									
							}				

							
							if (menu_index_pointer == 1 && menu_horizontal == 9 && menu_edit_settings_mode == 0) //Размах перемещения
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(48,2);																
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Размах виброперемещения", 23);						
								string_scroll(msg, 23);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.03f", max_displacement_icp - min_displacement_icp);
								ssd1306_WriteString(buffer,font_8x14,1); 		
														
								//ssd1306_UpdateScreen();	

								menu_edit_mode = 0 ; //Запрещаем редактирование									
							}							




							//Режим настройки, ICP
							
							if (menu_index_pointer == 1 && menu_horizontal == 1 && menu_edit_settings_mode == 1) //Номер параметра для показа на гл. экране
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);												
								triangle_left(48,2);
								triangle_right(55,2);							
								triangle_right(59,2);							
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Параметр на главном меню", 24);						
								string_scroll(msg, 24);
								
								ssd1306_SetCursor(0,32);			
								
								if (menu_edit_mode == 1) //Режим редактирования
								{											
											edit_mode_int(&icp_menu_points_for_showing);										
								}
								else //Нормальный режим
								{
									snprintf(buffer, sizeof buffer, "%d", icp_menu_points_for_showing);
									ssd1306_WriteString(buffer,font_8x14,1); 
								}												
								
								//ssd1306_UpdateScreen();				
							}
							
							
							if (menu_index_pointer == 1 && menu_horizontal == 2 && menu_edit_settings_mode == 1) //Предупр. уставка
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);												
								triangle_left(48,2);
								triangle_right(55,2);							
								triangle_right(59,2);							
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Предупредительная уставка", 25);						
								string_scroll(msg, 25);
								
								ssd1306_SetCursor(0,32);			
								
								if (menu_edit_mode == 1) //Режим редактирования
								{
											
											edit_mode(&hi_warning_icp);
										
								}
								else //Нормальный режим
								{
									snprintf(buffer, sizeof buffer, "%.01f", hi_warning_icp);
									ssd1306_WriteString(buffer,font_8x14,1); 
								}												
								
								//ssd1306_UpdateScreen();				
							}					
							
							
							
							if (menu_index_pointer == 1 && menu_horizontal == 3 && menu_edit_settings_mode == 1) //Авар. уставка
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(48,2);						
								triangle_right(55,2);						
								triangle_right(59,2);								
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Аварийная уставка", 17);						
								string_scroll(msg, 17);						
								ssd1306_SetCursor(0,32);

								if (menu_edit_mode == 1) //Режим редактирования
								{
									edit_mode(&hi_emerg_icp);
								}
								else //Нормальный режим
								{
									snprintf(buffer, sizeof buffer, "%.01f", hi_emerg_icp);
									ssd1306_WriteString(buffer,font_8x14,1); 
								}					
														
								//ssd1306_UpdateScreen();				
							}	
							
							
							
							if (menu_index_pointer == 1 && menu_horizontal == 4 && menu_edit_settings_mode == 1) //Режим цифрового фильтра
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(48,2);						
								triangle_right(55,2);				
								triangle_right(59,2);								
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Режим фильтра", 13);						
								string_scroll(msg, 13);
								
								ssd1306_SetCursor(0,32);										
								
								if (menu_edit_mode == 1) //Режим редактирования
								{
									edit_mode_int((int16_t*)&filter_mode_icp);
								}
								else //Нормальный режим
								{
									snprintf(buffer, sizeof buffer, "%d", filter_mode_icp);
									ssd1306_WriteString(buffer,font_8x14,1); 
								}					
														
								//ssd1306_UpdateScreen();				
							}							

							if (menu_index_pointer == 1 && menu_horizontal == 5 && menu_edit_settings_mode == 1) //Коэф. усиления
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(48,2);						
								triangle_right(55,2);							
								triangle_right(59,2);	
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Коэффициент усиления", 20);						
								string_scroll(msg, 20);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.05f", icp_coef_K);										
								ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
														
								//ssd1306_UpdateScreen();				
								
								menu_edit_mode = 0 ; //Запрещаем редактирование								
							}	

							
							if (menu_index_pointer == 1 && menu_horizontal == 6 && menu_edit_settings_mode == 1) //Коэф. смещения
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(48,2);																					
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Коэффициент смещения", 20);						
								string_scroll(msg, 20);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.05f", icp_coef_B);										
								ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
														
								//ssd1306_UpdateScreen();

								menu_edit_mode = 0 ; //Запрещаем редактирование																
							}

					}
					
//////////4-20 menu		
					if (channel_4_20_ON == 1)
					{					
					
							if (menu_index_pointer == 2 && menu_horizontal == 0) 
							{
								
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);										
														
								if (break_sensor_420 == 1 & channel_4_20_ON == 1) //Символ обрыва
								{							
										if (temp_stat_1 == 0) 
										{
											ssd1306_SetCursor(0,15);											
											ssd1306_WriteString("ОБРЫВ",font_8x15_RU,1);
											ssd1306_SetCursor(0,30);	
											ssd1306_WriteString("ДАТЧИКА",font_8x15_RU,1);
										}
										else ssd1306_WriteString(" ",font_8x14,1);
								}
								else
								{
								
									if (menu_edit_settings_mode == 0)	
									{
										triangle_right(55,2);										
									}
									else
									{
										triangle_right(55,2);										
										triangle_right(59,2);
									}
									
									ssd1306_SetCursor(0,15);																									
									
																		
									strncpy(msg,"Расчетное значение", 18);						
									string_scroll(msg, 18);
									
									ssd1306_SetCursor(0,30);				
									
									snprintf(buffer, sizeof buffer, "%.03f", calculated_value_4_20);
									ssd1306_WriteString(buffer,font_8x14,1);							
									
								}
								//ssd1306_UpdateScreen();				
							}			

							if (menu_index_pointer == 2 && menu_horizontal == 1 && menu_edit_settings_mode == 0)							
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(48,2);															
								ssd1306_SetCursor(0,15);	
								ssd1306_WriteString("Ток",font_8x15_RU,1);		
								ssd1306_SetCursor(0,30);
								snprintf(buffer, sizeof buffer, "%.03f", mean_4_20);
								ssd1306_WriteString(buffer,font_8x14,1);
								//ssd1306_UpdateScreen();				
								menu_edit_mode = 0 ; //Запрещаем редактирование									
							}

						
							
							//Режим настройки канала 4-20
							if (menu_index_pointer == 2 && menu_horizontal == 1 && menu_edit_settings_mode == 1) //Нижний диапазон
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(48,2);						
								triangle_right(55,2);							
								triangle_right(59,2);									
								ssd1306_SetCursor(0,15);	

								strncpy(msg,"Нижний предел диапазона для пересчета", 37);						
								string_scroll(msg, 37);
								
								ssd1306_SetCursor(0,32);				
								
								
								if (menu_edit_mode == 1) //Режим редактирования
								{								
									edit_mode(&down_user_range_4_20);
								}
								else 
								{
									snprintf(buffer, sizeof buffer, "%.01f", down_user_range_4_20);
									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
								}
														
								//ssd1306_UpdateScreen();				
							}	
							
							if (menu_index_pointer == 2 && menu_horizontal == 2 && menu_edit_settings_mode == 1) //Верхний диапазон
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(48,2);						
								triangle_right(55,2);							
								triangle_right(59,2);									
								ssd1306_SetCursor(0,15);	

								strncpy(msg,"Верхний предел диапазона для пересчета", 38);						
								string_scroll(msg, 38);
								
								ssd1306_SetCursor(0,32);				
								
								
								if (menu_edit_mode == 1) //Режим редактирования
								{								
									edit_mode(&down_user_range_4_20);
								}
								else 
								{
									snprintf(buffer, sizeof buffer, "%.01f", up_user_range_4_20);
									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
								}
														
								//ssd1306_UpdateScreen();				
							}	
							
							
							
							if (menu_index_pointer == 2 && menu_horizontal == 3 && menu_edit_settings_mode == 1) //Уставка нижняя предупр.
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(48,2);						
								triangle_right(55,2);					
								triangle_right(59,2);	
								ssd1306_SetCursor(0,15);	

								strncpy(msg,"Уставка нижняя предупредительная", 32);						
								string_scroll(msg, 32);
								
								ssd1306_SetCursor(0,32);				
								
								
								if (menu_edit_mode == 1) //Режим редактирования
								{								
									edit_mode(&lo_warning_420);
								}
								else 
								{
									snprintf(buffer, sizeof buffer, "%.01f", lo_warning_420);
									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
								}
														
								//ssd1306_UpdateScreen();				
							}		
							
							
							if (menu_index_pointer == 2 && menu_horizontal == 4 && menu_edit_settings_mode == 1) //Уставка нижняя авар.
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(48,2);						
								triangle_right(55,2);			
								triangle_right(59,2);									
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Уставка нижняя аварийная", 24);						
								string_scroll(msg, 24);						
								
								ssd1306_SetCursor(0,32);				
												
								
								if (menu_edit_mode == 1) //Режим редактирования
								{
									edit_mode(&lo_emerg_420);
								}
								else 
								{
									snprintf(buffer, sizeof buffer, "%.01f", lo_emerg_420);
									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
								}
														
								//ssd1306_UpdateScreen();				
							}					

							if (menu_index_pointer == 2 && menu_horizontal == 5 && menu_edit_settings_mode == 1) //Уставка врехняя предупр.
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(48,2);						
								triangle_right(55,2);							
								triangle_right(59,2);									
								ssd1306_SetCursor(0,15);	

								strncpy(msg,"Уставка верхняя предупредительная", 33);						
								string_scroll(msg, 33);
								
								ssd1306_SetCursor(0,32);				
														
								if (menu_edit_mode == 1) //Режим редактирования
								{
									edit_mode(&hi_warning_420);
								}
								else 
								{
									snprintf(buffer, sizeof buffer, "%.01f", hi_warning_420);			
									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
								}
														
								//ssd1306_UpdateScreen();				
							}

							if (menu_index_pointer == 2 && menu_horizontal == 6 && menu_edit_settings_mode == 1) //Уставка врехняя авар.
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(48,2);						
								triangle_right(55,2);							
								triangle_right(59,2);									
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Уставка верхняя аварийная", 25);						
								string_scroll(msg, 25);
								
								ssd1306_SetCursor(0,32);				
								
								if (menu_edit_mode == 1) //Режим редактирования
								{
									edit_mode(&hi_emerg_420);
								}
								else 
								{
									snprintf(buffer, sizeof buffer, "%.01f", hi_emerg_420);			
									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
								}
														
								//ssd1306_UpdateScreen();				
							}					
							
							
							if (menu_index_pointer == 2 && menu_horizontal == 7 && menu_edit_settings_mode == 1)							
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(48,2);						
								triangle_right(55,2);							
								triangle_right(59,2);									
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Коэффициент усиления", 20);						
								string_scroll(msg, 20);		
								
								ssd1306_SetCursor(0,30);
								snprintf(buffer, sizeof buffer, "%.05f", coef_ampl_420);
								ssd1306_WriteString(buffer,font_8x14,1);
								//ssd1306_UpdateScreen();								
								
								menu_edit_mode = 0 ; //Запрещаем редактирование
							}

							if (menu_index_pointer == 2 && menu_horizontal == 8 && menu_edit_settings_mode == 1)							
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(48,2);																
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Коэффициент смещения", 20);						
								string_scroll(msg, 20);		
								
								ssd1306_SetCursor(0,30);
								snprintf(buffer, sizeof buffer, "%.05f", coef_offset_420);
								ssd1306_WriteString(buffer,font_8x14,1);
								//ssd1306_UpdateScreen();				

								menu_edit_mode = 0 ; //Запрещаем редактирование								
							}								
						
					}
					
//////////485 menu		
					if (channel_485_ON == 1)
					{

							if (menu_index_pointer == 3 && menu_horizontal == 0) //Значение регистра
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("485",font_8x14,1);										
														
								if (break_sensor_485 == 1) //Символ обрыва
								{							
										if (temp_stat_1 == 0) 
										{
											ssd1306_SetCursor(0,15);											
											ssd1306_WriteString("ОБРЫВ",font_8x15_RU,1);
											ssd1306_SetCursor(0,30);	
											ssd1306_WriteString("ДАТЧИКА",font_8x15_RU,1);
										}
										else ssd1306_WriteString(" ",font_8x14,1);
								}
								else
								{
										
										if (menu_edit_settings_mode == 0)
										{
											triangle_right(55,2);
										}
										else		
										{
											triangle_right(55,2);
											triangle_right(59,2);
										}
										
										ssd1306_SetCursor(0,15);
										
										//if (menu_485_points_for_showing != 0)	
										{
											strncpy(msg,"Значение регистра ", 18);
											string_scroll_with_number(msg, 18, menu_485_points_for_showing);					

											ssd1306_SetCursor(0,30);				
											
											if (master_array[menu_485_points_for_showing].master_type == 1)
											{
												snprintf(buffer, sizeof buffer, "%.02f", master_array[menu_485_points_for_showing].master_value);
											}
											else
											{
												snprintf(buffer, sizeof buffer, "%d", (int16_t) master_array[menu_485_points_for_showing].master_value);
											}
											
											ssd1306_WriteString(buffer,font_8x14,1);											
										}						
								}
								
								//ssd1306_UpdateScreen();				
							}

							if (menu_edit_settings_mode == 0) 
							for (uint8_t i = 0; i< REG_485_QTY; i++)
							{								
								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 0) //Значение регистра
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);																					
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);
									triangle_left(48,2);						
									if (i != REG_485_QTY-1) triangle_right(55,2);				
									ssd1306_SetCursor(0,15);	
									
									strncpy(msg,"Значение регистра ", 18);						
									string_scroll_with_number(msg, 18, i);

									ssd1306_SetCursor(0,30);
									if (master_array[i].master_type == 1)
									{
										snprintf(buffer, sizeof buffer, "%.02f", master_array[i].master_value);
									}
									else
									{
										snprintf(buffer, sizeof buffer, "%d", (int16_t) master_array[i].master_value);
									}
									ssd1306_WriteString(buffer,font_8x14,1);
									
									menu_edit_mode = 0 ; //Запрещаем редактирование												 									
								}									
																
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 1) //Вкл/выкл опрос
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);		
									triangle_up(50,1);						
									triangle_down(50,8);				
									ssd1306_SetCursor(0,15);	
									
									strncpy(msg,"Включить опрос регистра ", 24);						
									string_scroll_with_number(msg, 24, i);
									ssd1306_SetCursor(0,30);
									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode_int(&settings[REG_485_START_ADDR + 16*i + 0]);																			
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%d", master_array[i].master_on);
										ssd1306_WriteString(buffer,font_8x14,1);										
									}	
									
								}									
								
								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 2) //Адрес устройства
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);	
									triangle_up(50,1);						
									triangle_down(50,8);				
									ssd1306_SetCursor(0,15);	
									
									strncpy(msg,"Адрес устройства регистра ", 26);						
									string_scroll_with_number(msg, 26, i);
									ssd1306_SetCursor(0,30);
									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode_int(&settings[REG_485_START_ADDR + 16*i + 1]);																			
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%d", master_array[i].master_addr);
										ssd1306_WriteString(buffer,font_8x14,1);
									}	
								
								}	
								
								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 3) //Номер регистра
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);
									triangle_up(50,1);						
									triangle_down(50,8);				
									ssd1306_SetCursor(0,15);	
									
									strncpy(msg,"Адрес регистра ", 15);						
									string_scroll_with_number(msg, 15, i);

									ssd1306_SetCursor(0,30);
									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode_int(&settings[REG_485_START_ADDR + 16*i + 2]);																			
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%d", master_array[i].master_numreg);
										ssd1306_WriteString(buffer,font_8x14,1);
									}	
						
								}	

								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 4) //Функциональный код
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);
									triangle_up(50,1);						
									triangle_down(50,8);					
									ssd1306_SetCursor(0,15);	
									
									strncpy(msg,"Функциональный код регистра ", 28);						
									string_scroll_with_number(msg, 28, i);

									ssd1306_SetCursor(0,30);
									
									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode_int(&settings[REG_485_START_ADDR + 16*i + 3]);																			
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%d", master_array[i].master_func);
										ssd1306_WriteString(buffer,font_8x14,1);
									}	
							
								}
								
								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 5) //Предупредительная уставка
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);
									triangle_up(50,1);						
									triangle_down(50,8);				
									ssd1306_SetCursor(0,15);	
									
									strncpy(msg,"Предупредительная уставка регистра ", 35);						
									string_scroll_with_number(msg, 35, i);

									ssd1306_SetCursor(0,30);
									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode(&master_array[i].master_warning_set);
										
											convert_float_and_swap(master_array[i].master_warning_set, &temp_buf[0]);	 
											settings[REG_485_START_ADDR + 16*i + 12] = temp_buf[0];
											settings[REG_485_START_ADDR + 16*i + 13] = temp_buf[1];																														
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%.01f", master_array[i].master_warning_set);
										ssd1306_WriteString(buffer,font_8x14,1);
									}	
						
								}			

								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 6) //Аварийная уставка
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);
									triangle_up(50,1);						
									triangle_down(50,8);					
									ssd1306_SetCursor(0,15);	
									
									strncpy(msg,"Аварийная уставка регистра ", 27);						
									string_scroll_with_number(msg, 27, i);

									ssd1306_SetCursor(0,30);
									
									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode(&master_array[i].master_emergency_set);
										
											convert_float_and_swap(master_array[i].master_emergency_set, &temp_buf[0]);	 
											settings[REG_485_START_ADDR + 16*i + 14] = temp_buf[0];
											settings[REG_485_START_ADDR + 16*i + 15] = temp_buf[1];																														
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%.01f", master_array[i].master_emergency_set);
										ssd1306_WriteString(buffer,font_8x14,1);
									}									
								}									

								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 7) //Коэф. А
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);
									triangle_up(50,1);						
									triangle_down(50,8);				
									ssd1306_SetCursor(0,15);	
									
									strncpy(msg,"Коэффициент А регистра ", 23);						
									string_scroll_with_number(msg, 23, i);

									ssd1306_SetCursor(0,30);
									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode(&master_array[i].master_coef_A);
										
											convert_float_and_swap(master_array[i].master_coef_A, &temp_buf[0]);	 
											settings[REG_485_START_ADDR + 16*i + 6] = temp_buf[0];
											settings[REG_485_START_ADDR + 16*i + 7] = temp_buf[1];																														
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%.05f", master_array[i].master_coef_A);
										ssd1306_WriteString(buffer,font_8x14,1);
									}									
								}		

								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 8) //Коэф. B
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);
									triangle_up(50,1);						
									triangle_down(50,8);					
									ssd1306_SetCursor(0,15);	
									
									strncpy(msg,"Коэффициент В регистра ", 23);						
									string_scroll_with_number(msg, 23, i);

									ssd1306_SetCursor(0,30);
									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode(&master_array[i].master_coef_B);
										
											convert_float_and_swap(master_array[i].master_coef_B, &temp_buf[0]);	 
											settings[REG_485_START_ADDR + 16*i + 8] = temp_buf[0];
											settings[REG_485_START_ADDR + 16*i + 9] = temp_buf[1];																														
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%.05f", master_array[i].master_coef_B);
										ssd1306_WriteString(buffer,font_8x14,1);
									}																
								}	

								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 9) //Тип данных
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);
									triangle_up(50,1);						
									triangle_down(50,8);					
									ssd1306_SetCursor(0,15);	
									
									strncpy(msg,"Тип данных регистра ", 20);						
									string_scroll_with_number(msg, 20, i);

									ssd1306_SetCursor(0,30);
									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode_int(&settings[REG_485_START_ADDR + 16*i + 4]);										
									}
									else //Нормальный режим
									{
											snprintf(buffer, sizeof buffer, "%d", master_array[i].master_type);
											ssd1306_WriteString(buffer,font_8x14,1);
									}																						
								}				

								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 10) //Таймаут
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);
									triangle_up(50,1);																	
									ssd1306_SetCursor(0,15);	
									
									strncpy(msg,"Таймаут регистра ", 17);						
									string_scroll_with_number(msg, 17, i);

									ssd1306_SetCursor(0,30);
									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode_int(&settings[REG_485_START_ADDR + 16*i + 5]);										
									}
									else //Нормальный режим
									{
											snprintf(buffer, sizeof buffer, "%d", master_array[i].request_timeout);
											ssd1306_WriteString(buffer,font_8x14,1);
									}																						
								}									
								
							}							
							
								
							if (menu_index_pointer == 3 && menu_horizontal == 1 && menu_edit_settings_mode == 1) //Номер параметра для показа на гл. экране
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("485",font_8x14,1);												
								triangle_left(48,2);
								triangle_right(55,2);							
								triangle_right(59,2);							
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Параметр на главном меню", 24);						
								string_scroll(msg, 24);
								
								ssd1306_SetCursor(0,30);			
								
								if (menu_edit_mode == 1) //Режим редактирования
								{											
											edit_mode_int(&menu_485_points_for_showing);										
								}
								else //Нормальный режим
								{
									snprintf(buffer, sizeof buffer, "%d", menu_485_points_for_showing);
									ssd1306_WriteString(buffer,font_8x14,1); 
								}										
												
							}
							
							if (menu_index_pointer == 3 && menu_horizontal == 2 && menu_edit_settings_mode == 1) //Скорость обмена
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("485",font_8x14,1);												
								triangle_left(48,2);	
								triangle_left(53,2);
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Скорость", 8);						
								string_scroll(msg, 8);
								
								ssd1306_SetCursor(0,30);			
								
								if (menu_edit_mode == 1) //Режим редактирования
								{
									edit_mode_from_list(&baud_rate_uart_3, (uint32_t*)&baudrate_array);
								}
								else 
								{
									snprintf(buffer, sizeof buffer, "%.00f", baud_rate_uart_3);			
									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
								}								
												
							}								
							
							//ssd1306_UpdateScreen();
					}
					
//////////Реле

					if (menu_index_pointer == 4 && menu_horizontal == 0) //Состояние реле
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Реле",font_8x15_RU,1);																
						
						if (menu_edit_settings_mode == 0) 
						{
							triangle_right(55,2);						
						}
						else
						{
							triangle_right(55,2);
							triangle_right(59,2);
						}
						
						ssd1306_SetCursor(0,15);																									
						ssd1306_WriteString("Пред",font_8x15_RU,1);		
						ssd1306_WriteString(".",font_8x14,1);		
						snprintf(buffer, sizeof buffer, "%d", state_warning_relay);
						ssd1306_WriteString(buffer,font_8x14,1);							
						ssd1306_SetCursor(0,30);				
						ssd1306_WriteString("Авар",font_8x15_RU,1);		
						ssd1306_WriteString(".",font_8x14,1);		
						snprintf(buffer, sizeof buffer, "%d", state_emerg_relay);
						ssd1306_WriteString(buffer,font_8x14,1);							
						
						//ssd1306_UpdateScreen();				
					}							
					
					
					if (menu_index_pointer == 4 && menu_horizontal == 1 && menu_edit_settings_mode == 0) //Аттрибут события ICP, 4-20
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Реле",font_8x15_RU,1);			
						triangle_left(48,2);						
						triangle_right(55,2);										
						ssd1306_SetCursor(0,15);	
						
						strncpy(msg,"Аттрибут события", 16);						
						string_scroll(msg, 16);
						ssd1306_WriteString(" 1",font_8x14,1);
						
						ssd1306_SetCursor(0,30);														
						snprintf(buffer, sizeof buffer, "0x%X", trigger_event_attribute);			
						ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим

						menu_edit_mode = 0 ; //Запрещаем редактирование						
						
						//ssd1306_UpdateScreen();				
					}						
					
					if (menu_index_pointer == 4 && menu_horizontal == 2 && menu_edit_settings_mode == 0) //Аттрибут события 485 пред. уставка
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Реле",font_8x15_RU,1);			
						triangle_left(48,2);						
						triangle_right(55,2);										
						ssd1306_SetCursor(0,15);	
						
						strncpy(msg,"Аттрибут события", 16);						
						string_scroll(msg, 16);
						ssd1306_WriteString(" 2",font_8x14,1);
						
						ssd1306_SetCursor(0,30);														
						snprintf(buffer, sizeof buffer, "0x%X", trigger_485_event_attribute_warning);			
						ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим

						menu_edit_mode = 0 ; //Запрещаем редактирование
						
						//ssd1306_UpdateScreen();				
					}						
					
					if (menu_index_pointer == 4 && menu_horizontal == 3 && menu_edit_settings_mode == 0) //Аттрибут события 485 авар. уставка
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Реле",font_8x15_RU,1);			
						triangle_left(48,2);																				
						ssd1306_SetCursor(0,15);	
						
						strncpy(msg,"Аттрибут события", 16);						
						string_scroll(msg, 16);
						ssd1306_WriteString(" 3",font_8x14,1);
						
						ssd1306_SetCursor(0,30);														
						snprintf(buffer, sizeof buffer, "0x%X", trigger_485_event_attribute_emerg);			
						ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим

						menu_edit_mode = 0 ; //Запрещаем редактирование
						
						//ssd1306_UpdateScreen();				
					}							
					
					
					if (menu_index_pointer == 4 && menu_horizontal == 1 && menu_edit_settings_mode == 1) //Режим работы реле
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Реле",font_8x15_RU,1);			
						triangle_left(48,2);						
						triangle_right(55,2);				
						triangle_right(59,2);
						ssd1306_SetCursor(0,15);	
						ssd1306_WriteString("Режим",font_8x15_RU,1);		
						ssd1306_SetCursor(0,32);				
										
						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&mode_relay);
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", mode_relay);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
						}
												
						//ssd1306_UpdateScreen();				
					}						


					if (menu_index_pointer == 4 && menu_horizontal == 2 && menu_edit_settings_mode == 1) //Задержка на срабатывание реле
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Реле",font_8x15_RU,1);			
						triangle_left(48,2);						
						triangle_right(55,2);				
						triangle_right(59,2);
						ssd1306_SetCursor(0,15);	
						
						strncpy(msg,"Задержка на срабатывание", 24);						
						string_scroll(msg, 24);
						
						ssd1306_SetCursor(0,32);				
									
						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&delay_relay);
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", delay_relay);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
						}
												
						//ssd1306_UpdateScreen();				
					}						
					
					
					if (menu_index_pointer == 4 && menu_horizontal == 3 && menu_edit_settings_mode == 1) //Задержка на выход из срабатывания реле
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Реле",font_8x15_RU,1);			
						triangle_left(48,2);		
						triangle_right(55,2);							
						triangle_right(59,2);						
						ssd1306_SetCursor(0,15);	

						strncpy(msg,"Задержка на выход из срабатывания", 33);						
						string_scroll(msg, 33);
						
						ssd1306_SetCursor(0,32);										
						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&delay_relay_exit);
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", delay_relay_exit);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
						}
			
												
						//ssd1306_UpdateScreen();				
					}	
					

					if (menu_index_pointer == 4 && menu_horizontal == 4 && menu_edit_settings_mode == 1) //Тест работы реле
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Реле",font_8x15_RU,1);			
						triangle_left(48,2);												
						ssd1306_SetCursor(0,15);	

						strncpy(msg,"Тест реле", 9);						
						string_scroll(msg, 9);
						
						ssd1306_SetCursor(0,32);										
						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&test_relay);
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", test_relay);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
						}	
												
						//ssd1306_UpdateScreen();				
					}						
					
					
					
					
//////////Общие настройки	
					
					if (menu_index_pointer == 5 && menu_horizontal == 0) 
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						//ssd1306_WriteString("Настройки",font_8x15_RU,1);																
						//ssd1306_WriteString(".",font_8x14,1);														
						triangle_right(55,2);						
						ssd1306_SetCursor(0,15);																									

						strncpy(msg,"Настройки", 9);						
						string_scroll(msg, 9);						

						//horizont_line(0,45);						
						
						//ssd1306_UpdateScreen();				
					}
					
					
					if (menu_index_pointer == 5 && menu_horizontal == 1) //Адрес устройства
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Настр",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
						triangle_left(48,2);						
						triangle_right(55,2);				
						ssd1306_SetCursor(0,15);	
						ssd1306_WriteString("Адрес",font_8x15_RU,1);		
						ssd1306_SetCursor(0,32);				
						
						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&slave_adr);
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", slave_adr);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
						}
												
						//ssd1306_UpdateScreen();				
					}	
												
					if (menu_index_pointer == 5 && menu_horizontal == 2) //Скорость обмена
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Настр",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
						triangle_left(48,2);						
						triangle_right(55,2);				
						ssd1306_SetCursor(0,15);	

						strncpy(msg,"Скорость", 8);						
						string_scroll(msg, 8);
						
						ssd1306_SetCursor(0,32);				
								
						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_from_list(&baud_rate_uart_2, (uint32_t*)&baudrate_array);
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%.00f", baud_rate_uart_2);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
						}
												
						//ssd1306_UpdateScreen();				
					}						
					
									
					
					if (menu_index_pointer == 5 && menu_horizontal == 3) //Время прогрева
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Настр",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
						triangle_left(48,2);						
						triangle_right(55,2);				
						ssd1306_SetCursor(0,15);	

						strncpy(msg,"Время прогрева", 14);						
						string_scroll(msg, 14);
						
						ssd1306_SetCursor(0,32);				
								
						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&warming_up);
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", warming_up);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
						}
												
						//ssd1306_UpdateScreen();				
					}		
					
					if (menu_index_pointer == 5 && menu_horizontal == 4) //Сброс настроек 
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Настр",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
						triangle_left(48,2);												
						ssd1306_SetCursor(0,15);	

						strncpy(msg,"Сброс настроек", 14);						
						string_scroll(msg, 14);
						
						ssd1306_SetCursor(0,32);				
								
						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&reset_to_default);
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", reset_to_default);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
						}

						//ssd1306_UpdateScreen();						
					}						



////////////Информация

					if (menu_index_pointer == 6 && menu_horizontal == 0) 
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);																		
						triangle_right(55,2);						
						ssd1306_SetCursor(0,15);											
						strncpy(msg,"Информация", 10);						
						string_scroll(msg, 10);							
						//ssd1306_UpdateScreen();				
					}

					
					if (menu_index_pointer == 6 && menu_horizontal == 1) //Напряжение питания контроллера
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Инф",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
						triangle_left(48,2);						
						triangle_right(55,2);				
						ssd1306_SetCursor(0,15);	

						strncpy(msg,"Напряжение питания", 18);						
						string_scroll(msg, 18);
						
						ssd1306_SetCursor(0,32);				
						snprintf(buffer, sizeof buffer, "%.01f", power_supply_voltage);				
						ssd1306_WriteString(buffer,font_8x14,1);
						//ssd1306_UpdateScreen();				
					}	
					
					if (menu_index_pointer == 6 && menu_horizontal == 2) //Версия ПО
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Инф",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
						triangle_left(48,2);						
						triangle_right(55,2);												
						ssd1306_SetCursor(0,15);	

						strncpy(msg,"Версия ПО", 9);						
						string_scroll(msg, 9);
						
						ssd1306_SetCursor(0,32);				
						snprintf(buffer, sizeof buffer, "%.02f", VERSION);				
						ssd1306_WriteString(buffer,font_8x14,1);
						//ssd1306_UpdateScreen();				
					}
					
					
					if (menu_index_pointer == 6 && menu_horizontal == 3) //% ошибок timeout modbus master
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Инф",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
						triangle_left(48,2);																					
						ssd1306_SetCursor(0,15);	

						ssd1306_WriteString("MMTE",font_8x14,1);	
						
						ssd1306_SetCursor(0,32);				
						snprintf(buffer, sizeof buffer, "%.01f", mb_master_timeout_error_percent);				
						ssd1306_WriteString(buffer,font_8x14,1);
						//ssd1306_UpdateScreen();				
					}					

							
					

//////////Конфигурация
					
					if (menu_index_pointer == 7 && menu_horizontal == 0 && config_mode == 1) 
					{
						ssd1306_Fill(0);											
						triangle_right(55,2);						
						ssd1306_SetCursor(0,15);																									

						strncpy(msg,"КОНФИГУРАЦИЯ", 12);						
						string_scroll(msg, 12);						

						//ssd1306_UpdateScreen();				
					}
					
					if (menu_index_pointer == 7 && menu_horizontal == 1) //Включаем канал ICP
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Конф",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
						triangle_left(48,2);						
						triangle_right(55,2);				
						ssd1306_SetCursor(0,15);	
						ssd1306_WriteString("ICP",font_8x14,1);		
						ssd1306_SetCursor(0,32);				
						
						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&channel_ICP_ON);
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", channel_ICP_ON);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
						}
												
						//ssd1306_UpdateScreen();				
					}	
					
					
					if (menu_index_pointer == 7 && menu_horizontal == 2) //Включаем канал 4-20
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Конф",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
						triangle_left(48,2);						
						triangle_right(55,2);				
						ssd1306_SetCursor(0,15);	
						ssd1306_WriteString("4-20",font_8x14,1);		
						ssd1306_SetCursor(0,32);				
						
						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&channel_4_20_ON);
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", channel_4_20_ON);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
						}
												
						//ssd1306_UpdateScreen();				
					}	
					
					
					if (menu_index_pointer == 7 && menu_horizontal == 3) //Включаем канал 485
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Конф",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
						triangle_left(48,2);														
						ssd1306_SetCursor(0,15);	
						ssd1306_WriteString("485",font_8x14,1);		
						ssd1306_SetCursor(0,32);				
						
						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&channel_485_ON);
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", channel_485_ON);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
						}
												
						//ssd1306_UpdateScreen();				
					}	
					
					
				//Рисуем на экранчике
				ssd1306_UpdateScreen();	
					
				//Инверсия переменной (для мигания меню в режиме редакции)	
				temp_stat_1 = !temp_stat_1;

			
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
					
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0)
		{
			button_left ++;
		}		
		else
		{
			if ( button_left > 7 ) 
			{
				button_left_pressed_in = 1;				
				button_right_pressed_in = 0;
				button_up_pressed_in = 0;
				button_down_pressed_in = 0;
//				button_center_pressed_in_short = 0;
//				button_center_pressed_in_long = 0;
				button_center = 0;
				
				button_left = 0;
				
				temp_str = 0;
			}		
		}
		
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == 0)
		{
			button_right ++;		
		}		
		else
		{
			if ( button_right > 7 ) 
			{
				button_left_pressed_in = 0;				
				button_right_pressed_in = 1;
				button_up_pressed_in = 0;
				button_down_pressed_in = 0;
//				button_center_pressed_in_short = 0;
//				button_center_pressed_in_long = 0;
				button_center = 0;
				
				button_right = 0;
				
				temp_str = 0;
			}	
		}
		
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == 0)
		{
			button_up ++;
		}		
		else
		{
			if ( button_up > 7 ) 
			{
				button_left_pressed_in = 0;				
				button_right_pressed_in = 0;
				button_up_pressed_in = 1;
				button_down_pressed_in = 0;
//				button_center_pressed_in_short = 0;
//				button_center_pressed_in_long = 0;
				button_center = 0;
				
				button_up = 0;
				temp_str = 0;
			}			
		}
		
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0)
		{
			button_down ++;
		}		
		else
		{
			if ( button_down > 7 ) 
			{
				button_left_pressed_in = 0;				
				button_right_pressed_in = 0;
				button_up_pressed_in = 0;
				button_down_pressed_in = 1;
//				button_center_pressed_in_short = 0;
//				button_center_pressed_in_long = 0;
				button_center = 0;
				
				button_down = 0;
				temp_str = 0;
			}			
		}
		
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0)
		{
			button_center ++;	
			
			if ( button_center >= 100 ) 
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
		else
		{
			if ( button_center > 2 && button_center < 100 && button_center_pressed_in_long == 0) 
			{
				
					button_left_pressed_in = 0;				
					button_right_pressed_in = 0;
					button_up_pressed_in = 0;
					button_down_pressed_in = 0;
					button_center_pressed_in_short = 1;
					button_center_pressed_in_long = 0;
									
					button_center = 0;				
			}

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
		
		if (bootloader_state == 0)
		{
			HAL_UART_Receive_DMA(&huart2, receiveBuffer, 16);					
			
			xSemaphoreGive( Semaphore_Modbus_Tx );
		}		
		
		if (bootloader_state == 1)
		{
			boot_timer_counter = 0;			
			
			HAL_UART_Receive_DMA(&huart2, boot_receiveBuffer, 128);									
		}	
		
		
    
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
											for (volatile uint16_t i=adr_of_registers, j=0; i < outer_register; i++, j++)
											{
												transmitBuffer[j*2+3] = settings[i] >> 8; //значение регистра Lo 		
												transmitBuffer[j*2+4] = settings[i] & 0x00FF; //значение регистра Hi		
											}
									
											crc = crc16(transmitBuffer, count_registers*2+3);				
									
											transmitBuffer[count_registers*2+3] = crc;
											transmitBuffer[count_registers*2+3+1] = crc >> 8;		
																				
												
											HAL_UART_Transmit_DMA(&huart2, transmitBuffer, count_registers*2+5);
									}
									else
									{
											for (uint16_t i=0, j=0; i < count_registers; i++, j++)
											{
												transmitBuffer[j*2+3] = settings[adr_of_registers + i] >> 8; //значение регистра Lo 		
												transmitBuffer[j*2+4] = settings[adr_of_registers + i] & 0x00FF; //значение регистра Hi		
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
				
				//Команда для перепрошивки
				if (receiveBuffer[1] == 0x62 && receiveBuffer[2] == 0x6F && receiveBuffer[3] == 0x6F && receiveBuffer[4] == 0x74 && boot_timer_counter == 0)
				{					
					
					transmitBuffer[0] = 0x72;
					transmitBuffer[1] = 0x65;
					transmitBuffer[2] = 0x61;
					transmitBuffer[3] = 0x64;
					transmitBuffer[4] = 0x79;
										
					HAL_UART_Transmit_DMA(&huart2, transmitBuffer, 5);
					
					bootloader_state = 1;		
					
					rtc_write_backup_reg(1, bootloader_state);
					
					NVIC_SystemReset();
					
					//receiveBuffer[1] = 0x00; boot_receiveBuffer[1] = 0x00;
					
				}
				else bootloader_state = 0;
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
	volatile uint16_t calculated_crc = 0;
	volatile uint16_t actual_crc = 0;
	volatile float32_t temp;
	uint16_t rawValue = 0;
	
  /* Infinite loop */
  for(;;)
  {
		
		xSemaphoreTake( Semaphore_Master_Modbus_Rx, portMAX_DELAY );					
				
		__HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_IDLEF); 				
		__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
		
		HAL_UART_DMAStop(&huart3); 

		HAL_UART_Receive_DMA(&huart3, master_receive_buffer, 9); 
		
		if (master_receive_buffer[0] == master_array[master_response_received_id].master_addr)
		{						
				f_number = master_receive_buffer[1]; //номер функции	
				byte_number = master_receive_buffer[2];//кол-во байт
						
			
				//считаем crc
				calculated_crc = crc16(master_receive_buffer, 3 + byte_number);										
				actual_crc = master_receive_buffer[3 + byte_number];
				actual_crc += master_receive_buffer[3 + byte_number + 1] << 8;
				
				if (calculated_crc == actual_crc)
				{
						if (master_receive_buffer[1] == 0x03 || master_receive_buffer[1] == 0x04) //Holding Register (FC=03)
						{															
								if ( master_array[master_response_received_id].master_type == 0 ) //Тип данных, Dec
								{	
									master_array[master_response_received_id].master_value = (master_receive_buffer[3] << 8 ) + master_receive_buffer[4];
								}
								if ( master_array[master_response_received_id].master_type == 1 ) //Тип данных, Float
								{
									temp_data[0] = (master_receive_buffer[3] << 8 ) + master_receive_buffer[4];
									temp_data[1] = (master_receive_buffer[5] << 8 ) + master_receive_buffer[6];
									master_array[master_response_received_id].master_value = convert_hex_to_float(&temp_data[0], 0);
								}								
								if ( master_array[master_response_received_id].master_type == 2 ) //Тип данных, Int
								{									
									rawValue = (master_receive_buffer[3] << 8 ) + master_receive_buffer[4];
																		
									if ( rawValue >> 14 == 0 )
									{
										master_array[master_response_received_id].master_value = rawValue; 																			
									}
									else
									{										
										master_array[master_response_received_id].master_value = rawValue | ~((1 << 15) - 1);
									}
								}
								if ( master_array[master_response_received_id].master_type == 3 ) //Тип данных, Abs. int
								{									
									rawValue = (master_receive_buffer[3] << 8 ) + master_receive_buffer[4];
																		
									if ( rawValue >> 14 == 0 )
									{
										master_array[master_response_received_id].master_value = rawValue; 																			
									}
									else
									{										
										master_array[master_response_received_id].master_value = rawValue | ~((1 << 15) - 1);
										master_array[master_response_received_id].master_value = -master_array[master_response_received_id].master_value;
									}
								}								
						}
						
						//Применяем кооэф. к значению
						master_array[master_response_received_id].master_value = master_array[master_response_received_id].master_value * master_array[master_response_received_id].master_coef_A + master_array[master_response_received_id].master_coef_B;
						
						xTaskNotifyGive( xTask18 ); //Посылаем уведомление, если получен ответ 							
						
						//Устанавливаем признак "обрыва нет"
						break_sensor_485 = 0;
						//Обнуляем таймер обрыва датчика 485
						timer_485_counter = 0;
						
				}
				else 
				{
					mb_master_crc_error++;						
				}
				
				mb_master_crc_error_percent = (float32_t) mb_master_crc_error * 100.0 / mb_master_response;
				
				//Счетчик всех ответов
				mb_master_response++;			
		}
			
    
  }
  /* USER CODE END Master_Modbus_Receive */
}

/* Master_Modbus_Transmit function */
void Master_Modbus_Transmit(void const * argument)
{
  /* USER CODE BEGIN Master_Modbus_Transmit */
	uint16_t crc = 0;
	TimeOut_t xTimeOut;
		
	xTask18 = xTaskGetCurrentTaskHandle();
	
  /* Infinite loop */
  for(;;)
  {
		
		for(uint8_t i=0; i< REG_485_QTY; i++)
		{	
				master_transmit_buffer[0] = master_array[i].master_addr;
				master_transmit_buffer[1] = master_array[i].master_func;
				master_transmit_buffer[2] = (master_array[i].master_numreg - 1) >> 8; 		//Смещение адреса, т.к. регистр номер 1 равно адресу 0.
				master_transmit_buffer[3] = (master_array[i].master_numreg - 1) & 0x00FF;
				master_transmit_buffer[4] = 0;			
				if (master_array[i].master_type == 0) master_transmit_buffer[5] = 1;
				else master_transmit_buffer[5] = 2;				
						
				crc = crc16(master_transmit_buffer, 6);				
						
				master_transmit_buffer[6] = crc;
				master_transmit_buffer[7] = crc >> 8;
				
				master_response_received_id = i;

			
				if ( master_array[i].master_on == 1) //Если регистр выключен, то запрашиваем следующий		
				{					
					
					HAL_UART_Transmit_DMA(&huart3, master_transmit_buffer, 8);
					
					//Счетчик запросов
					mb_master_request++;

					
					//Фиксируем время для расчета таймаута					
					xTimeOutBefore = xTaskGetTickCount();			
					
					//Ждем уведомление о получении ответа, либо ошибка по таймауту
					ulTaskNotifyTake( pdTRUE, master_array[i].request_timeout ); 
					
					//Проверка таймаута
					xTotalTimeOutSuspended = xTaskGetTickCount() - xTimeOutBefore;					
					
					if ( xTotalTimeOutSuspended >= master_array[i].request_timeout ) 
					{
						mb_master_timeout_error++;											
					}
					
					mb_master_timeout_error_percent = (float32_t) mb_master_timeout_error * 100.0 / mb_master_request; 						
					
				}				
		}
		
		//Общий интервал опроса всех регистров
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
	volatile float32_t y = 0;
	
  /* Infinite loop */
  for(;;)
  {
		
		//Смещение на -1 (т.е. 1й регистр == settings[0])
		
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
		settings[29] = icp_menu_points_for_showing; 			
		
		
		convert_float_and_swap(mean_4_20, &temp[0]);		
		settings[36] = temp[0];
		settings[37] = temp[1];
		
		settings[46] = break_sensor_420;
		
		convert_float_and_swap(calculated_value_4_20, &temp[0]);		
		settings[55] = temp[0];
		settings[56] = temp[1];

		convert_float_and_swap(max_4_20, &temp[0]);		
		settings[58] = temp[0];
		settings[59] = temp[1];
		convert_float_and_swap(max_4_20 - min_4_20, &temp[0]);		
		settings[60] = temp[0];
		settings[61] = temp[1];

		settings[64] = menu_485_points_for_showing;

		settings[70] = trigger_485_event_attribute_warning;
		settings[71] = trigger_485_event_attribute_emerg;
		settings[73] = break_sensor_485; 
		convert_float_and_swap(mb_master_crc_error_percent, &temp[0]);	
		settings[74] = temp[0];
		settings[75] = temp[1];
		convert_float_and_swap(mb_master_timeout_error_percent, &temp[0]);	
		settings[76] = temp[0];
		settings[77] = temp[1];	
		
		convert_float_and_swap((mb_master_request - mb_master_response), &temp[0]);	
		settings[78] = temp[0];
		settings[79] = temp[1];	

		settings[80] = warning_relay_counter; 
		settings[81] = emerg_relay_counter; 
		settings[82] = state_warning_relay;
		settings[83] = state_emerg_relay;
		
		settings[87] = trigger_event_attribute;

		convert_float_and_swap(power_supply_voltage, &temp[0]);		
		settings[98] = temp[0];
		settings[99] = temp[1];

		convert_float_and_swap(cpu_float, &temp[0]);		
		settings[103] = temp[0];
		settings[104] = temp[1];
		
		convert_float_and_swap(VERSION, &temp[0]);
		settings[105] = temp[0];
		settings[106] = temp[1];

		settings[120] = hart_value;
		
		convert_float_and_swap(max_acceleration_icp, &temp[0]);	
		settings[122] = temp[0];
		settings[123] = temp[1];
		convert_float_and_swap(max_velocity_icp, &temp[0]);	
		settings[124] = temp[0];
		settings[125] = temp[1];
		convert_float_and_swap(max_displacement_icp, &temp[0]);	
		settings[126] = temp[0];
		settings[127] = temp[1];				
		convert_float_and_swap(max_acceleration_icp - min_acceleration_icp, &temp[0]);	
		settings[128] = temp[0];
		settings[129] = temp[1];
		convert_float_and_swap(max_velocity_icp - min_velocity_icp, &temp[0]);	
		settings[130] = temp[0];
		settings[131] = temp[1];
		convert_float_and_swap(max_displacement_icp - min_displacement_icp, &temp[0]);	
		settings[132] = temp[0];
		settings[133] = temp[1];

	
		if (menu_edit_mode == 0)
		for (uint8_t i = 0; i < REG_485_QTY; i++)
		{			
				master_array[i].master_on = settings[REG_485_START_ADDR + 16*i + 0];
				master_array[i].master_addr = settings[REG_485_START_ADDR + 16*i + 1];
				master_array[i].master_numreg = settings[REG_485_START_ADDR + 16*i + 2];
				master_array[i].master_func = settings[REG_485_START_ADDR + 16*i + 3];
				master_array[i].master_type = settings[REG_485_START_ADDR + 16*i + 4];
				master_array[i].request_timeout = settings[REG_485_START_ADDR + 16*i + 5];		
				
				master_array[i].master_coef_A = convert_hex_to_float(&settings[REG_485_START_ADDR + 16*i + 4], 2);
				master_array[i].master_coef_B = convert_hex_to_float(&settings[REG_485_START_ADDR + 16*i + 6], 2);
								 
			
				if (master_array[i].master_type == 0) //Тип, dec
				{
					settings[REG_485_START_ADDR + 16*i + 10] = master_array[i].master_value; 
				}
				if (master_array[i].master_type == 1) //Тип, float
				{					
					convert_float_and_swap(master_array[i].master_value, &temp[0]);	 //Отдаем значение
					settings[REG_485_START_ADDR + 16*i + 10] = temp[0];
					settings[REG_485_START_ADDR + 16*i + 11] = temp[1];
				}
				if (master_array[i].master_type == 2 || master_array[i].master_type == 3) //Тип, int
				{
					settings[REG_485_START_ADDR + 16*i + 10] = (int16_t) master_array[i].master_value; 
				}
				
				
				master_array[i].master_warning_set = convert_hex_to_float(&settings[REG_485_START_ADDR + 16*i + 10], 2);	
				master_array[i].master_emergency_set = convert_hex_to_float(&settings[REG_485_START_ADDR + 16*i + 12], 2);	
		}

		
		
		//Применение/запись настроек
		if (settings[107] == -21555) //0xABCD int16
		{		
			
			//xSemaphoreTake( Mutex_Setting, portMAX_DELAY );
			
			settings[107] = 0x0;
			
			taskENTER_CRITICAL(); 						
			st_flash = write_registers_to_flash(settings);						
			read_init_settings();			
			taskEXIT_CRITICAL(); 					
			
			init_menu(0);
			FilterInit();
			
			//xSemaphoreGive( Mutex_Setting );			
			//NVIC_SystemReset();			
		}
		
		//Сброс настроек
		if (settings[108] == -9030 || (menu_edit_mode == 0 && reset_to_default == 1)) //0xDCBA int16
		{
			settings[108] = 0x0;
			
			for(uint16_t i=0; i< REG_COUNT; i++) 
			{
				if ( 	i == 15 || i == 17 || 
							i == 51 || i == 53 ||
							i == 90 || i == 92  )
				{				
					settings[i] = settings[i];			
				}
				else settings[i] = 0;	
			}
					
			settings[100] = 10; 		
			
			settings[109] = 3000; 
			
			convert_float_and_swap(22, &temp[0]);	
			settings[110] = temp[0];
			settings[111] = temp[1];		
			
			convert_float_and_swap(26, &temp[0]);	
			settings[112] = temp[0];
			settings[113] = temp[1];		
			
			convert_float_and_swap(115200, &temp[0]);	
			settings[101] = temp[0];
			settings[102] = temp[1];		
	
			settings[119] = 100;		


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
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); //Замыкаем реле 2 (норм. замкнутый контакт)
	
  /* Infinite loop */
  for(;;)
  {
		
		//Обнуляем только в режиме работы "без памяти"
		if (mode_relay == 0)
		{
			state_warning_relay = 0;
			state_emerg_relay = 0;
		}
		
		
		if (warming_flag == 0)
		{			
				//Источник сигнала ICP
				if (channel_ICP_ON == 1)
				{
						//Предупр. реле
						if (rms_velocity_icp >= hi_warning_icp && rms_velocity_icp < hi_emerg_icp)								
						{							
							state_warning_relay = 1;						
							trigger_event_attribute |= (1<<15);					
							flag_for_delay_relay_exit = 1;							
							xSemaphoreGive( Semaphore_Relay_1 );							
						}						
						else if ( rms_velocity_icp < hi_warning_icp )
						{							
							if (mode_relay == 0) trigger_event_attribute &= ~(1<<15);			
						}
						
						//Авар. реле
						if ( rms_velocity_icp >= hi_emerg_icp ) 
						{								
							state_emerg_relay = 1;				
							trigger_event_attribute |= (1<<14);
							flag_for_delay_relay_exit = 1;														
							xSemaphoreGive( Semaphore_Relay_2 );
						}
						else if ( rms_velocity_icp < hi_emerg_icp )
						{							
							if (mode_relay == 0) trigger_event_attribute &= ~(1<<14);
						}
				}
				
				//Источник сигнала 4-20
				if (channel_4_20_ON == 1)
				{							
						if ( (calculated_value_4_20 >= hi_warning_420 && mean_4_20 < hi_emerg_420) || 
								 (calculated_value_4_20 <= lo_warning_420 && mean_4_20 > lo_emerg_420) ) 
						{							
							state_warning_relay = 1;			
							trigger_event_attribute |= (1<<13);
							flag_for_delay_relay_exit = 1;														
							xSemaphoreGive( Semaphore_Relay_1 );
						}
						else if ( mean_4_20 > lo_warning_420 || mean_4_20 < hi_warning_420)
						{							
							if (mode_relay == 0) trigger_event_attribute &= ~(1<<13);
						}
						
						if ( calculated_value_4_20 <= lo_emerg_420 || calculated_value_4_20 >= hi_emerg_420 ) 
						{							
							state_emerg_relay = 1;
							trigger_event_attribute |= (1<<12);				
							flag_for_delay_relay_exit = 1;
							xSemaphoreGive( Semaphore_Relay_2 );							
						}
						else if ( calculated_value_4_20 > lo_emerg_420 || calculated_value_4_20 < hi_emerg_420 ) 
						{							
							if (mode_relay == 0) trigger_event_attribute &= ~(1<<12);							
						}
				}
				
				//Источник сигнала 485 (Modbus)
				if (channel_485_ON == 1)
				{		

						for (uint8_t i = 0; i< REG_485_QTY; i++)
						{
								if (master_array[i].master_on == 1)
								{			
										//Предупредительная уставка
										if (master_array[i].master_value >= master_array[i].master_warning_set) 
										{
											trigger_485_event_attribute_warning |= (1<<(15-i));								
											state_warning_relay = 1;
											flag_for_delay_relay_exit = 1;							
											xSemaphoreGive( Semaphore_Relay_1 );							
										}	
										else						
										{
											if (mode_relay == 0) trigger_485_event_attribute_warning &= ~(1<<(15-i));														
										}
										
										
										//Аварийная уставка
										if (master_array[i].master_value >= master_array[i].master_emergency_set) 
										{
											trigger_485_event_attribute_emerg |= (1<<(15-i));								
											state_warning_relay = 1;
											state_emerg_relay = 1;
											flag_for_delay_relay_exit = 1;							
											xSemaphoreGive( Semaphore_Relay_2 );							
										}	
										else						
										{
											if (mode_relay == 0) trigger_485_event_attribute_emerg &= ~(1<<(15-i));														
										}
								}
						}					
					
						
				}
				
				if (mode_relay == 0)
				{
						//Сброс предупр. реле 
						if (state_warning_relay == 0)
						{								
							xSemaphoreGive( Semaphore_Relay_1 );							
						}
						
						//Сброс авар. реле 
						if (state_emerg_relay== 0)
						{							
							xSemaphoreGive( Semaphore_Relay_2 );							
						}				
				}
				
		} //Закрываем условие на "прогрев"
		
		
		//Тест реле
		if (test_relay == 1 && menu_edit_mode == 0)
		{
			xSemaphoreGive( Semaphore_Relay_1 );
			state_warning_relay = 1;
			xSemaphoreGive( Semaphore_Relay_2 );
			state_emerg_relay = 1;
			
			test_relay = 0;
		}
		
		//Квитирование
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0 || settings[96] == 1 || (menu_horizontal == 0 && button_center_pressed_in_long == 1)) 
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			state_warning_relay = 0;
			
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
			state_emerg_relay = 0;
			
			trigger_event_attribute = 0;
			trigger_485_event_attribute_warning = 0;
			trigger_485_event_attribute_emerg = 0;
			
			settings[96] = 0;
			
			if (menu_horizontal == 0) 
			{
				button_center_pressed_in_long = 0;
				menu_edit_mode = 0;
			}
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
void Relay_1_Task(void const * argument) //Нормально разомкнутый контакт
{
  /* USER CODE BEGIN Relay_1_Task */
	uint8_t prev_state_relay;
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_Relay_1, portMAX_DELAY );		
		
		
		if (warming_flag == 0 && state_warning_relay == 1)
		{			
			osDelay(delay_relay);
			if (state_warning_relay == 1)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);				
				
			if (prev_state_relay == 0) warning_relay_counter++;
		}
		
		
		if (state_warning_relay == 0 && mode_relay == 0)
		{
			if (flag_for_delay_relay_exit == 1) { osDelay(delay_relay_exit); flag_for_delay_relay_exit = 0; }
			if (state_warning_relay == 0 && mode_relay == 0)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		}
		
		prev_state_relay = state_warning_relay;
  }
  /* USER CODE END Relay_1_Task */
}

/* Relay_2_Task function */
void Relay_2_Task(void const * argument) //Нормально замкнутый контакт
{
  /* USER CODE BEGIN Relay_2_Task */
	uint8_t prev_state_relay;
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_Relay_2, portMAX_DELAY );		
				
		
		if (warming_flag == 0 && state_emerg_relay == 1)
		{			
			osDelay(delay_relay);
			if (state_emerg_relay == 1)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);		

			if (prev_state_relay == 0) emerg_relay_counter++;			 
		}
		
		if (state_emerg_relay == 0 && mode_relay == 0)
		{
			if (flag_for_delay_relay_exit == 1) { osDelay(delay_relay_exit); flag_for_delay_relay_exit = 0; }
			if (state_emerg_relay == 0 && mode_relay == 0)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		}   
		
		
		prev_state_relay = state_emerg_relay;
  }
  /* USER CODE END Relay_2_Task */
}

/* HART_Receive_Task function */
void HART_Receive_Task(void const * argument)
{
  /* USER CODE BEGIN HART_Receive_Task */
	
	uint16_t crc = 0;
	uint16_t count_registers = 0;
	uint16_t adr_of_registers = 0;
	uint16_t recieve_calculated_crc = 0;
	uint16_t recieve_actual_crc = 0;
	uint16_t outer_register = 0;
	uint8_t func_number = 0;
	uint16_t byte_qty = 0;
	
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_HART_Receive, portMAX_DELAY );		
		
		__HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_IDLEF); 				
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		
		HAL_UART_DMAStop(&huart1); 				

		HAL_UART_Receive_DMA(&huart1, HART_receiveBuffer, 16);							
		
		
		if (HART_receiveBuffer[0] == hart_slave_address)
		{		
				
				func_number = HART_receiveBuffer[1]; //номер функции	
				byte_qty = HART_receiveBuffer[2];//кол-во байт
			
			  recieve_calculated_crc = crc16(HART_receiveBuffer, 6);
				recieve_actual_crc = (HART_receiveBuffer[7] << 8) + HART_receiveBuffer[6];
				
				//Проверяем crc
				if (recieve_calculated_crc == recieve_actual_crc) 
				{						
					if (HART_receiveBuffer[1] == 0x03 || HART_receiveBuffer[1] == 0x04) //Holding Register (FC=03)
					{
						hart_value = ( HART_receiveBuffer[3] << 8 ) + HART_receiveBuffer[4];
					}
						
				}
		}

    
  }
  /* USER CODE END HART_Receive_Task */
}

/* HART_Transmit_Task function */
void HART_Transmit_Task(void const * argument)
{
  /* USER CODE BEGIN HART_Transmit_Task */
	uint16_t crc = 0;
	
  /* Infinite loop */
  for(;;)
  {   
		
		if (hart_switch_on == 1)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);		
			osDelay(50);	

			HART_transmitBuffer[0] = hart_slave_address;
			HART_transmitBuffer[1] = hart_func;
			HART_transmitBuffer[2] = hart_slave_numreg >> 8;
			HART_transmitBuffer[3] = hart_slave_numreg & 0x00FF;
			HART_transmitBuffer[4] = hart_regs_qty >> 8;
			HART_transmitBuffer[5] = hart_regs_qty & 0x00FF;		
			
			crc = crc16(HART_transmitBuffer, 6);				
						
			HART_transmitBuffer[6] = crc;
			HART_transmitBuffer[7] = crc >> 8;	
			
			HAL_UART_Transmit_DMA(&huart1, HART_transmitBuffer, 8);

			
			osDelay(hart_timeout_transmit);		
					
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);				
			
		}
		
		osDelay(2000);
		
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
	
		//Баттерворт 8п 1200 Гц 	
		static float32_t coef_main_low_gain[] = {		
			1*0.02037590912608726,  2*0.02037590912608726,  1*0.02037590912608726,  1.8113034229625364,  -0.89280705946688521,         
			1*0.018539845829478551,  2*0.018539845829478551,  1*0.018539845829478551,  1.6480877493283532,  -0.72224713264626716,         
			1*0.017343705326686571,  2*0.017343705326686571,  1*0.017343705326686571,  1.541757603583966,   -0.61113242489071218,         
			1*0.016758555190594621,  2*0.016758555190594621,  1*0.016758555190594621,  1.4897410561066624,  -0.55677527686904082         
		};

		
		arm_biquad_cascade_df1_init_f32(&filter_main_low_icp, 4, (float32_t *) &coef_main_low_gain[0], &pStates_main_low_icp[0]);									
		
		//Butterworth 3 Order, LowPass 100 Hz
		static float32_t coef_main_low_100_gain[] = {
                
			1*0.00014876521137360051,  2*0.00014876521137360051,  1*0.00014876521137360051,  1.9751611962490403,   -0.97575625709453451,        
			1*0.012123675033811735,    1*0.012123675033811735,    0*0.012123675033811735,    0.97575264993237654,  0                          
		};
		
		arm_biquad_cascade_df1_init_f32(&filter_main_low_4_20, 2, (float32_t *) &coef_main_low_100_gain[0], &pStates_main_low_4_20[0]);	
		

		//Баттерворт, 8п, 2Гц 
		static float32_t coef_main_highpass_2Hz_gain[] = {				
			1*0.99990418420245675,  -2*0.99990418420245675,  1*0.99990418420245675,  1.9998082479378831,  -0.99980848887194396,        
			1*0.9997272992408186 ,  -2*0.9997272992408186 ,  1*0.9997272992408186 ,  1.9994544780359176,  -0.99945471892735671,        
			1*0.99959195962591507,  -2*0.99959195962591507,  1*0.99959195962591507,  1.9991837988224159,  -0.99918403968124414,        
			1*0.99951872963503841,  -2*0.99951872963503841,  1*0.99951872963503841,  1.9990373388494853,  -0.9990375796906682					
		};		
		

		//Баттерворт, 8п, 4Гц 
		static float32_t coef_main_highpass_5Hz_gain[] = {                                                 

			1*0.99980826632053266,  -2*0.99980826632053266,  1*0.99980826632053266,  1.9996160508191096,  -0.99961701446302098,        
			1*0.9994546267924348,   -2*0.9994546267924348,   1*0.9994546267924348,   1.998908771933338,   -0.99890973523640125,        
			1*0.99918413177870291,  -2*0.99918413177870291,  1*0.99918413177870291,  1.9983677820362298,  -0.99836874507858209,        
			1*0.99903780198479519,  -2*0.99903780198479519,  1*0.99903780198479519,  1.9980751225189326,  -0.99807608542024806			
		};
  		


		//Баттерворт, 8п, 6Гц 
		static float32_t coef_main_highpass_10Hz_gain[] = {		             						
			1*0.99971224640664647,  -2*0.99971224640664647,  1*0.99971224640664647,  1.9994234088177905,  -0.99942557680879573,        
			1*0.99918198269730008,  -2*0.99918198269730008,  1*0.99918198269730008,  1.9983628819740664,  -0.99836504881513399,        
			1*0.99877651629648445,  -2*0.99877651629648445,  1*0.99877651629648445,  1.9975519496120855,  -0.99755411557385232,        
			1*0.99855721667054131,  -2*0.99855721667054131,  1*0.99855721667054131,  1.9971133505979874,  -0.99711551608417792
		};  
		


		if (FILTER_MODE == 1 || FILTER_MODE == 0) 		
		{
			arm_biquad_cascade_df1_init_f32(&filter_main_high_icp, 4, (float32_t *) &coef_main_highpass_2Hz_gain[0], &pStates_main_high_icp[0]);				
						
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_1_icp, 4, (float32_t *) &coef_main_highpass_2Hz_gain[0], &pStates_highpass_1_icp[0]);							
							
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_2_icp, 4, (float32_t *) &coef_main_highpass_2Hz_gain[0], &pStates_highpass_2_icp[0]);				
			
		}
		else
		if (FILTER_MODE == 2)		
		{
			arm_biquad_cascade_df1_init_f32(&filter_main_high_icp, 4, (float32_t *) &coef_main_highpass_5Hz_gain[0], &pStates_main_high_icp[0]);				
						
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_1_icp, 4, (float32_t *) &coef_main_highpass_5Hz_gain[0], &pStates_highpass_1_icp[0]);							
							
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_2_icp, 4, (float32_t *) &coef_main_highpass_5Hz_gain[0], &pStates_highpass_2_icp[0]);							
		}
		else
		if (FILTER_MODE == 3)		
		{
			arm_biquad_cascade_df1_init_f32(&filter_main_high_icp, 4, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_main_high_icp[0]);				
							
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_1_icp, 4, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_highpass_1_icp[0]);							
							
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_2_icp, 4, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_highpass_2_icp[0]);				
									
		}	
		
		
		
//		arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_3_icp, 2, (float32_t *) &coef_main_highpass_30Hz_gain[0], &pStates_highpass_3_icp[0]);							
//		arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_4_icp, 2, (float32_t *) &coef_main_highpass_300Hz_gain[0], &pStates_highpass_4_icp[0]);							
		
}

uint32_t rtc_read_backup_reg(uint32_t BackupRegister) 
{
    RTC_HandleTypeDef RtcHandle;
    RtcHandle.Instance = RTC;
    return HAL_RTCEx_BKUPRead(&RtcHandle, BackupRegister);
}
 
void rtc_write_backup_reg(uint32_t BackupRegister, uint32_t data) 
{
    RTC_HandleTypeDef RtcHandle;
    RtcHandle.Instance = RTC;
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&RtcHandle, BackupRegister, data);
    HAL_PWR_DisableBkUpAccess();
}

void string_scroll(char* msg, uint8_t len)
{		
	
	for(int i = temp_str; i < len; i++)	
		ssd1306_WriteChar(msg[i],font_8x15_RU,1);		
	
	if ( temp_str > len ) 
	{
		temp_str = 0;		
	}
	else 
	{
		temp_str++;		
	}
	
	osDelay(200);
	
}

void string_scroll_with_number(char* msg, uint8_t len, uint8_t number)
{		
	
	for(int i = temp_str; i < len; i++)
	{	
		ssd1306_WriteChar(msg[i],font_8x15_RU,1);		
	}
	
	snprintf(buffer, sizeof buffer, "%d", number);
	ssd1306_WriteChar(buffer[0],font_8x14,1);		
	
	if ( temp_str > len ) 
	{
		temp_str = 0;		
	}
	else 
	{
		temp_str++;		
	}
	
	osDelay(200);
	
}

void edit_mode(float32_t *var)
{
	//Целая часть
	if (temp_stat_1 == 0 && digit_rank == 0) 
	{									
		snprintf(buffer, sizeof buffer, "%.01f", *var);									
		ssd1306_WriteString(buffer,font_8x14,1);
	}
	else if (temp_stat_1 == 1 && digit_rank == 0) 
	{
		fractpart = modf(*var, &intpart)*10;
		snprintf(buffer, sizeof buffer, "%d", (int)*var);									
		ssd1306_WriteString(buffer,font_8x14, 0);
		
		ssd1306_WriteString(".",font_8x14,1);									
		snprintf(buffer, sizeof buffer, "%d", (int)fractpart);									
		ssd1306_WriteString(buffer,font_8x14,1);									
	}															

	//Дробная часть
	if (temp_stat_1 == 0 && digit_rank == 1) 
	{
		snprintf(buffer, sizeof buffer, "%.01f", *var);									
		ssd1306_WriteString(buffer,font_8x14,1);
	}
	else if (temp_stat_1 == 1 && digit_rank == 1) 
	{
		snprintf(buffer, sizeof buffer, "%d", (int)*var);									
		ssd1306_WriteString(buffer,font_8x14,1);
	}							
	
	//Изменяем значение
	if (button_up_pressed_in == 1 && digit_rank == 0) 
	{ 
			*var+=1.0; 
			button_up_pressed_in = 0; 
	};
	
	if (button_up_pressed_in == 1 && digit_rank == 1) 
	{ 
			*var+=0.1; 
			button_up_pressed_in = 0; 
	};
	
	if (button_down_pressed_in == 1 && digit_rank == 0) 
	{ 
			*var-=1.0;  
			button_down_pressed_in = 0; 
	};
	
	if (button_down_pressed_in == 1 && digit_rank == 1) 
	{ 
			*var-=0.1; 
			button_down_pressed_in = 0; 
	};
}	

void edit_mode_int8(uint8_t *var) 
{

	if (temp_stat_1 == 0 && digit_rank == 0) 
	{									
		snprintf(buffer, sizeof buffer, "%d", *var);									
		ssd1306_WriteString(buffer,font_8x14,1);
	}
	else if (temp_stat_1 == 1 && digit_rank == 0) 
	{		
		snprintf(buffer, sizeof buffer, "", *var);									
		ssd1306_WriteString(buffer,font_8x14, 0);								
	}															
	
	//Изменяем значение
	if (button_up_pressed_in == 1 && digit_rank == 0) 
	{ 
			*var+=1; 
			button_up_pressed_in = 0; 
	};

	if (button_down_pressed_in == 1 && digit_rank == 0) 
	{ 
			*var-=1;  
			button_down_pressed_in = 0; 
	};

}	

void edit_mode_int(int16_t *var) 
{

	if (temp_stat_1 == 0 && digit_rank == 0) 
	{									
		snprintf(buffer, sizeof buffer, "%d", *var);									
		ssd1306_WriteString(buffer,font_8x14,1);
	}
	else if (temp_stat_1 == 1 && digit_rank == 0) 
	{		
		snprintf(buffer, sizeof buffer, "", *var);									
		ssd1306_WriteString(buffer,font_8x14, 0);								
	}															
	
	//Изменяем значение
	if (button_up_pressed_in == 1 && digit_rank == 0) 
	{ 
			*var+=1; 
			button_up_pressed_in = 0; 
	};

	if (button_down_pressed_in == 1 && digit_rank == 0) 
	{ 
			*var-=1;  
			button_down_pressed_in = 0; 
	};

}	

void edit_mode_from_list(float32_t *var, uint32_t* list)
{
	
	//Целая часть
	if (temp_stat_1 == 0) 
	{									
		snprintf(buffer, sizeof buffer, "%d", (int)*var);									
		ssd1306_WriteString(buffer,font_8x14,1);
	}
	else if (temp_stat_1 == 1) 
	{
		fractpart = modf(*var, &intpart)*10;
		snprintf(buffer, sizeof buffer, "%d", (int)*var);									
		ssd1306_WriteString(buffer,font_8x14, 0);								
	}															
		
	//Изменяем значение
	if (button_up_pressed_in == 1) 
	{ 
			if (iter < 15) *var=list[iter++];					
			button_up_pressed_in = 0; 
	};

	
	if (button_down_pressed_in == 1) 
	{ 
			if (iter > 0) *var=list[iter--];  
			button_down_pressed_in = 0; 
	};
	

}	

void init_menu(uint8_t where_from) //Иниц. меню (1 - конфиг, т.е. зажата кнопка при вкл.)
{	
	number_of_items_in_the_menu = 0;
	menu_horizontal = 0;
	config_mode = 0;
	menu_index = 0;
	
	if (channel_ICP_ON == 1) 
	{			
		menu_index_array[number_of_items_in_the_menu] = items_menu_icp;
		number_of_items_in_the_menu++;
	}
	
	if (channel_4_20_ON == 1) 
	{
		menu_index_array[number_of_items_in_the_menu] = items_menu_4_20;
		number_of_items_in_the_menu++;
	}
	
	if (channel_485_ON == 1)
	{
		menu_index_array[number_of_items_in_the_menu] = items_menu_485;
		number_of_items_in_the_menu++;
	}
	
	menu_index_array[number_of_items_in_the_menu] = items_menu_relay;
	number_of_items_in_the_menu ++; //Реле
	
	menu_index_array[number_of_items_in_the_menu] = items_menu_common;
	number_of_items_in_the_menu ++; //Основные настройки
 			
	menu_index_array[number_of_items_in_the_menu] = items_menu_info;
	number_of_items_in_the_menu ++; //Информация
	
	
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0 && where_from == 1) 
	{
		config_mode = 1;	
		menu_index_array[number_of_items_in_the_menu] = items_menu_config;
		number_of_items_in_the_menu++; //Включаем доп. меню для конфигурации								
	}	
	
	menu_index_pointer = menu_index_array[0];	

}

void save_settings(void)
{
			uint16_t temp[2];	
			volatile uint8_t res = 0;
	
			//xSemaphoreTake( Mutex_Setting, portMAX_DELAY );
	
	
//			convert_float_and_swap(hi_warning_icp, &temp[0]);		
//			settings[4] = temp[0];
//			settings[5] = temp[1];
//			convert_float_and_swap(hi_emerg_icp, &temp[0]);		
//			settings[8] = temp[0];
//			settings[9] = temp[1];		
//				settings[19] = filter_mode_icp;	
//			convert_float_and_swap(lo_warning_420, &temp[0]);		
//			settings[38] = temp[0];
//			settings[39] = temp[1];	
//			
//			convert_float_and_swap(lo_warning_420, &temp[0]);		
//			settings[38] = temp[0];
//			settings[39] = temp[1];	
//			convert_float_and_swap(lo_emerg_420, &temp[0]);		
//			settings[42] = temp[0];
//			settings[43] = temp[1];	
//			convert_float_and_swap(hi_warning_420, &temp[0]);		
//			settings[40] = temp[0];
//			settings[41] = temp[1];	
//			convert_float_and_swap(hi_emerg_420, &temp[0]);		
//			settings[44] = temp[0];
//			settings[45] = temp[1];	
//			
//			
//			//settings[64] = slave_adr_mb_master;				
//			convert_float_and_swap(baud_rate_uart_3, &temp[0]);
//			settings[65] = temp[0];
//			settings[66] = temp[1];										
//			settings[68] = slave_reg_mb_master;					
//			//settings[70] = slave_func_mb_master;
//			//settings[71] = quantity_reg_mb_master;
//			
//			settings[84] = mode_relay;
//			settings[86] = delay_relay;
//			settings[88] = delay_relay_exit;
//			
//			settings[100] = slave_adr;
//			convert_float_and_swap(baud_rate_uart_2, &temp[0]);
//			settings[101] = temp[0];
//			settings[102] = temp[1];										
//			settings[109] = warming_up;
//			
			settings[28] = channel_ICP_ON;
			settings[57] = channel_4_20_ON;
			settings[72] = channel_485_ON;
	
	
	
			
			taskENTER_CRITICAL(); 									
			res = write_registers_to_flash(settings);				
			taskEXIT_CRITICAL(); 			
		

			init_menu(0);			
	
			ssd1306_Fill(0);
			ssd1306_SetCursor(0,0);												
			ssd1306_WriteString("Настр",font_8x15_RU,1);																												
			ssd1306_WriteString(".",font_8x14,1);	
			ssd1306_SetCursor(0,15);	
			ssd1306_WriteString("сохр",font_8x15_RU,1);																																		
			ssd1306_WriteString(".",font_8x14,1);	
			ssd1306_UpdateScreen();			
			osDelay(2000);	
	
			//xSemaphoreGive( Mutex_Setting );
			
			//NVIC_SystemReset();		
}




/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
