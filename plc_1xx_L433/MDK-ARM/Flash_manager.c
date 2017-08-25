#include "main.h"
#include "arm_math.h"
#include "math.h"
#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_flash.h"
#include "Flash_manager.h"

//typedef struct     
//{
//		float32_t* data;
//    uint32_t crc;
//	
//} flash_data_register;

void Flash_manager()
{
	uint32_t status = 0;
	
//	flash_data_register* dr;
//	dr->data = 1;

	FLASH_EraseInitTypeDef EraseInitStruct;
		
	uint32_t PAGEError = 0;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = 100;
	EraseInitStruct.NbPages = 1;

	status = HAL_FLASH_Unlock();	
	status = HAL_FLASHEx_Erase(&EraseInitStruct,&PAGEError);	
	//HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x8032000, *(uint32_t *) &dr ); 

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	status = HAL_FLASH_GetError();

	HAL_FLASH_Lock(); 

//	status = *(__IO uint32_t*)0x8032000;
//	status = *(float*) status;
	
}

