#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "math.h"
#include <stdint.h>
#include "flash_service.h"
#include <stdlib.h>
#include "stm32l4xx_hal_flash_ex.h"

void write_to_FLASH(float32_t settings)
{					
			
		HAL_FLASH_Unlock();  
	
		FLASH_EraseInitTypeDef EraseInitStruct;
	
    uint32_t PAGEError = 0;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page = FLASH_BASE_ADDRESS;
    EraseInitStruct.NbPages = 1;

		//FLASH_OBProgramInitTypeDef RdpInitStruct;		
		//RdpInitStruct.RDPLevel = OB_RDP_LEVEL_0;		
		//HAL_FLASHEx_OBProgram(&RdpInitStruct);		
		//HAL_FLASHEx_OBGetConfig(&RdpInitStruct);
		

		
	
		//taskENTER_CRITICAL();		
	
    HAL_FLASHEx_Erase(&EraseInitStruct,&PAGEError);
		
	
		
		for(uint32_t i = 0; i < REG_COUNT; i++)
		{
		
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_BASE_ADDRESS + i*4, *(uint32_t *)&settings );   
			
		}		
		
    HAL_FLASH_Lock();  
		
		//taskEXIT_CRITICAL();
}		
		
		
uint32_t FLASH_Read(uint32_t address)
{
    return (*(__IO uint32_t*)address);
}

float32_t* read_from_FLASH(void)
{
		float32_t* float_receiveBuffer = malloc(sizeof(float32_t*) * REG_COUNT);
		uint32_t read;
		float32_t temp;
	
		for(uint32_t i = 0; i < REG_COUNT; i++)
		{
			read = FLASH_Read(FLASH_BASE_ADDRESS + i*4);			
			temp = *(float*)&read;
			float_receiveBuffer[i] = temp;			
		}
		
		return float_receiveBuffer;
}

