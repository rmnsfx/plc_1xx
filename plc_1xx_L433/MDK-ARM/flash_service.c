#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "math.h"
#include <stdint.h>
#include "flash_service.h"


void write_to_FLASH(void)
{		
		uint32_t receiveBuffer[64];		
		float32_t f = 1.1;
	
	
		
		FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PAGEError = 0;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page = FLASH_BASE_ADDRESS;
    EraseInitStruct.NbPages = 1;

    HAL_FLASH_Unlock();   
		
    HAL_FLASHEx_Erase(&EraseInitStruct,&PAGEError);   
		
		for(uint32_t i = 0; i < REG_COUNT; i++)
		{
			//f = 10 + i; 
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_BASE_ADDRESS + i*4, *(uint32_t *)&f );   
			
		}
    
    HAL_FLASH_Lock();  
}		
		
		
uint32_t FLASH_Read(uint32_t address)
{
    return (*(__IO uint32_t*)address);
}

void read_from_FLASH(void)
{
		float32_t float_receiveBuffer[64];
		uint32_t read;
		float32_t temp;
	
		for(uint32_t i = 0; i < REG_COUNT; i++)
		{
			read = FLASH_Read(FLASH_BASE_ADDRESS + i*4);			
			temp = *(float*)&read;
			float_receiveBuffer[i] = temp;			
		}
}

