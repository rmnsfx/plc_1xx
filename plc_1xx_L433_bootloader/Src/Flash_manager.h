#include "main.h"
#include "arm_math.h"
#include "math.h"
#include <stdint.h>

#ifndef FLASH_MANAGER_H_
#define FLASH_MANAGER_H_


typedef struct     
{		uint32_t* data;
    uint32_t crc;	
} flash_data;

uint8_t read_registers_from_flash(uint16_t* data_out);
uint8_t write_registers_to_flash(uint16_t* data);
uint16_t crc16(uint8_t *adr_buffer, uint32_t byte_cnt);
uint16_t flash_crc16(uint32_t adr, uint32_t byte_cnt);


#endif /* FLASH_MANAGER_H_ */
