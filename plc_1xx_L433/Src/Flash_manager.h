#include "main.h"
#include "arm_math.h"
#include "math.h"
#include <stdint.h>

#ifndef FLASH_MANAGER_H_
#define FLASH_MANAGER_H_

typedef struct     
{
		uint32_t* data;
    uint32_t crc;
	
} flash_data;

void write_flash(uint32_t page, uint32_t* data, uint32_t size);
uint32_t read_flash(uint32_t addr);
uint16_t crc16(uint8_t *adr_buffer, uint32_t byte_cnt);

#endif /* FLASH_MANAGER_H_ */
