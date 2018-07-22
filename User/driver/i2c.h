#ifndef __USER_I2C_H__
#define __USER_I2C_H__

#include "sys.h"

void I2C_Init(void);
uint8_t I2C_Ready(uint8_t address);
void I2C_Read(uint8_t address, uint8_t * data, uint16_t size);
void I2C_Write(uint8_t address, uint8_t * data, uint16_t size);

#endif
