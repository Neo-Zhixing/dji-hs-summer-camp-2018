#include "execute_task.h"
#include "can_device.h"
#include "uart_device.h"
#include "cmsis_os.h"
#include "calibrate.h"
#include "pid.h"
#include "sys.h"
#include "i2c.h"



uint8_t theAddress;

void execute_task(const void* argu)
{
	I2C_Init();
  while(1) {
		uint8_t data[1] = {0x03};
		I2C_Write(0x90, data, 1);
		uint8_t data2[1];
		I2C_Read(0x91, data2, 1);
		theAddress = data2[0];
    osDelay(50);
  }
}
