#include "execute_task.h"
#include "can_device.h"
#include "uart_device.h"
#include "cmsis_os.h"
#include "calibrate.h"
#include "pid.h"
#include "sys.h"
#include "i2c_device.h"



uint8_t theAddress;

void execute_task(const void* argu)
{
	I2C_Init();
  while(1) {
		theAddress = External_ADC_Read(0x03);
    osDelay(50);
  }
}
