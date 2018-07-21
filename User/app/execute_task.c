#include "execute_task.h"
#include "grayscale_sensor.h"
#include "can_device.h"
#include "uart_device.h"
#include "cmsis_os.h"
#include "calibrate.h"
#include "pid.h"
#include "sys.h"

uint16_t * grayscale_data;
void execute_task(const void* argu)
{
	grayscale_init();
  while(1) {
		grayscale_data = grayscale_read();
		
    osDelay(50);
  }
}
