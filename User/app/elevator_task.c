#include "elevator_task.h"
#include "cmsis_os.h"
#include "can_device.h"
#include "uart_device.h"
#include "elevator_storage.h"

void elevator_task(const void* argu)
{
  while(1) {
		send_elevator_motor_current(rc.ch4*43, rc.ch2*43, 0);
		osDelay(5);
  }
}
