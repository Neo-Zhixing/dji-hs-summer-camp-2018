#include "elevator_task.h"
#include "cmsis_os.h"
#include "uart_device.h"
#include "can_device.h"
#include "elevator_storage.h"

void elevator_task(const void* argu)
{
	
	elevator_init();
  while(1) {
		elevator_update();
		//elevator_target_coordinates.y += rc.ch2 *0.01;
		//send_elevator_motor_current(10000, 10000, 10000);
		osDelay(5);
		int16_t theCurrent[3] = {C610_MAX_CURRENT*-0.5, C610_MAX_CURRENT*-0.5, C610_MAX_CURRENT*-0.5};
		
		send_mill_motor_current(theCurrent);
  }
}
