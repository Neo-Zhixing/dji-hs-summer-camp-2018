#include "elevator_task.h"
#include "cmsis_os.h"
#include "uart_device.h"
#include "can_device.h"
#include "elevator_storage.h"

void elevator_task(const void* argu)
{
	
	elevator_init();
	storage_blocker_swing(STORAGE_BLOCKER_SWING_MID);
  while(1) {
		elevator_update();
		elevator_target_coordinates.y -= rc.ch4 *0.04;
		
		if (rc.kb.bit.Q)
			elevator_target_coordinates.y += 5;
		else if (rc.kb.bit.E)
			elevator_target_coordinates.y -= 5;
		claw_horizontal_set(rc.sw1 == RC_DN || rc.mouse.l);
		
			//int16_t theCurrent[3] = {-10000, -10000, -10000};
		//send_mill_motor_current(theCurrent);
		//int16_t theCurrent[3] = {-10000, -10000, -10000};
		//send_mill_motor_current(theCurrent);
		osDelay(5);
  }
}
