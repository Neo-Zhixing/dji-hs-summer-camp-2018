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
				if (rc.sw2 == RC_DN) {
			elevator_target_coordinates.x += rc.ch3 * 0.05f;
			elevator_target_coordinates.y += rc.ch4 * 0.14f;
			elevator_target_coordinates.w = rc.ch1 * 6;
			elevator_target_coordinates.z += rc.ch2 * 0.05f;
		}
		if (elevator_target_coordinates.y < 9000) {
			elevator_target_coordinates.w = 0;
		}
		
		elevator_update();
		elevator_apply_currents();
		
		uint8_t value;
		read_digital_io(DIGI_IO2, &value);
		elevator_target_coordinates.flywheel_speed = value ? -600 : 0;
  	claw_horizontal_set(rc.sw1 != RC_DN);
		claw_vertical_set(rc.sw1 != RC_UP);
		osDelay(5);
  }
}
