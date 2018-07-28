#include "elevator_task.h"
#include "cmsis_os.h"
#include "uart_device.h"
#include "can_device.h"
#include "elevator_storage.h"

void elevator_task(const void* argu)
{
	
	elevator_init();
	storage_blocker_swing(STORAGE_BLOCKER_SWING_MID);
	set_digital_io_dir(DIGI_IO2, IO_INPUT);
  while(1) {
		elevator_update();
		elevator_target_coordinates.y -= rc.ch4 *0.15;
		elevator_target_coordinates.x += rc.ch3 *0.05;
		uint8_t value;
		read_digital_io(DIGI_IO2, &value);
		
		elevator_target_coordinates.flywheel_speed =  value ? -300 : 0;
		if (rc.kb.bit.Q)
			elevator_target_coordinates.y += 5;
		else if (rc.kb.bit.E)
			elevator_target_coordinates.y -= 5;
  	claw_horizontal_set(rc.sw1 == RC_DN);
		claw_vertical_set(rc.sw1 == RC_UP);
		
			//int16_t theCurrent[3] = {-10000, -10000, -10000};
		//send_mill_motor_current(theCurrent);
		//int16_t theCurrent[3] = {-10000, -10000, -10000};
		//send_mill_motor_current(theCurrent);
		osDelay(5);
  }
}
