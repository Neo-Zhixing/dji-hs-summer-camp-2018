#include "elevator_task.h"
#include "cmsis_os.h"
#include "uart_device.h"
#include "can_device.h"
#include "elevator_storage.h"

uint8_t bit_values[8];


void elevator_task(const void* argu)
{
	elevator_init();
	storage_blocker_swing(STORAGE_BLOCKER_SWING_MID);
	set_pwm_group_param(PWM_GROUP3, 20000);
	start_pwm_output(PWM_IO9); 
	start_pwm_output(PWM_IO10);
	
  while(1) {
		if (rc.sw2 == RC_DN) {
			elevator_target_coordinates.x += rc.ch1 * 0.05f;
			elevator_target_coordinates.y += rc.ch4 * 0.05f;
			elevator_target_coordinates.w = rc.ch3 * 6;
			elevator_target_coordinates.z += rc.ch2 * 0.02f;
		}
		coordinate_t coords;
		if (rc.kb.bit.A) {
			coords.x = 0;
			coords.y = 0;
			elevator_move_to_storage_coordinates(&coords);
		} else if (rc.kb.bit.B) {
			coords.x = 0;
			coords.y = 1;
			elevator_move_to_storage_coordinates(&coords);
		} else if (rc.kb.bit.C) {
			coords.x = 0;
			coords.y = 2;
			elevator_move_to_storage_coordinates(&coords);
		} else if (rc.kb.bit.D) {
			coords.x = 1;
			coords.y = 0;
			elevator_move_to_storage_coordinates(&coords);
		} else if (rc.kb.bit.E) {
			coords.x = 1;
			coords.y = 1;
			elevator_move_to_storage_coordinates(&coords);
		} else if (rc.kb.bit.F) {
			coords.x = 1;
			coords.y = 2;
			elevator_move_to_storage_coordinates(&coords);
		}
		
		if (rc.kb.bit.X) {
			reset_motor_measurement(&motor_claw[0]);
			reset_motor_measurement(&motor_claw[1]);
			elevator_target_coordinates.z = 0;
			elevator_target_coordinates.x = 0;
		}
		
		elevator_update();
		elevator_apply_currents();
		
		uint8_t value;
		for(uint8_t i=0; i<8; i++) {
			read_digital_io(i+1, &value);
			bit_values[i] = value;
		}
		read_digital_io(2, &value);
		
		if (rc.kb.bit.SHIFT) {
			elevator_target_coordinates.flywheel_speed = -1500;
			int16_t currents[3] = {0, 0, 0};
			currents[coords.y] = 1000;
			send_mill_motor_current(currents);
			storage_blocker_swing(coords.x ? STORAGE_BLOCKER_SWING_LEFT : STORAGE_BLOCKER_SWING_RIGHT);
		}	else if (rc.kb.bit.CTRL) {
			elevator_target_coordinates.flywheel_speed = 1500;
			int16_t currents[3] = {0, 0, 0};
			currents[coords.y] = -1000;
			send_mill_motor_current(currents);
			storage_blocker_swing(coords.x ? STORAGE_BLOCKER_SWING_LEFT : STORAGE_BLOCKER_SWING_RIGHT);
		} else {
			elevator_target_coordinates.flywheel_speed = 0;
			int16_t currents[3] = {0, 0, 0};
			send_mill_motor_current(currents);
			storage_blocker_swing(STORAGE_BLOCKER_SWING_MID);
		}
		
		
  	claw_horizontal_set(rc.sw1 != RC_DN);
		claw_vertical_set(0);
		
		set_pwm_param(PWM_IO9, rc.kb.bit.Q ? 1300 : 1500);
		set_pwm_param(PWM_IO10, rc.kb.bit.R ? 1300 : 1500);
		
		osDelay(5);
  }
}
