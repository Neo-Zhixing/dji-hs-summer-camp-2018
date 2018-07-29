#include "elevator_storage.h"

#include "cmsis_os.h"

typedef enum {
	BLOCK_EMPTY,
	BLOCK_RED,
	BLOCK_GREEN,
	BLOCK_YELLOW,
	BLOCK_BLUE,
} block_t;


const uint8_t block_storage_x_slot_number = 2;
const uint8_t block_storage_y_slot_number = 3;
block_t block_storage[block_storage_x_slot_number][block_storage_y_slot_number];

typedef struct {
	uint8_t x, y;
} coordinate_t;

/*
	-------------
2   |     |     |
	|     |     |
	-------------
1   |     |     |
	|     |     |
	-------------
0   |     |     |
	|     |     |
y   -------------
  x    0     1
*/

block_t block_storage_get(coordinate_t * coordinate) {
	return block_storage[coordinate->x][coordinate->y];
}

void block_storage_set(coordinate_t * coordinate, block_t block) {
	block_storage[coordinate->x][coordinate->y] = block;
}

static coordinate_t block_storage_cooordinates;
coordinate_t * block_storage_coordinate_for_block(block_t block) {
	for (uint8_t x=0; x < block_storage_x_slot_number; x++)
		for (uint8_t y=0; y < block_storage_y_slot_number; y++) {
		if (block_storage[x][y] == block) {
			block_storage_cooordinates.x = x; 
			block_storage_cooordinates.y = y;
			return &block_storage_cooordinates;
		}
	}
	return NULL;
}


void storage_blocker_swing(storage_blocker_swing_state state) {
	set_pwm_param(PWM_IO7, state);
}

void claw_horizontal_set(uint8_t closed) {
	set_pwm_param(PWM_IO1, closed ? 2000 : 1730);
	set_pwm_param(PWM_IO2, closed ? 960 : 1200);
	set_pwm_param(PWM_IO3, closed ? 2050 : 1780);
	set_pwm_param(PWM_IO4, closed ? 1000 : 1350);
}

void claw_vertical_set(uint8_t closed) {
	set_pwm_param(PWM_IO5, closed ? 1000 : 1500);
	set_pwm_param(PWM_IO6, closed ? 950 : 1300);
}


#include "pid.h"
#include "can_device.h"
elevator_target_coordinates_t elevator_target_coordinates;

struct {
	pid_t balance[2];
 	pid_t elevator_pos[2];
	pid_t elevator_speed[2];
	pid_t claw_pos[2];
	pid_t flywheel_speed[2];
	pid_t claw_rotate_pos, claw_rotate_speed;
} elevator_pids;


void elevator_init() {
	set_digital_io_dir(ELEVATOR_LIMIT_SWITCH_LEFT, IO_INPUT);
	set_digital_io_dir(ELEVATOR_LIMIT_SWITCH_RIGHT, IO_INPUT);
	set_digital_io_dir(CLAW_X_LIMIT_SWITCH, IO_INPUT);
	set_digital_io_dir(CLAW_Z_LIMIT_SWITCH, IO_INPUT);
	pid_init(&elevator_pids.balance[0], 500, 1000, 3, 0, 0);
	pid_init(&elevator_pids.balance[1], 500, 1000, 3, 0, 0);
	pid_init(&elevator_pids.elevator_pos[0], 5000, 1000, 1.5, 0.03, 0);
	pid_init(&elevator_pids.elevator_pos[1], 5000, 1000, 1.5, 0, 0);
	pid_init(&elevator_pids.elevator_speed[0], C620_MAX_CURRENT*0.95, 1000, 1, 0, 0);
	pid_init(&elevator_pids.elevator_speed[1], C620_MAX_CURRENT*0.95, 1000, 1, 0, 0);
	pid_init(&elevator_pids.claw_pos[0], GM3510_MAX_CURRENT*0.95, 1000, 30, 0.2f, 0);
	pid_init(&elevator_pids.claw_pos[1], GM3510_MAX_CURRENT*0.95, 1000, 30, 0, 0);
	pid_init(&elevator_pids.flywheel_speed[0], 1000, 100, 1, 0, 0);
	pid_init(&elevator_pids.flywheel_speed[1], 1000, 100, 1, 0, 0);

	pid_init(&elevator_pids.claw_rotate_pos, 1000, 100, 2, 0.1, 0.5);
	pid_init(&elevator_pids.claw_rotate_speed, 3000, 100, 1.5, 0, 0.2);
	set_pwm_group_param(PWM_GROUP1, 20000);
	set_pwm_group_param(PWM_GROUP2, 20000);
	claw_horizontal_set(1);
	claw_vertical_set(1);
	storage_blocker_swing(STORAGE_BLOCKER_SWING_MID);
	for (uint8_t i=1; i<=6; i++)
		start_pwm_output(i);

	reset_motor_measurement(&motor_elevator[0]);
	reset_motor_measurement(&motor_elevator[1]);
	reset_motor_measurement(&motor_claw[0]);
	reset_motor_measurement(&motor_claw[1]);
	
	elevator_target_coordinates.x = 0;
	elevator_target_coordinates.y = 0;
	elevator_target_coordinates.z = 0;
	elevator_target_coordinates.w = 0;
}

int16_t elevator_power[2];
int16_t claw_power[2];
int16_t flywheel_power[2];
int16_t claw_rotate_power;
void elevator_update() {
	if (elevator_target_coordinates.x > 7000) elevator_target_coordinates.x = 7000;
	if (elevator_target_coordinates.y < 0) elevator_target_coordinates.y = 0;
	else if (elevator_target_coordinates.y > 15000) elevator_target_coordinates.y = 15000;
	if (elevator_target_coordinates.z > 4600) elevator_target_coordinates.z = 4600;
	float current_elevator_pos = (float)(motor_elevator[0].total_angle - motor_elevator[1].total_angle) / 2.0f;

	
	elevator_power[0] = -pid_calc(&elevator_pids.balance[0], -motor_elevator[0].total_angle, current_elevator_pos) \
		- pid_calc(&elevator_pids.elevator_pos[0], -motor_elevator[0].total_angle, elevator_target_coordinates.y);
	elevator_power[1] = pid_calc(&elevator_pids.balance[1], motor_elevator[1].total_angle, current_elevator_pos) \
		+ pid_calc(&elevator_pids.elevator_pos[1], motor_elevator[1].total_angle, elevator_target_coordinates.y);
	
	for(uint8_t i=0; i<2; i++) {
		elevator_power[i] = pid_calc(&elevator_pids.elevator_speed[i], motor_elevator[i].speed_rpm, elevator_power[i]);
	}
	
	flywheel_power[0] = pid_calc(&elevator_pids.flywheel_speed[0], motor_flywheel[0].speed_rpm, elevator_target_coordinates.flywheel_speed);
	flywheel_power[1] = pid_calc(&elevator_pids.flywheel_speed[1], motor_flywheel[1].speed_rpm, -elevator_target_coordinates.flywheel_speed);
	claw_power[0] = pid_calc(&elevator_pids.claw_pos[0], motor_claw[0].total_angle, -elevator_target_coordinates.x);
	claw_power[1] = pid_calc(&elevator_pids.claw_pos[1], motor_claw[1].total_angle, elevator_target_coordinates.z);
	claw_rotate_power = pid_calc(&elevator_pids.claw_rotate_pos, motor_claw[2].total_angle, elevator_target_coordinates.w);
	claw_rotate_power = pid_calc(&elevator_pids.claw_rotate_speed, motor_claw[2].speed_rpm, claw_rotate_power);

	
	uint8_t elevator_left_limit, elevator_right_limit, claw_x, claw_z;
	read_digital_io(ELEVATOR_LIMIT_SWITCH_LEFT, &elevator_left_limit);
	read_digital_io(ELEVATOR_LIMIT_SWITCH_RIGHT, &elevator_right_limit);
	read_digital_io(CLAW_X_LIMIT_SWITCH, &claw_x);
	read_digital_io(CLAW_Z_LIMIT_SWITCH, &claw_z);
	
	if (!claw_x) {
		reset_motor_measurement(&motor_claw[0]);
		if(elevator_target_coordinates.x < 0) elevator_target_coordinates.x = 0;
	}
	if (!claw_z) {
		reset_motor_measurement(&motor_claw[1]);
		if(elevator_target_coordinates.z < 0) elevator_target_coordinates.z = 0;
	}
	if (!elevator_left_limit)
		reset_motor_measurement(&motor_elevator[0]);
	if (!elevator_right_limit)
		reset_motor_measurement(&motor_elevator[1]);
}

void elevator_apply_currents() {
	send_elevator_motor_current(elevator_power, claw_power);
	send_flywheel_motor_current(flywheel_power);
	send_claw_rotate_motor_current(claw_rotate_power);
}

static int16_t abs(int16_t value) {
	return value > 0 ? value : -value;
}
static float fabs(float value) {
	return value > 0 ? value : -value;
}
uint8_t elevator_ready() {
	return \
	(abs(motor_elevator[0].speed_rpm) < 20) && (fabs(elevator_pids.elevator_pos[0].err[NOW]) < 20) && \
	(abs(motor_elevator[1].speed_rpm) < 20) && (fabs(elevator_pids.elevator_pos[1].err[NOW]) < 20) && \
	(fabs(elevator_pids.balance[0].err[NOW] < 20)) && (fabs(elevator_pids.balance[0].err[NOW]) < 20) && \
	(abs(motor_claw[0].speed_rpm) < 20) && (fabs(elevator_pids.claw_pos[0].err[NOW]) < 20) && \
	(abs(motor_claw[1].speed_rpm) < 20) && (fabs(elevator_pids.claw_pos[1].err[NOW]) < 20);
	
}

const float elevator_position_x[2] = {
	300,
	600,
};
const float elevator_position_y[3] = {
	300,
	600, 
	900,
};

void elevator_move_to_storage_coordinates(coordinate_t * theCoords) {
	elevator_target_coordinates.x = elevator_position_x[theCoords->x];
	elevator_target_coordinates.y = elevator_position_y[theCoords->y];
	//elevator_target_coordinates.z = AFIXEDVALUE;
}

void elevator_move_to_coordinates(elevator_target_coordinates_t * theCoords) {
	elevator_target_coordinates.x = theCoords->x;
	elevator_target_coordinates.y = theCoords->y;
	elevator_target_coordinates.z = theCoords->z;
	elevator_target_coordinates.w = theCoords->w;
}

void elevator_store_block(block_t theBlock) {
	coordinate_t * theCoords = block_storage_coordinate_for_block(BLOCK_EMPTY);
	if (theCoords == NULL) {
		return;
	}
	storage_blocker_swing(theCoords->x == 0 ? STORAGE_BLOCKER_SWING_RIGHT : STORAGE_BLOCKER_SWING_LEFT);
	elevator_move_to_storage_coordinates(theCoords);
	
	
	block_storage_set(theCoords, theBlock);
}


