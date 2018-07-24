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
	set_pwm_param(PWM_IO4, state);
}

void claw_horizontal_set(uint8_t closed) {
	uint32_t delta = closed ? 500 : 0;
	uint32_t base = 1500;
	set_pwm_param(PWM_IO1, base + delta);
	set_pwm_param(PWM_IO2, base - delta);
}

void claw_vertical_set(uint8_t closed) {
	set_pwm_param(PWM_IO3, closed ? 1000 : 1500);
}


#include "pid.h"
#include "can_device.h"
elevator_target_coordinates_t elevator_target_coordinates;

static struct {
	pid_t left_balance, right_balance;
	pid_t elevator_pos, claw_pos;
} elevator_pids;

#define ELEVATOR_LIMIT_SWITCH_LEFT     5
#define ELEVATOR_LIMIT_SWITCH_RIGHT    6
#define CLAW_MOVE_LIMIT_SWITCH         7
void elevator_init() {
	set_digital_io_dir(ELEVATOR_LIMIT_SWITCH_LEFT, IO_INPUT);
	set_digital_io_dir(ELEVATOR_LIMIT_SWITCH_RIGHT, IO_INPUT);
	set_digital_io_dir(CLAW_MOVE_LIMIT_SWITCH, IO_INPUT);
	pid_init(&elevator_pids.left_balance, GM3510_MAX_CURRENT*0.2, 1000, 80, 5, 0);
	pid_init(&elevator_pids.right_balance, GM3510_MAX_CURRENT*0.2, 1000, 80, 5, 0);
	pid_init(&elevator_pids.elevator_pos, GM3510_MAX_CURRENT*0.95, 1000, 30, 0, 0);
	pid_init(&elevator_pids.claw_pos, GM3510_MAX_CURRENT*0.95, 1000, 30, 0, 0);
	
	set_pwm_group_param(PWM_GROUP1, 20000);
	set_pwm_group_param(PWM_GROUP2, 20000);
	claw_horizontal_set(1);
	claw_vertical_set(1);
	for (uint8_t i=1; i<=5; i++) {
		start_pwm_output(i);
	}
	
	/*
	uint8_t elevator_switch_left = 0;
	uint8_t elevator_switch_right = 0;
	uint8_t claw_switch = 0;
	while(!elevator_switch_left && !elevator_switch_right && !claw_switch) {
		read_digital_io(ELEVATOR_LIMIT_SWITCH_LEFT, &elevator_switch_left);
		read_digital_io(ELEVATOR_LIMIT_SWITCH_RIGHT, &elevator_switch_right);
		read_digital_io(CLAW_MOVE_LIMIT_SWITCH, &claw_switch);
		send_elevator_motor_current(
			elevator_switch_left * GM3510_MAX_CURRENT,
			elevator_switch_left * GM3510_MAX_CURRENT,
			claw_switch * GM3510_MAX_CURRENT
		);
	}
	initial_coordinates.elevator_left = motor_elevator_left.total_angle;
	initial_coordinates.elevator_right = motor_elevator_right.total_angle;
	initial_coordinates.claw_move = motor_claw_move.total_angle;
	*/
	reset_motor_measurement(&motor_elevator_left);
	reset_motor_measurement(&motor_elevator_right);
	reset_motor_measurement(&motor_claw_move);
	
	elevator_target_coordinates.x = 0;
	elevator_target_coordinates.y = 0;
}

void elevator_update() {
	int32_t left_pos = motor_elevator_left.total_angle;
	int32_t right_pos = motor_elevator_right.total_angle;
	float current_elevator_pos = (float)(left_pos + right_pos) / 2.0f;
	
	float left_balance_speed = pid_calc(&(elevator_pids.left_balance), left_pos, current_elevator_pos);
	float right_balance_speed = pid_calc(&(elevator_pids.right_balance), right_pos, current_elevator_pos);
	
	float elevation_speed = pid_calc(&(elevator_pids.elevator_pos), current_elevator_pos, elevator_target_coordinates.y);
	
	float left_speed = left_balance_speed + elevation_speed;
	float right_speed = right_balance_speed + elevation_speed;
	
	int32_t claw_pos = motor_claw_move.total_angle;
	float claw_move_speed = pid_calc(&(elevator_pids.claw_pos), claw_pos, elevator_target_coordinates.x);
	/*
	if ((left_pos < -43138 && left_speed < 0) || (left_pos > 0 && left_speed > 0))
		left_speed = 0;
	
	if ((right_pos < -43138 && left_speed < 0) || (right_pos > 0 && right_speed > 0))
		right_speed = 0;
	*/
	send_elevator_motor_current((int16_t)left_speed, (int16_t)right_speed, (int16_t) claw_move_speed);
	
	uint8_t value;
	read_digital_io(CLAW_MOVE_LIMIT_SWITCH, &value);
	write_led_io(LED_IO1, value ? LED_ON : LED_OFF);
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


