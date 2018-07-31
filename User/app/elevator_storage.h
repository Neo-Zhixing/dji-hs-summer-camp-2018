#ifndef __ELEVATOR_STORAGE_H__
#define __ELEVATOR_STORAGE_H__

#include "sys.h"

#define ELEVATOR_LIMIT_SWITCH_LEFT     5
#define ELEVATOR_LIMIT_SWITCH_RIGHT    6
#define CLAW_X_LIMIT_SWITCH            7
#define CLAW_Z_LIMIT_SWITCH            4


typedef struct {
	uint8_t x, y;
} coordinate_t;

typedef struct {
	float x, y, z;
	float	w;
	float flywheel_speed;
} elevator_target_coordinates_t;
extern elevator_target_coordinates_t elevator_target_coordinates;
void elevator_init(void);


extern int16_t elevator_power[];
extern int16_t claw_power[];
extern int16_t flywheel_power[];

void elevator_update(void);
void elevator_apply_currents(void);

typedef enum {
	STORAGE_BLOCKER_SWING_LEFT = 1300,
	STORAGE_BLOCKER_SWING_RIGHT = 1700,
	STORAGE_BLOCKER_SWING_MID = 1500,
	STORAGE_BLOCKER_SWING_OPEN_LEFT = 950,
	STORAGE_BLOCKER_SWING_OPEN_RIGHT = 1950,
} storage_blocker_swing_state;
void storage_blocker_swing(storage_blocker_swing_state state);

void claw_horizontal_set(uint8_t closed);
void claw_vertical_set(uint8_t closed);
void elevator_move_to_storage_coordinates(coordinate_t * theCoords);

#endif
