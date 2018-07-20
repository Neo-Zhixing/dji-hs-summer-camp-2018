#ifndef __COLOR_SENSOR_H__
#define __COLOR_SENSOR_H__

#include "sys.h"

typedef enum {
	COLOR_SENSOR_RAW = 0x04,
	COLOR_SENSOR_LCC = 0x02,
  COLOR_SENSOR_RGB  = 0x01,
} color_sensor_mode;

typedef enum {
	COLOR_BLUE = 1,
	COLOR_CYAN = 2,
	COLOR_RED = 3,
	COLOR_GREEN = 4,
	COLOR_PINK = 5,
	COLOR_YELLOW = 6,
	COLOR_WHITE = 7,
	COLOR_BLACK = 8,
	COLOR_UNKNOWN = 0,
} color_code;

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} color_rgb;

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t c;
} color_raw;

typedef struct {
	color_code color;
	uint16_t lux;
	uint16_t ct;
} color_lcc;

typedef union {
	color_lcc lcc;
	color_rgb rgb;
	color_raw raw;
} color_data;

void color_sensor_init(uint8_t uart_id);
void color_sensor_brightness(uint8_t uart_id, uint8_t brightness);
void color_sensor_calibrate(uint8_t uart_id);
void color_sensor_config(uint8_t uart_id, uint8_t save, uint8_t mode);
color_data color_sensor_get(uint8_t uart_id);

#endif
