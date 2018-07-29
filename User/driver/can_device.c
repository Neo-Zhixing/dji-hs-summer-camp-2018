/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/**
  *********************** (C) COPYRIGHT 2018 DJI **********************
  * @update
  * @history
  * Version     Date              Author           Modification
  * V1.0.0      January-15-2018   ric.luo
  * @verbatim
  *********************** (C) COPYRIGHT 2018 DJI **********************
  */ 
 



#include "can_device.h"
#include "detect_task.h"
#include "sys.h"

/* Elevator Motors */
moto_measure_t motor_elevator[2];
moto_measure_t motor_claw[3];
/* 底盘电机 */
moto_measure_t motor_chassis[4];
moto_measure_t motor_mill[3];
moto_measure_t motor_flywheel[2];



void reset_motor_measurement(moto_measure_t *ptr) {
  ptr->round_cnt = 0;
  ptr->offset_ecd = ptr->ecd;
}

/**
  * @brief     CAN1 中断回调函数，在程序初始化时注册
  * @param     recv_id: CAN1 接收到的数据 ID
  * @param     data: 接收到的 CAN1 数据指针
  */
void can1_recv_callback(uint32_t recv_id, uint8_t data[])
{
	moto_measure_t * motors[8] = {
		&motor_chassis[0],
		&motor_chassis[1],
		&motor_chassis[2],
		&motor_chassis[3],
		&motor_claw[0],
		&motor_claw[1],
		&motor_elevator[0],
		&motor_elevator[1],
	};
	err_id_e errorIDs[8] = {
		CHASSIS_M1_OFFLINE,
		CHASSIS_M2_OFFLINE,
		CHASSIS_M3_OFFLINE,
		CHASSIS_M4_OFFLINE,
		CLAW_OFFLINE,
		CLAW_OFFLINE,
		ELEVATOR_OFFLINE,
		ELEVATOR_OFFLINE,
	};
	
	uint8_t id = (uint8_t)recv_id - 1;
	motors[id]->msg_cnt++ <= 50 ? get_moto_offset(motors[id], data) : \
	encoder_data_handle(motors[id], data);
	err_detector_hook(errorIDs[id]);
}
  
/**
  * @brief     CAN2 中断回调函数，在程序初始化时注册
  * @param     recv_id: CAN2 接收到的数据 ID
  * @param     data: 接收到的 CAN2 数据指针
  */
void can2_recv_callback(uint32_t recv_id, uint8_t data[])
{
	moto_measure_t * motors[6] = {
		&motor_mill[0],
		&motor_mill[1],
		&motor_mill[2],
		&motor_claw[2],
		&motor_flywheel[0],
		&motor_flywheel[1],
	};
	err_id_e errorIDs[6] = {
		MILL_OFFLINE,
		MILL_OFFLINE,
		MILL_OFFLINE,
		DEVICE_NORMAL,
		FLYWHEEL_OFFLINE,
		FLYWHEEL_OFFLINE,
	};
	
	uint8_t id = (uint8_t)recv_id - 1;
	motors[id]->msg_cnt++ <= 50 ? get_moto_offset(motors[id], data) : \
	encoder_data_handle(motors[id], data);
	err_detector_hook(errorIDs[id]);
}

/**
  * @brief     获得电机初始偏差
  * @param     ptr: 电机参数 moto_measure_t 结构体指针
  * @param     data: 接收到的电机 CAN 数据指针
  */
static void get_moto_offset(moto_measure_t *ptr, uint8_t data[])
{
  ptr->ecd        = (uint16_t)(data[0] << 8 | data[1]);
  ptr->offset_ecd = ptr->ecd;
}

/**
  * @brief     计算电机的转速rmp 圈数round_cnt 
  *            总编码器数值total_ecd 总旋转的角度total_angle
  * @param     ptr: 电机参数 moto_measure_t 结构体指针
  * @param     data: 接收到的电机 CAN 数据指针
  */
static void encoder_data_handle(moto_measure_t *ptr, uint8_t data[])
{
  int32_t temp_sum = 0;
  
  ptr->last_ecd      = ptr->ecd;
  ptr->ecd           = (uint16_t)(data[0] << 8 | data[1]);

  ptr->speed_rpm     = (int16_t)(data[2] << 8 | data[3]);

  if (ptr->ecd - ptr->last_ecd > 5000)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -5000)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  ptr->total_angle = ptr->total_ecd * 360 / 8192;
  
  
  ptr->rate_buf[ptr->buf_cut++] = ptr->ecd_raw_rate;
  if (ptr->buf_cut >= FILTER_BUF)
    ptr->buf_cut = 0;
  for (uint8_t i = 0; i < FILTER_BUF; i++)
  {
    temp_sum += ptr->rate_buf[i];
  }
  ptr->filter_rate = (int32_t)(temp_sum/FILTER_BUF);
}

/**
  * @brief     发送底盘电机电流数据到电调
  */
void send_chassis_motor_current(int16_t current[])
{
  static uint8_t data[8];
	for (uint8_t i=0; i<4; i++) {
		data[2*i] = current[i] >> 8;
		data[2*i + 1] = current[i];
	}
  
  write_can(USER_CAN1, CAN_CHASSIS_ID, data);
}
void send_chassis_motor_zero_current(void)
{
  static uint8_t data[8];
	for (uint8_t i=0; i<8; i++)
		data[i] = 0;
  
  write_can(USER_CAN1, CAN_CHASSIS_ID, data);
}

/**
  * @brief     发送Elevator电机电流数据到电调
  */
void send_elevator_motor_current(int16_t elevator_current[], int16_t claw_current[])
{
  static uint8_t data[8];
  
	data[0] = claw_current[0] >> 8;
  data[1] = claw_current[0];
  data[2] = claw_current[1] >> 8;
  data[3] = claw_current[1];
	
  data[4] = elevator_current[0]>> 8;
  data[5] = elevator_current[0];
  data[6] = elevator_current[1] >> 8;
  data[7] = elevator_current[1];

  
  write_can(USER_CAN1, CAN_ELEVATOR_ID, data);
}

static int16_t mill_currents[3] = {0, 0, 0};
static int16_t claw_rotate_current = 0;

void send_mill_motor_current(int16_t current[])
{
  static uint8_t data[8];
	for (uint8_t i=0; i<3; i++) {
		data[2*i] = current[i] >> 8;
		data[2*i + 1] = current[i];
		mill_currents[i] = current[i];
	}
	data[6] = claw_rotate_current >> 8;
	data[7] = claw_rotate_current;
  
  write_can(USER_CAN2, CAN_MILL_ID, data);
}

void send_flywheel_motor_current(int16_t current[])
{
	static uint8_t data[8];
	for(uint8_t i=0; i<2; i++) {
		data[i*2] = current[i] >> 8;
		data[i*2+1] = current[i];
	}
  write_can(USER_CAN2, CAN_FLYWHEEL_ID, data);
}

void send_claw_rotate_motor_current(int16_t current)
{
	claw_rotate_current = current;
  static uint8_t data[8];
	for (uint8_t i=0; i<3; i++) {
		data[2*i] = mill_currents[i] >> 8;
		data[2*i + 1] = mill_currents[i];
	}
	data[6] = current >> 8;
	data[7] = current;
  
  write_can(USER_CAN2, CAN_MILL_ID, data);
}
