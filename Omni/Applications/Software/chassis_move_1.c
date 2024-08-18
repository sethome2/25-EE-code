/**
 * @file chassis_move.c
 * @author sethome
 * @brief
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "chassis_move.h"
#include "CAN_receive&send.h"
#include "math.h"
#include "pid.h"
#include "Stm32_time.h"
#include "small_tools.h"
#include "cap_ctl.h"

#include "stdlib.h"
#include "stdio.h"
#include "stdint.h"

#include "referee_handle_pack.h"

// wheel conf
#define WHEEL_RADIUS 0.15240f // m
#define PI 3.1415926f

// car conf
#define ROLLER_DISTANCE 100 // mm  轴距
#define WHEELS_DISTANCE 100 // mm  轮距

struct chassis_status chassis;
struct cap cap_chassis;	//电容组
// mm/s
#define FR 0
#define FL 1
#define BL 2
#define BR 3
float wheel_rpm[4]; // 底盘速度数组
extern float Plimit=1.0f;	//等比系数

// 马达速度环PID
pid_t motor_speed[4];

// 初始化底盘
void chassis_move_init()
{
	chassis.speed.max_x = 5.0f; // m/s
	chassis.speed.max_y = 5.0f; // m/s
	chassis.speed.max_r = 5;	//

	chassis.acc.max_x = 1.0f; // m/^2
	chassis.acc.max_y = 1.0f; // m/^2
	chassis.acc.max_r = 300;  //

	//底盘PID初始化
//	pid_set(&motor_speed[FR], 6000, 0, 20, 16000, 0);
//	pid_set(&motor_speed[FL], 6000, 0, 20, 16000, 0);
//	pid_set(&motor_speed[BL], 6000, 0, 20, 16000, 0);
//	pid_set(&motor_speed[BR], 6000, 0, 20, 16000, 0);
		pid_set(&motor_speed[FR], 12000, 0, 100, 16000, 0);
		pid_set(&motor_speed[FL], 12000, 0, 100, 16000, 0);
		pid_set(&motor_speed[BL], 12000, 0, 100, 16000, 0);
		pid_set(&motor_speed[BR], 12000, 0, 100, 16000, 0);
}

//限制值
inline void val_limit(float *val, float MAX)
{
	if (fabs(*val) > MAX)
	{
		if (*val > 0)
			*val = MAX;
		else
			*val = -MAX;
	}
}

//限制变化量
inline void change_limit(float last, float *now, float limit)
{
	float change = *now - last;
	if (fabs(change) > limit)
	{
		if (change > 0)
			*now = last + limit;
		else
			*now = last - limit;
	}
}

float chassis_power_limit()	//功率限制函数
{
	static float power;
	static uint16_t power_buffer;
	get_chassis_power_and_buffer(&power,&power_buffer);

	cap_chassis.cap_now_vol=24;
	cap_chassis.cap_last_vol=24;
	cap_chassis.cap_now_vol=cap_get_remain_vol();
	cap_chassis.cap_error_vol=cap_chassis.cap_now_vol-cap_chassis.cap_last_vol;//电容组电压变化量
	if(cap_chassis.cap_error_vol>1)	//电容组充电
	{
		Plimit=0.5;
	}
	else if(cap_chassis.cap_error_vol<0)	//电容组放电
	{
		Plimit=(cap_chassis.cap_now_vol+1)/12;
	}
	if(cap_chassis.cap_now_vol<6)	//电容组没电
	{
		Plimit=0.1;
	}
	if(power_buffer<10)	//缓冲能量过低，限死
	{
		Plimit=0.000001;
	}
	cap_chassis.cap_last_vol=cap_chassis.cap_now_vol;
	return Plimit;
}

// 计算底盘马达速度
void chassis_moto_speed_calc()
{
	// 最大速度限制
	val_limit(&chassis.speed.x, chassis.speed.max_x);
	val_limit(&chassis.speed.y, chassis.speed.max_y);
	val_limit(&chassis.speed.r, chassis.speed.max_r);

	decode_as_3508(CAN_1_1);
	decode_as_3508(CAN_1_2);
	decode_as_3508(CAN_1_3);
	decode_as_3508(CAN_1_4);

	// 计算速度分量
	wheel_rpm[FR] = +chassis.speed.x - chassis.speed.y + chassis.speed.r;
	wheel_rpm[FL] = +chassis.speed.x + chassis.speed.y + chassis.speed.r;
	wheel_rpm[BL] = -chassis.speed.x + chassis.speed.y + chassis.speed.r;
	wheel_rpm[BR] = -chassis.speed.x - chassis.speed.y + chassis.speed.r;

	// 当前速度
	chassis.speed.now_x = wheel_rpm[FL] / 2.0f - wheel_rpm[BL] / 2.0f;
	chassis.speed.now_y = wheel_rpm[FL] / 2.0f - wheel_rpm[FR] / 2.0f;
	chassis.speed.now_r = wheel_rpm[FR] / 2.0f + wheel_rpm[BL] / 2.0f;

	// 计算加速度
	uint32_t now_time = Get_sys_time_ms();
	static uint32_t last_time = 0;
	float dt = (now_time - last_time) / 1000.0f;
	last_time = now_time;
	chassis.acc.now_x = (chassis.speed.x - chassis.speed.now_x) / dt;
	chassis.acc.now_y = (chassis.speed.y - chassis.speed.now_y) / dt;
	chassis.acc.now_r = (chassis.speed.r - chassis.speed.now_r) / dt;

	// 限制加速度
	// if (fabs(chassis.acc.now_x) > chassis.acc.max_x)
	// {
	// 	if (chassis.speed.x < 0.0f)
	// 		chassis.speed.x = -(chassis.acc.max_x * dt + chassis.speed.now_x);
	// 	chassis.speed.x = chassis.acc.max_x * dt + chassis.speed.now_x;
	// }
	// if (fabs(chassis.acc.now_y) > chassis.acc.max_y)
	// {
	// 	if (chassis.speed.y < 0.0f)
	// 		chassis.speed.y = -(chassis.acc.max_y * dt + chassis.speed.now_y);
	// 	chassis.speed.y = chassis.acc.max_y * dt + chassis.speed.now_y;
	// }
	// if (fabs(chassis.acc.now_r) > chassis.acc.max_r)
	// {
	// 	if (chassis.speed.r < 0.0f)
	// 		chassis.speed.r = -(chassis.acc.max_r * dt + chassis.speed.now_r);
	// 	chassis.speed.r = chassis.acc.max_r * dt + chassis.speed.now_r;
	// }
	
//	Plimit=(cap_get_remain_vol()+1)/24;	//等比缩放

	chassis_power_limit();
	
	// 计算马达电流
	chassis.wheel_current[FR] = pid_cal(&motor_speed[FR], get_motor_data(chassis_FR).round_speed * WHEEL_RADIUS * PI, wheel_rpm[FR]);
	chassis.wheel_current[FL] = pid_cal(&motor_speed[FL], get_motor_data(chassis_FL).round_speed * WHEEL_RADIUS * PI, wheel_rpm[FL]);
	chassis.wheel_current[BL] = pid_cal(&motor_speed[BL], get_motor_data(chassis_BL).round_speed * WHEEL_RADIUS * PI, wheel_rpm[BL]);
	chassis.wheel_current[BR] = pid_cal(&motor_speed[BR], get_motor_data(chassis_BR).round_speed * WHEEL_RADIUS * PI, wheel_rpm[BR]);

	//限制
	val_limit(&motor_speed[FR].total_out,16384);
	val_limit(&motor_speed[FL].total_out,16384);
	val_limit(&motor_speed[BL].total_out,16384);
	val_limit(&motor_speed[BR].total_out,16384);

	// 设定马达电流 （在freeRTOS中发送）
//	set_motor((Plimit*chassis.wheel_current[FR]), chassis_FR);
//	set_motor((Plimit*chassis.wheel_current[FL]), chassis_FL);
//	set_motor((Plimit*chassis.wheel_current[BL]), chassis_BL);
//	set_motor((Plimit*chassis.wheel_current[BR]), chassis_BR);

	set_motor(chassis.wheel_current[FR], chassis_FR);
	set_motor(chassis.wheel_current[FL], chassis_FL);
	set_motor(chassis.wheel_current[BL], chassis_BL);
	set_motor(chassis.wheel_current[BR], chassis_BR);

	chassis.speed.last_x = chassis.speed.now_x;
	chassis.speed.last_y = chassis.speed.now_y;
	chassis.speed.last_r = chassis.speed.now_r;
}

int RampInc_float(int16_t *buffer, float now, float ramp)
{
	if (*buffer > 0)
	{
		if (*buffer > ramp)
		{
			now += ramp;
			*buffer -= ramp;
		}
		else
		{
			now += *buffer;
			*buffer = 0;
		}
	}
	else
	{
		if (*buffer < -ramp)
		{
			now += -ramp;
			*buffer -= -ramp;
		}
		else
		{
			now += *buffer;
			*buffer = 0;
		}
	}

	return now;
}
