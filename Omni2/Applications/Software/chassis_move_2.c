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
 

#include "stdlib.h"
#include "stdio.h"
#include "stdint.h"

#include "math.h"
#include "pid.h" 
#include "CAN_receive&send.h" 
#include "chassis_move.h"
#include "referee_handle_pack.h"
#include "cap_ctl.h"
#include "Stm32_time.h"
//引入斜坡函数。
#include "RampFunc.h"

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
float Plimit=1.0f;	//等比系数

float Power;
uint16_t Engerny_buffer;

// 马达速度环PID
pid_t motor_speed[4];

// 初始化底盘
void chassis_move_init()
{
	chassis.speed.max_x = 7.0f; // m/s 
	chassis.speed.max_y = 7.0f; // m/s
	chassis.speed.max_r = 5.0f;	//

	chassis.acc.max_x = 2.5f; // 1m/^2
	chassis.acc.max_y = 2.5f; // m/^2
	chassis.acc.max_r = 2.5f;  //

	pid_set(&motor_speed[FR], 8000, 0, 250, MAX_CURRENT, 0);
	pid_set(&motor_speed[FL], 8000, 0, 250, MAX_CURRENT, 0);
	pid_set(&motor_speed[BL], 8000, 0, 250, MAX_CURRENT, 0);
	pid_set(&motor_speed[BR], 8000, 0, 250, MAX_CURRENT, 0);
	srand(2); //初始化一个随机数种子，为了之后变速小陀螺使用
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


float now_p = 0.0f;
float b = 0.015f;
float power_limit(int16_t current[4])
{
	
//	float max_p; // = REFEREE_DATA.Chassis_Power_Limit - 2.0f; // 2w余量

//	if (cap.remain_vol <= 8)
//		max_p = REFEREE_DATA.Chassis_Power_Limit - 2.0f; // 2w余量
//	else if (cap.remain_vol > 8)
//	{
////			max_p += 14 * cap.remain_vol; 
//				max_p = REFEREE_DATA.Chassis_Power_Limit + cap.remain_vol * 14;
//	}
//	
//	if(max_p >= REFEREE_DATA.Chassis_Power_Limit + cap.remain_vol * 14)
//		max_p = REFEREE_DATA.Chassis_Power_Limit + cap.remain_vol * 14;

//	now_p = 0;
	
	
float max_p; // = REFEREE_DATA.Chassis_Power_Limit - 2.0f; // 2w余量

	if (cap.remain_vol <= 5)
		max_p = REFEREE_DATA.Chassis_Power_Limit - 2.0f; // 2w余量
	else if (cap.remain_vol > 5)
	{
			max_p = REFEREE_DATA.Chassis_Power_Limit + 100; // 超电最大功率 = 超电电压 * 14A 线圈最大电流 
	}
	
	if(max_p >= REFEREE_DATA.Chassis_Power_Limit + cap.remain_vol * 14)
		max_p = REFEREE_DATA.Chassis_Power_Limit + cap.remain_vol * 14;
	
	now_p = 0;	
	
	
	
	
	const float a = 1.23e-07;	 // k1
	const float k2 = 1.453e-07; // k2
	const float constant = 4.081f;
	const float toque_coefficient = (20.0f/16384.0f)*(0.3f)*(187.0f/3591.0f)/9.55f;
	for (int i = 0; i < 4; i++)
	{
		// 估算功率
		// 西交利物浦：https://github.com/MaxwellDemonLin/Motor-modeling-and-power-control/blob/master/chassis_power_control.c#L89
		now_p += fabs(current[i] * toque_coefficient * get_motor_data(i).speed_rpm +
					  k2 * get_motor_data(i).speed_rpm * get_motor_data(i).speed_rpm +
					  a * current[i] * current[i] + constant) / 0.85f;
	}

	float percentage = max_p / now_p;
	
	if (percentage > 1.0f)
		return 1.0f;
	return percentage - b;
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
	 if (fabs(chassis.acc.now_x) > chassis.acc.max_x)
	 {
	 	if (chassis.speed.x < 0.0f)
	 		chassis.speed.x = -(chassis.acc.max_x * dt + chassis.speed.now_x);
	 	chassis.speed.x = chassis.acc.max_x * dt + chassis.speed.now_x;
	 }
	 if (fabs(chassis.acc.now_y) > chassis.acc.max_y)
	 {
	 	if (chassis.speed.y < 0.0f)
	 		chassis.speed.y = -(chassis.acc.max_y * dt + chassis.speed.now_y);
	 	chassis.speed.y = chassis.acc.max_y * dt + chassis.speed.now_y;
	  }
	 if (fabs(chassis.acc.now_r) > chassis.acc.max_r)
	 {
	 	if (chassis.speed.r < 0.0f)
	 		chassis.speed.r = -(chassis.acc.max_r * dt + chassis.speed.now_r);
	 	chassis.speed.r = chassis.acc.max_r * dt + chassis.speed.now_r;
	 }

	 
	// 计算马达电流
	chassis.wheel_current[FR] = pid_cal(&motor_speed[FR], get_motor_data(chassis_FR).round_speed * WHEEL_RADIUS * PI, wheel_rpm[FR]);
	chassis.wheel_current[FL] = pid_cal(&motor_speed[FL], get_motor_data(chassis_FL).round_speed * WHEEL_RADIUS * PI, wheel_rpm[FL]);
	chassis.wheel_current[BL] = pid_cal(&motor_speed[BL], get_motor_data(chassis_BL).round_speed * WHEEL_RADIUS * PI, wheel_rpm[BL]);
	chassis.wheel_current[BR] = pid_cal(&motor_speed[BR], get_motor_data(chassis_BR).round_speed * WHEEL_RADIUS * PI, wheel_rpm[BR]);


	Plimit = power_limit(chassis.wheel_current);
	
	// 设定马达电流 （在freeRTOS中发送）

	set_motor((Plimit*chassis.wheel_current[FR]), chassis_FR);
	set_motor((Plimit*chassis.wheel_current[FL]), chassis_FL);
	set_motor((Plimit*chassis.wheel_current[BL]), chassis_BL);
	set_motor((Plimit*chassis.wheel_current[BR]), chassis_BR);//暂时注释掉，我现在只调试云台
	// set_motor(chassis.wheel_current[FR], chassis_FR);
	// set_motor(chassis.wheel_current[FL], chassis_FL);
	// set_motor(chassis.wheel_current[BL], chassis_BL);
	// set_motor(chassis.wheel_current[BR], chassis_BR);

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
int refresh_interval = 30;//更新次数，运行十次更新一次
int smaller_than_2_count = 0;
float valve=0.0f;
float random_anti_vision_r_s(float min, float max)
{
	static int run_count =100 ;
	run_count++;
        if (run_count >refresh_interval &&smaller_than_2_count<2) 
			{
				valve = generate_random_float(min, max);//下限和上限

				run_count = 0;
			if (valve < 2.0f) 
				{
					smaller_than_2_count++;
				}
				return  valve;
			}
		else if(run_count >refresh_interval &&smaller_than_2_count>=2)
		{
			smaller_than_2_count=0;
			return 3.5f;
		}
		else 
			return valve;
}
	
float generate_random_float(float min, float max) 
{
    return min + ((float)rand() / RAND_MAX) * (max - min);
}
