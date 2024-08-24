/**
 * @file shoot.c
 * @author sethome
 * @brief 发射模块
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "shoot.h"
#include "stdio.h"
#include "pid.h"
#include "Stm32_time.h"

#include "global_status.h"
#include "CAN_receive&send.h"
#include "referee_handle_pack.h"
#include "NUC_communication.h"


pid_t trigger_speed_pid;
pid_t trigger_location_pid;

pid_t shoot1_speed_pid;
pid_t shoot2_speed_pid;

shoot_t shoot;

uint16_t booster_cnt = 0;
uint8_t  booster_status = 0;
uint8_t  trigger_cnt_auto;

//初始化
void shoot_init()
{
#ifdef USE_3508_AS_SHOOT_MOTOR
	pid_set(&shoot1_speed_pid, 25, 0, 0, 3500, 0);
	pid_set(&shoot2_speed_pid, 25, 0, 0, 3500, 0);	
	
//	pid_set(&shoot1_speed_pid, 15, 0, 4, 5500, 0);
//	pid_set(&shoot2_speed_pid, 15, 0, 4, 5500, 0);

#endif

	pid_set(&trigger_speed_pid, 30, 0, 0, 16000, 300);
	pid_set(&trigger_location_pid, 140, 0, 0, 10000, 100);
//改用双环控制后，如果想让拨弹速度变大，那么位置环的p数值一定不能小，而且建议有弹链的情况下p稍微超调一点，可以起到预压紧固的效果
//	shoot.trigger_status =  SPEEDS;
	shoot.trigger_status = LOCATIONS;//默认双环控制
	shoot.speed_level = SHOOT_STOP;
	shoot.trigger_location.set = 5;//初始上电检测的位置不知道为什么会变成5左右，加个固定补偿
	shoot.trigger_location.now = 5;
}

//更新拨弹电机数据
void shoot_update()
{
#ifdef USE_3508_AS_SHOOT_MOTOR
	//如果使用3508作为摩擦轮电机的话
	decode_as_3508(SHOOT_MOTOR1);
	decode_as_3508(SHOOT_MOTOR2);
	shoot.shoot_speed[0] = get_motor_data(SHOOT_MOTOR1).speed_rpm;
	shoot.shoot_speed[1] = get_motor_data(SHOOT_MOTOR2).speed_rpm;
#endif

	decode_as_2006(TRIGGER_MOTOR);
	shoot.trigger_location.now = get_motor_data(TRIGGER_MOTOR).angle_cnt;
	shoot.trigger_speed = get_motor_data(TRIGGER_MOTOR).speed_rpm;
	
	shoot.trigger_given_current = get_motor_data(TRIGGER_MOTOR).given_current;
}

void shoot_set_shoot_Motor_speed(float speed)
{
#ifdef USE_3508_AS_SHOOT_MOTOR
	set_motor(pid_cal(&shoot1_speed_pid, get_motor_data(SHOOT_MOTOR1).speed_rpm, speed), SHOOT_MOTOR1);
	set_motor(pid_cal(&shoot2_speed_pid, get_motor_data(SHOOT_MOTOR2).speed_rpm, -speed), SHOOT_MOTOR2);
#else
	//适配其他拨弹电机
	PWM_snaill_set(PIN_2, (uint16_t)speed);
	PWM_snaill_set(PIN_3, (uint16_t)speed);
#endif
}

void shoot_pid_cal()
{
	static enum trigger_status_e last_status = LOCATIONS;

	//摩擦轮设定
	decode_as_3508(SHOOT_MOTOR1);
	shoot_set_shoot_Motor_speed((float)shoot.speed_level);

	decode_as_2006(TRIGGER_MOTOR);

	//拨弹电机设定
	if (shoot.trigger_status == LOCATIONS) //位置控制
	{
		if (last_status == SPEEDS)
		{
			shoot.trigger_location.set += shoot.trigger_location.now;
		}
		shoot.trigger_speed = pid_cal(&trigger_location_pid, shoot.trigger_location.now, shoot.trigger_location.set);
	}
	else if (shoot.trigger_status == SPEEDS) //速度控制
	{
		shoot.trigger_speed = shoot.set_trigger_speed;
	}

	//速度环
	set_motor(pid_cal(&trigger_speed_pid, get_motor_data(TRIGGER_MOTOR).speed_rpm, shoot.trigger_speed), TRIGGER_MOTOR);
	last_status = shoot.trigger_status;

}

void shoot_speed_limit()
{
	if (Global.input.shooter_status == 0)
	{
		shoot.speed_level = SHOOT_STOP;
		booster_status = 0;
		booster_cnt = 0;
		trigger_anti_kill_and_set_speed(0);
	}
	else if (Global.input.shooter_status == 1)
	{
		booster_cnt++;
		if (booster_cnt >= 500)
		{
			booster_status = 1;
		}

		if (booster_status == 1)
		{
			shoot.speed_level = SHOOT_30;
		}
		else if (booster_status == 0)
			shoot.speed_level = SHOOT_BEGIN;
	}
	else
	{
	}
}

void trigger_anti_kill_and_set_speed(float set)
{

	if (shoot.trigger_given_current > 10000)//达到卡弹电流阈值
		Global.input.trigger_kill_cnt = 18;//延时作用

	if (Global.input.trigger_kill_cnt > 5)
	{
		Global.input.trigger_kill_cnt--;
		Global.input.trigger_kill = 1;//卡弹标志位置一
	}
	else if (Global.input.trigger_kill_cnt == 5)//总共执行13次反转
		Global.input.trigger_kill = 0;
	
	if (Global.input.trigger_kill == 1)
	{
		shoot.set_trigger_speed = -10000;
	}
	
	else//下面才是正常发弹逻辑
	{

		if (REFEREE_DATA.Barrel_Heat < (REFEREE_DATA.Heat_Limit - 20))	
		{
			if (REFEREE_DATA.Barrel_Heat < (REFEREE_DATA.Heat_Limit - 150))//290-150 = 140 
				shoot.set_trigger_speed = set;//正常传入
			else if ((REFEREE_DATA.Barrel_Heat > (REFEREE_DATA.Heat_Limit - 150))&&(REFEREE_DATA.Barrel_Heat < (REFEREE_DATA.Heat_Limit - 100)))  //290- 100 = 190
				shoot.set_trigger_speed = set / 1.5f;
			else if ((REFEREE_DATA.Barrel_Heat > (REFEREE_DATA.Heat_Limit - 100))&&(REFEREE_DATA.Barrel_Heat < (REFEREE_DATA.Heat_Limit - 50)))  //290-50 = 240 
				shoot.set_trigger_speed = set / 3;
			else
				shoot.set_trigger_speed = set / 4;		
		}
		else
			shoot.set_trigger_speed = 0;
	}
}

void shoot_trigger_online()
{
	if (Global.input.shooter_status != 0)//开始发弹
	{
		switch (Global.input.shoot_num)//单发连发
		{
		case 0://默认单发
				shoot_Bullets(1);//单发
			break;
		case 1://连发
			if(fromNUC.shoot == 2)//自瞄控制发弹
//				trigger_anti_kill_and_set_speed(7000.0f);不再使用单速度环控制，改用双环
				shoot_Bullets(114514);
			else//操作手手动接管发弹
				shoot_Bullets(114514);
			break;

		default:
			trigger_anti_kill_and_set_speed(0);
			break;
		}
	}
	else if (Global.input.shooter_status == 0)
	{
		trigger_anti_kill_and_set_speed(0);
	}
}


void triger_clear_cnt()
{
	clear_motor_cnt(TRIGGER_MOTOR);
	shoot.trigger_location.now=0;
	shoot.trigger_location.set=0;
}

void shoot_set_trigger_location(int n)
{
	static float last_time = 0;
	if(n == 1)
	{
		if (Get_sys_time_ms() - last_time > 200)//单发判断
		//只有按下>200ms才会打一发，因为按下的时间可能会执行很多次，所以要延长间隔
		{
			shoot.trigger_location.set +=  A_BULLET_ANGEL;//200ms就打一发
	//		triger_clear_cnt();

		}	
	}
	else if(n ==114514)
	{
		if (Get_sys_time_ms() - last_time > 50)//50ms就打一发
		{
			shoot.trigger_location.set +=  A_BULLET_ANGEL;

		}
	}
	else//给反陀螺预留接口，初步意向一次4连发
	{
		if (Get_sys_time_ms() - last_time > 150)//150ms打4发
		{
			shoot.trigger_location.set += 4* A_BULLET_ANGEL;

		}
				
	}
	last_time = Get_sys_time_ms();
}
void shoot_set_trigger_speed(int set)
{
	shoot.trigger_status = SPEEDS;
	shoot.set_trigger_speed = set;
	triger_clear_cnt();
}

int shoot_Bullets(int n)
{
	shoot.trigger_status = LOCATIONS;
	shoot_set_trigger_location(n);
	return n;
}


// end of file
