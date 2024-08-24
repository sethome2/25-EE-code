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
#include "math.h"
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
uint8_t booster_status = 0;
uint8_t trigger_cnt_auto;

// 初始化
void shoot_init()
{
#ifdef USE_3508_AS_SHOOT_MOTOR
	// 摩擦轮电机
	pid_set(&shoot1_speed_pid, 27, 0, 0, 3500, 0);
	pid_set(&shoot2_speed_pid, 27, 0, 0, 3500, 0);

#endif
	// 拨弹电机
	pid_set(&trigger_speed_pid, 5, 0, 0, 16000, 300);
	pid_set(&trigger_location_pid, 140, 0, 0, 10000, 100);

	// 改用双环控制后，如果想让拨弹速度变大，那么位置环的p数值一定不能小，而且建议有弹链的情况下p稍微超调一点，可以起到预压紧固的效果
	shoot.trigger_status = SPEEDS;
	//	shoot.trigger_status = LOCATIONS;//默认双环控制
	shoot.speed_level = SHOOT_STOP;
	shoot.trigger_location.set = 0; // 初始上电检测的位置不知道为什么会变成5左右，加个固定补偿
	shoot.trigger_location.now = 0;
}

// 更新拨弹电机数据
void shoot_update()
{
#ifdef USE_3508_AS_SHOOT_MOTOR
	// 如果使用3508作为摩擦轮电机的话
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
	// 适配其他拨弹电机
	PWM_snaill_set(PIN_2, (uint16_t)speed);
	PWM_snaill_set(PIN_3, (uint16_t)speed);
#endif
}

void shoot_pid_cal(void) // 融合了多个功能，射速切换，热量控制，自瞄非自瞄pid不同
{
	float set;
	if (Global.input.shoot_num == 1 && Global.input.shoot_fire && Global.input.shooter_status) // 高射速模式&&开火键按下&&摩擦轮开启
		set = 6000.0f;
	else if (Global.input.shoot_num == 0 && Global.input.shoot_fire && Global.input.shooter_status) // 低射速模式&&开火键按下&&摩擦轮开启
		set = 3000.0f;
	else
		set = 0.0f;
	// 摩擦轮设定
	decode_as_3508(SHOOT_MOTOR1);
	shoot_set_shoot_Motor_speed((float)shoot.speed_level);

	decode_as_2006(TRIGGER_MOTOR);

	// 拨弹电机设定

	if (shoot.trigger_status == LOCATIONS) // 位置控制
	{
		pid_set(&trigger_speed_pid, 10, 0, 0, 16000, 300);
		shoot.trigger_speed = pid_cal(&trigger_location_pid, shoot.trigger_location.now, shoot.trigger_location.set);
	}
	else if (shoot.trigger_status == SPEEDS) // 速度控制
	{
		if (Global.input.shoot_fire == 1 || fromNUC.shoot == 2)
		{
			pid_set(&trigger_speed_pid, 40, 0, 0, 16000, 300);
		}
		else
			pid_set(&trigger_speed_pid, 5, 0, 0, 16000, 300);
	}
	// 回学校之后用switch case 重写，现在没车不敢试
	if (Global.input.shoot_fire)
	{
		if (Global.input.vision_status == 0) // 非自瞄
		{
			if (power_heat_data.shooter_17mm_1_barrel_heat < (robot_status.shooter_barrel_heat_limit - 20)) // 180
			{
				if (power_heat_data.shooter_17mm_1_barrel_heat < (robot_status.shooter_barrel_heat_limit - 120)) // 80
					RC_trigger_anti_kill_and_set_speed(set / 1.5f);
				else if ((power_heat_data.shooter_17mm_1_barrel_heat > (robot_status.shooter_barrel_heat_limit - 120)) && (power_heat_data.shooter_17mm_1_barrel_heat < (robot_status.shooter_barrel_heat_limit - 90))) // 80--110
					RC_trigger_anti_kill_and_set_speed(set / 3.0);
				else
					RC_trigger_anti_kill_and_set_speed(set / 4.0f); // 150-180
			}
			else
				shoot.set_trigger_speed = 0.0f;
		}
		else if (Global.input.vision_status == 1) // 自瞄
		{
			if (power_heat_data.shooter_17mm_1_barrel_heat < (robot_status.shooter_barrel_heat_limit - 20))
			{
				if (power_heat_data.shooter_17mm_1_barrel_heat < (robot_status.shooter_barrel_heat_limit - 140)) // 60
					RC_trigger_anti_kill_and_set_speed(set);
				else if ((power_heat_data.shooter_17mm_1_barrel_heat > (robot_status.shooter_barrel_heat_limit - 140)) && (power_heat_data.shooter_17mm_1_barrel_heat < (robot_status.shooter_barrel_heat_limit - 70))) // 60--130
					RC_trigger_anti_kill_and_set_speed(set/1.5f);
				else
					RC_trigger_anti_kill_and_set_speed(set/4.0f);
			}
			else
				shoot.set_trigger_speed = 0.0f;
		}
	}
	else // 遥控器控制
		RC_trigger_anti_kill_and_set_speed(Global.input.shoot_RC*10);
	// 速度环
	if (shoot.set_trigger_speed || get_motor_data(TRIGGER_MOTOR).speed_rpm >= 5)
		set_motor(pid_cal(&trigger_speed_pid, get_motor_data(TRIGGER_MOTOR).speed_rpm, shoot.set_trigger_speed), TRIGGER_MOTOR);
	if (!shoot.set_trigger_speed && get_motor_data(TRIGGER_MOTOR).speed_rpm <= 5)
		set_motor(0, TRIGGER_MOTOR); // 防抖直接给拨弹盘电机给0
}

void shoot_speed_limit()
{
	if (Global.input.shooter_status == 0)
	{
		shoot.speed_level = SHOOT_STOP;
		booster_status = 0;
		booster_cnt = 0;
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
			shoot.speed_level = SHOOT_BEGIN; // 刚开始时让摩擦轮反转防止卡弹，不然就寄了
	}
}

void RC_trigger_anti_kill_and_set_speed(float set) // 给遥控器任务单独用的，用于裁判系统剑录时防止卡弹，现也用于键鼠操作下的堵转反转
{
	shoot.trigger_status = SPEEDS;			 // 速度环控制，进行模式切换
	if (shoot.trigger_given_current > 10000) // 达到卡弹电流阈值
		Global.input.trigger_kill_cnt = 18;	 // 延时作用

	if (Global.input.trigger_kill_cnt > 5)
	{
		Global.input.trigger_kill_cnt--;
		Global.input.trigger_kill = 1; // 卡弹标志位置一
	}
	else if (Global.input.trigger_kill_cnt == 5) // 总共执行13次反转
		Global.input.trigger_kill = 0;

	if (Global.input.trigger_kill == 1)
	{
		shoot.set_trigger_speed = -10000;
	}

	else // 下面才是正常发弹逻辑
	{
		shoot.set_trigger_speed = set; // 正常传入
	}
}



/*********************** 以下为未被使用的函数，后期测试后考虑删除**************************/ 
void PC_trigger_set_speed(float set) // 用于键鼠模式下发弹操作，由于键鼠模式的函数有单独的总防止卡逻辑所以在声明一个。
{
	shoot.trigger_status = SPEEDS;
	if (Global.input.vision_status == 0)
	{
		if (REFEREE_DATA.Barrel_Heat < (REFEREE_DATA.Heat_Limit - 20)) // 180
		{
			if (REFEREE_DATA.Barrel_Heat < (REFEREE_DATA.Heat_Limit - 120)) // 80
				shoot.set_trigger_speed = set / 1.5f;
			else if ((REFEREE_DATA.Barrel_Heat > (REFEREE_DATA.Heat_Limit - 120)) && (REFEREE_DATA.Barrel_Heat < (REFEREE_DATA.Heat_Limit - 90))) // 80--110
				shoot.set_trigger_speed = set / 3.0f;
			else
				shoot.set_trigger_speed = set / 4.0f; // 150-180
		}
		else
			shoot.set_trigger_speed = -0.01f;
	}
	else if (Global.input.vision_status == 1)
	{
		if (REFEREE_DATA.Barrel_Heat < (REFEREE_DATA.Heat_Limit - 20))
		{
			if (REFEREE_DATA.Barrel_Heat < (REFEREE_DATA.Heat_Limit - 140)) // 60
				shoot.set_trigger_speed = set;
			else if ((REFEREE_DATA.Barrel_Heat > (REFEREE_DATA.Heat_Limit - 140)) && (REFEREE_DATA.Barrel_Heat < (REFEREE_DATA.Heat_Limit - 70))) // 60--130
				shoot.set_trigger_speed = set / 1.5f;
			else
				shoot.set_trigger_speed = set / 4.0f;
		}
		else
			shoot.set_trigger_speed = -0.01f;
	}

	else
		shoot.set_trigger_speed = 0;
}

void shoot_trigger_online()
{											 // 卡弹的时候
	if (shoot.trigger_given_current > 10000) // 达到卡弹电流阈值
		Global.input.trigger_kill_cnt = 18;	 // 延时作用

	if (Global.input.trigger_kill_cnt > 5)
	{
		Global.input.trigger_kill_cnt--;
		Global.input.trigger_kill = 1; // 卡弹标志位置一
	}
	else if (Global.input.trigger_kill_cnt == 5) // 总共执行13次反转
		Global.input.trigger_kill = 0;

	if (Global.input.trigger_kill == 1)
	{
		shoot.set_trigger_speed = -10000;
	}
	else // 不卡弹的时候执行发射逻辑
	{
		if (Global.input.shooter_status != 0) // 开始发弹
		{
			switch (Global.input.shoot_num) // 单发连发
			{
			case 0:														   // 默认单发
				if (fromNUC.shoot == 2 && Global.input.vision_status == 1) // 右键+左键自瞄控制发弹
					PC_trigger_set_speed(3000.0f);
				else if (Global.input.vision_status == 0) // 无自瞄启动时手动发弹
					PC_trigger_set_speed(3000.0f);
				//						shoot_Bullets(1);//单发
				// shoot_Bullets(114514);
				break;
			case 1: // 连发//	if(fromNUC.shoot == 2&& Global.input.vision_status==1&&Global.input.anti_stauts==0)

				if (fromNUC.shoot == 2 && Global.input.vision_status == 1) // 右键+左键自瞄控制发弹
					PC_trigger_set_speed(6000.0f);
				else if (Global.input.vision_status == 0) // 无自瞄启动时手动发弹
					PC_trigger_set_speed(6000.0f);
				break;
			}
		}
		else
		{
			PC_trigger_set_speed(-0.01f);
		}
	}
}

void triger_clear_cnt()
{
	clear_motor_cnt(TRIGGER_MOTOR);
	shoot.trigger_location.now = 0;
	shoot.trigger_location.set = 0;
}

void shoot_set_trigger_location(int n)
{
	static float last_time = 0;
	if (REFEREE_DATA.Barrel_Heat < (REFEREE_DATA.Heat_Limit - 20)) // 在小于热量限制的时候正常发弹，若马上到达临界热量就停止发弹
	{
		if (n == 1)
		{
			if (Get_sys_time_ms() - last_time > 150) // 单发判断
			// 只有按下>200ms才会打一发，因为按下的时间可能会执行很多次，所以要延长间隔
			{
				shoot.trigger_location.set = shoot.trigger_location.now + A_BULLET_ANGEL; // 默认发两下算了
			}
		}
		last_time = Get_sys_time_ms();
	}
}
void shoot_set_trigger_speed(int set)
{
	shoot.trigger_status = SPEEDS;
	shoot.set_trigger_speed = set;
	triger_clear_cnt();
}
/*因为现在主要是以速度环为主要逻辑，只有单发时才使用位置环，
所以不用判断之前的是什么样，因为绝对是速度环控制
理由是目前机械卡弹还是会存在，如果使用位置环会导致卡弹回退后会疯狂发弹导致超热量死亡
，其实在卡弹时位置环set=now就可以避免，但是目前位置环表现不好，所以先采用速度环连发。
*/

int shoot_Bullets(int n)
{
	shoot.trigger_status = LOCATIONS;
	shoot_set_trigger_location(n);
	return n;
}
//end of file