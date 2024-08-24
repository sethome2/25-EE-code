/**
 * @file NUC_communication.c
 * @brief 
 * @author sethome (you@domain.com)
 * @version 0.1
 * @date 2022-11-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "NUC_communication.h"
#include "IMU_updata.h"
#include "Global_status.h"
#include "USB_VirCom.h"
#include "Stm32_time.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "gimbal.h"
STM32_data_t toNUC;
unsigned char data[128];

uint32_t ready_time =0;
float auto_yaw,auto_pitch,visual_yaw,trans_delay_data,raw_vision_data;
bool vision_update_flag;
float from_input;
int err_time =50;

int decodeSTM32(STM32_data_t *target, unsigned char rx_buff[], unsigned int len)
{
//    if (len != sizeof(STM32_data_t))
//        return -1;

    memcpy(target, rx_buff, sizeof(STM32_data_t));
    return 0;
}
int encodeSTM32(STM32_data_t *target, unsigned char tx_buff[], unsigned int len)
{
//    if (len < sizeof(STM32_data_t))
//        return -1;

    memcpy(tx_buff, target, sizeof(STM32_data_t));
    return 0;
}

int decodeNUC(NUC_data_t *target, unsigned char rx_buff[], unsigned int len)
{
//    if (len != sizeof(NUC_data_t))
//        return -1;

    memcpy(target, rx_buff, sizeof(NUC_data_t));
    return 0;
}
int encodeNUC(NUC_data_t *target, unsigned char tx_buff[], unsigned int len)
{
//    if (len < sizeof(NUC_data_t))
//        return -1;

    memcpy(tx_buff, target, sizeof(NUC_data_t));
    return 0;
}
void board_to_nuc(void)//放在定时中断的任务，为了保证固定频率去发送
{
	toNUC.header = (unsigned)'h';
	toNUC.Q0 = IMU_data.AHRS.q[0];//传递imu结算的四元数姿态
	toNUC.Q1 = IMU_data.AHRS.q[1];
	toNUC.Q2 = IMU_data.AHRS.q[2];
	toNUC.Q3 = IMU_data.AHRS.q[3];
	
	toNUC.yaw = IMU_data.AHRS.yaw_rad_cnt;
	toNUC.pitch = IMU_data.AHRS.pitch;


	toNUC.x_speed = 100.0f;//以下五个参数为
	toNUC.y_speed = 100.0f;
	toNUC.r_speed = 100.0f;

	toNUC.bullet = 100;
	toNUC.distance = 100;

	toNUC.robot_speed_mps = 26.0f;
	
	if(Global.input.anti_stauts==1)//反陀螺
		toNUC.mode = 1;
	else
		toNUC.mode = 0;			
	
	if(get_robot_id() >= 100)
		toNUC.enemy_color = 1;//敌方红色
	else
		toNUC.enemy_color = 0;
	if(toNUC.enemy_color)//此时读取红色敌人的血量数据
	{
		toNUC.enemy_blood_[0] = game_robot_HP.red_7_robot_HP;//哨兵
		toNUC.enemy_blood_[1] = game_robot_HP.red_1_robot_HP;
		toNUC.enemy_blood_[2] = game_robot_HP.red_2_robot_HP;
		toNUC.enemy_blood_[3] = game_robot_HP.red_3_robot_HP;
		toNUC.enemy_blood_[4] = game_robot_HP.red_4_robot_HP;
		toNUC.enemy_blood_[5] = game_robot_HP.red_5_robot_HP;
	}
	else
	{
		toNUC.enemy_blood_[0] = game_robot_HP.blue_7_robot_HP;//哨兵
		toNUC.enemy_blood_[1] = game_robot_HP.blue_1_robot_HP;
		toNUC.enemy_blood_[2] = game_robot_HP.blue_2_robot_HP;
		toNUC.enemy_blood_[3] = game_robot_HP.blue_3_robot_HP;
		toNUC.enemy_blood_[4] = game_robot_HP.blue_4_robot_HP;
		toNUC.enemy_blood_[5] = game_robot_HP.blue_5_robot_HP;
	
	}
	toNUC.time_data =Get_sys_time_ms();//记录此时的时间戳
	encodeSTM32(&toNUC, data, sizeof(STM32_data_t));
	VirCom_send(data, sizeof(STM32_data_t));
}

/*自瞄任务，不要问为什么不妨到单独文件，问就是试过*/
void auto_ctrl(void)
{
		float record_now = gimbal.yaw.set;
	static uint8_t lost_count;
		/*******更新数据*******/
		if(fromNUC.yaw!=0)
		{
			lost_count  =0;
			trans_delay_data =fromNUC.yaw*ANGLE_TO_RAD -(get_history_data(YAW_DATA_HIS,err_time)-IMU_data.AHRS.yaw_rad_cnt);
			visual_yaw =  AutoAim_Algorithm(&yaw_auto_kf,trans_delay_data);
			//visual_yaw =trans_delay_data;
			data_kalman_process();
		}
		else
		{
			lost_count++;
		}
		/*******自瞄操控*******/
		if (vision_update_flag ==1&& lost_count<15 )
		{
//			raw_vision_data = IMU_data.AHRS.yaw_rad_cnt-visual_yaw;
			//auto_yaw = auto_aim_judgement(raw_vision_data);//包括多种模式判断，返回对应的数值
			auto_yaw = IMU_data.AHRS.yaw_rad_cnt-visual_yaw;
		}
		else
			auto_yaw = record_now;	

		if (fromNUC.pitch != 0.0f)
		{
			auto_pitch = IMU_data.AHRS.pitch - (fromNUC.pitch*ANGLE_TO_RAD);
		}
		else if(fromNUC.pitch == 0.0f)	
			auto_pitch = IMU_data.AHRS.pitch;
}

void data_kalman_process(void)
{
	ready_time++;
	if(ready_time >30)
	{
		vision_update_flag =1;
	}
	else
		vision_update_flag =0;
}
float  auto_aim_judgement(float input)
{
//	if(fabs(fromNUC.yaw)<1.5f&&fromNUC.shoot!=0)
//	{
//		from_input =input;
//	}
//	else 
//	{
//	from_input =input;
//	}

	return from_input;

}
//
bool update_sensor_data(float new_data) //采样30ms，平滑数据防止突变
{
	ready_time++;
//	i =medianFilter(DATA_SIZE, new_data);	
	if(ready_time >30)
	{
		return true;
	}
	else
		return false;
}
void vision_reset(void)
{
	ready_time =0;
	
}


//end of file
