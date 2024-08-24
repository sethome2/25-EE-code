/**
 * @file gimbal.c
 * @author sethome
 * @brief
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "gimbal.h"
#include "IMU_updata.h"
#include "CAN_receive&send.h"
#include "Global_status.h"
#include "control_setting.h"
#include "Stm32_time.h"
#include "stm32f4xx_hal.h"

struct gimbal_status gimbal;
/* 编码器控制pid*/
pid_t pitch_location_speed_pid;
pid_t pitch_location_pid;

pid_t yaw_location_speed_pid;
pid_t yaw_location_pid;
/* 陀螺仪控制pid*/
pid_t pitch_absolute_speed_pid;
pid_t pitch_absolute_pid;

pid_t yaw_absolute_speed_pid;
pid_t yaw_absolute_pid;
/* 自瞄控制pid*/
pid_t pitch_auto_speed_pid;
pid_t pitch_auto_pid;

pid_t yaw_auto_speed_pid;
pid_t yaw_auto_pid;
/* 无头控制pid，并未使用*/
pid_t yaw_location_speed_pid_tank;
pid_t yaw_location_pid_tank;

float pitch_test_link =  8.0f;//自瞄角度系数，目前使用二分法经验得来
float yaw_test_link = 12.0f;
/*前馈控制参数*/
float KF=0.0f;
//目标值
float Target = 0.0f;
float Pre_Target = 0.0f;
//当前值
float Now = 0.0f;
float set_motor_time;
float forward_out;
//云台初始化
void gimbal_init()
{
/* 编码器控制pid*/
	pid_set(&pitch_location_speed_pid, 0.0f, 0.0, 0.0f, 30000.0f, 20000.0f); 
	pid_set(&pitch_location_pid, 0.0f, 0.0f,0.0f, 8000.0f, 0.01f);

	pid_set(&yaw_location_speed_pid, 0, 0, 0.0, 25000.0f, 3600.0f); 
	pid_set(&yaw_location_pid, 0, 0, 0, 8000.0f, 0.0f);     
/* 陀螺仪控制pid*/
	pid_set(&pitch_absolute_pid, 16.0f, 0.0, 0.0f, 30000.0f, 20000.0f); 
	pid_set(&pitch_absolute_speed_pid, 10000.0f, 0.0f,0.0f, 20000.0f, 0.01f);
	
	pid_set(&yaw_absolute_pid, 20, 0, 0.0, 25000.0f, 3600.0f); 
	pid_set(&yaw_absolute_speed_pid, 13000, 0, 0000, 25000.0f, 0.0f);  
	
//	pid_set(&pitch_absolute_pid, 0.0f, 0.0, 0.0f, 30000.0f, 20000.0f); 
//	pid_set(&pitch_absolute_speed_pid, 0.0f, 0.0f,0.0f, 20000.0f, 0.01f);
	
//	pid_set(&yaw_absolute_pid, 0, 0, 0.0, 25000.0f, 3600.0f); 
//	pid_set(&yaw_absolute_speed_pid, 0, 0, 0000, 25000.0f, 0.0f);  
/* 自瞄控制pid*/	
	pid_set(&pitch_auto_pid, 15.0f, 0.0, 0.0f, 30000.0f, 20000.0f); //当前pitch有过大阻力，无法使用
	pid_set(&pitch_auto_speed_pid, 10000.0f, 0.0f,0.0f, 20000.0f, 0.01f);
	
	pid_set(&yaw_auto_pid, 30, 0, 0.0, 25000.0f, 3600.0f); 
	pid_set(&yaw_auto_speed_pid, 14000, 0, 0000, 25000.0f, 0.0f);  
/*自瞄控制pid*/	
	
/* 其他参数，初始补偿之类的*/
	  gimbal.pitch.now = 0;
	  gimbal.pitch.set = 0;
	//  gimbal.pitch.offset = 0;
	  gimbal.pitch.stable = 0;
	  gimbal.set_pitch_speed = 0;

	  gimbal.yaw.now = 0;
	  gimbal.yaw.set = 0;
	//  gimbal.yaw.offset = 0;
	  gimbal.yaw.stable = 0;
	  gimbal.set_yaw_speed = 0;
/* 云台控制模式*/
	  gimbal.yaw_status  = ABSOLUTE; // 默认为绝对位置位置控制模式（陀螺仪）
	  gimbal.pitch_status = ABSOLUTE;
}

void gimbal_set_offset(float ab_pitch, float ab_yaw,float lo_pitch,float lo_yaw)
{
	gimbal.pitch.absoulte_offset = ab_pitch;
	gimbal.yaw.absoulte_offset = ab_yaw;
	gimbal.pitch.location_offset = lo_pitch;
	gimbal.yaw.location_offset = lo_yaw;
}

void gimbal_updata()
{
	decode_as_6020(YAW_MOTOR);
	decode_as_6020(PITCH_MOTOR);
	//获取编码器数值用于绝对控制
	if (gimbal.yaw_status == LOCATION)//没做
	{
		
	}
	else if(gimbal.yaw_status == ABSOLUTE )
	{
		gimbal.yaw.now = IMU_data.AHRS.yaw_rad_cnt;
		gimbal.yaw_speed = cos(IMU_data.AHRS.pitch)*IMU_data.gyro[2]
							-sin(IMU_data.AHRS.pitch)*IMU_data.gyro[0];//多角度融合
		
	}
	if (gimbal.pitch_status == LOCATION)//没做
	{
		gimbal.pitch.now = degree2rad(get_motor_data(PITCH_MOTOR).angle - gimbal.pitch.location_offset); 
		gimbal.pitch_speed = IMU_data.gyro[1]; //获取y转轴（对应pitch轴）角速度，做闭环用。
		
	}
	else if(gimbal.pitch_status == ABSOLUTE )
	{

		gimbal.pitch.now =IMU_data.AHRS.pitch- degree2rad(gimbal.pitch.absoulte_offset);//不要乱改哈，后续会加上绝对/相对的补偿
		gimbal.pitch_speed = IMU_data.gyro[1];
	}
}

// 设定角度
void gimbal_set(float pitch, float yaw)
{
  gimbal.yaw_status = gimbal.pitch_status = LOCATION; // 以位置模式控制

  // 范围限定 防止超出机械限位
  if (fabs(pitch) < 15)
    gimbal.pitch.set = pitch;

  gimbal.yaw.set = yaw;
}

void gimbal_set_pitch(float pitch,float up_angle,float down_angle)
{

  if (rad2degree(pitch) >up_angle)
  {
	gimbal.pitch.set = degree2rad(up_angle);
  }
  else if(rad2degree(pitch) < -down_angle)
  {
	gimbal.pitch.set = degree2rad(-down_angle) ;
  }
  
}

// 设定速度
void gimbal_set_speed(float pitch, float yaw)
{
  gimbal.yaw_status = gimbal.pitch_status = SPEED; // 以速度模式控制

  gimbal.set_pitch_speed = pitch;
  gimbal.set_yaw_speed = yaw;
}
void gimbal_set_yaw_speed(float yaw)
{
  gimbal.yaw_status = SPEED; // 以速度模式控制

  gimbal.set_yaw_speed = yaw;
}

void gimbal_pid_cal()
{
	gimbal_set_pitch(gimbal.pitch.set,16.0f,18.0f);//限位
  // 位置环 （位置控制模式下）
	/*yaw行为控制*/
  if (gimbal.yaw_status == LOCATION)//位置环
  {
    gimbal.set_yaw_speed = pid_cal(&yaw_location_pid, gimbal.yaw.now, gimbal.yaw.set);
	set_motor(pid_cal(&yaw_location_speed_pid, gimbal.yaw_speed, gimbal.set_yaw_speed), YAW_MOTOR);
  }
  else if(gimbal.yaw_status ==ABSOLUTE &&Global.input.vision_status == 0)//角度换(手操）
  {
	  
	gimbal.set_yaw_speed = pid_cal(&yaw_absolute_pid, gimbal.yaw.now, gimbal.yaw.set);
	set_motor(pid_cal(&yaw_absolute_speed_pid, gimbal.yaw_speed, gimbal.set_yaw_speed), YAW_MOTOR);
  }
  else if(Global.input.vision_status == 1)//（自瞄）
  {
	gimbal.yaw.set = auto_yaw;
	gimbal.set_yaw_speed = pid_cal(&yaw_auto_pid, gimbal.yaw.now, gimbal.yaw.set);
	set_motor(pid_cal(&yaw_auto_speed_pid, gimbal.yaw_speed, gimbal.set_yaw_speed), YAW_MOTOR);
  }
  else
    gimbal.set_yaw_speed = gimbal.yaw_speed;
  
	/*pitch行为控制*/
  if (gimbal.pitch_status == LOCATION)
  {
    gimbal.set_pitch_speed = pid_cal(&pitch_location_pid, gimbal.pitch.now, gimbal.pitch.set);
	set_motor(pid_cal(&pitch_location_speed_pid, gimbal.pitch_speed, gimbal.set_pitch_speed), PITCH_MOTOR);  
  }
  else if(gimbal.pitch_status ==ABSOLUTE&&Global.input.vision_status == 0)
  {
	gimbal.set_pitch_speed = pid_cal(&pitch_absolute_pid,gimbal.pitch.now, gimbal.pitch.set);
	set_motor(pid_cal(&pitch_absolute_speed_pid, gimbal.pitch_speed, gimbal.set_pitch_speed), PITCH_MOTOR);
  }
 else if(Global.input.vision_status == 1)
 {
	gimbal.pitch.set = auto_pitch;
	gimbal.set_pitch_speed = pid_cal(&pitch_auto_pid,gimbal.pitch.now, gimbal.pitch.set);
	set_motor(pid_cal(&pitch_auto_speed_pid, gimbal.pitch_speed, gimbal.set_pitch_speed), PITCH_MOTOR);
 }
  else
    gimbal.set_pitch_speed = gimbal.pitch_speed;
	set_motor_time = Get_sys_time_ms();
}

void gimbal_clear_cnt(void)
{
  clear_motor_cnt(YAW_MOTOR);
}
