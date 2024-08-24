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
#include "small_tools.h"
#include "Global_status.h"

#include "DBUS_remote_control.h"
#include "USB_VirCom.h"

struct gimbal_status gimbal;

pid_t pitch_speed_pid;
pid_t pitch_location_pid;


pid_t yaw_speed_pid;
pid_t yaw_location_pid;

pid_t yaw_speed_pid_tank;
pid_t yaw_location_pid_tank;

pid_t yaw_speed_pid_flsp;
pid_t yaw_location_pid_flsp;

//云台初始化
void gimbal_init()
{

//  pid_set(&pitch_speed_pid, 1000.0f, 0.0, 100.0f, 18000.0f, 2000.0f);//适用于陀螺仪控制  
//	pid_set(&pitch_location_pid, 20.0f, 0.0f,0.0f, 100.0f, 0.0f);			

//  pid_set(&pitch_speed_pid, 350.0f, 0.0, 8500.0f, 25000.0f, 2000.0f);//编码器   
//	pid_set(&pitch_location_pid, 80.0f, 0.0f,10.0f, 1200.0f, 0.0f);	
  pid_set(&pitch_speed_pid, 500.0f, 0.0, 9000.0f, 25000.0f, 2000.0f);//编码器   
	pid_set(&pitch_location_pid, 80.0f, 0.0f,900.0f, 1200.0f, 0.0f);
//	
//  pid_set(&yaw_speed_pid, 200, 1.0f, 1.0, 25000.0f, 3600.0f); //最大数值限幅    
//  pid_set(&yaw_location_pid, 5, 0.01f, 250, 200.0f, 0.0f);       // 3600	
	pid_set(&yaw_speed_pid, 320, 1.0f, 1.0, 25000.0f, 3600.0f); //最大数值限幅    
  pid_set(&yaw_location_pid, 5, 0.01f, 320, 200.0f, 0.0f);  
//	pid_set(&yaw_speed_pid, 210, 1.0f, 1.0, 25000.0f, 3600.0f); //最大数值限幅    
//  pid_set(&yaw_location_pid, 5, 0, 250, 200.0f, 0.0f);       // 3600	
	
	pid_set(&yaw_speed_pid_flsp, 20, 0.15f, 60.0f, 30000.0f, 100.0f); //最大数值限幅
  pid_set(&yaw_location_pid_flsp, 100, 0.1, 80.0f, 100.0f, 0.0f);       // 3600	

	

  gimbal.pitch.now = 0;
  gimbal.pitch.set = 0;
  gimbal.pitch.offset = 0;
  gimbal.pitch.stable = 0;
  gimbal.set_pitch_speed = 0;

  gimbal.yaw.now = 0;
  gimbal.yaw.set = 0;
  gimbal.yaw.offset = 0;
  gimbal.yaw.stable = 0;
  gimbal.set_yaw_speed = 0;
	
  gimbal.yaw_status = gimbal.pitch_status = LOCATION; // 默认为位置控制模式
}


void gimbal_set_offset(float pitch, float yaw)
{
  gimbal.pitch.offset = pitch;
  gimbal.yaw.offset = yaw;
}

void gimbal_updata()
{
  // 从陀螺仪或电机获取数据
	
	
  // yaw轴更新
  decode_as_6020(YAW_MOTOR);
	//decode_as_6020_test(YAW_MOTOR);
  // 电机
	if(Global.mode==FLSP)
	 	gimbal.yaw_speed=get_motor_data(YAW_MOTOR).speed_rpm;
	 else
		gimbal.yaw_speed = (get_motor_data(YAW_MOTOR).ecd - get_motor_data(YAW_MOTOR).last_ecd) / ECD_MAX;
	
//  // 陀螺仪
		if(Global.mode==FLSP)
    {
			gimbal.yaw.now = get_motor_data(YAW_MOTOR).angle - gimbal.yaw.offset;
			gimbal.yaw.set = rad2degree(IMU_data.AHRS.yaw_rad_cnt);
    }
		else
			gimbal.yaw.now = rad2degree(IMU_data.AHRS.yaw_rad_cnt);


  // pitch轴更新
  decode_as_6020(PITCH_MOTOR);
		
  // 电机
//  gimbal.pitch.now = degree2rad(get_motor_data(PITCH_MOTOR).angle_cnt - gimbal.pitch.offset); // 角度转为弧度
//  gimbal.pitch_speed = (get_motor_data(YAW_MOTOR).ecd - get_motor_data(YAW_MOTOR).last_ecd) / ECD_MAX;
			
  // 陀螺仪
	gimbal.pitch.now = IMU_data.AHRS.pitch; // 角度转为弧度
  gimbal.pitch_speed = -IMU_data.gyro[0]; //获取y转轴（对应pitch轴）角速度，做闭环用。
		
		
		
//  // 电机
//  gimbal.pitch.now = degree2rad(get_motor_data(PITCH_MOTOR).angle_cnt - gimbal.pitch.offset); // 角度转为弧度
////  gimbal.pitch_speed = (get_motor_data(YAW_MOTOR).ecd - get_motor_data(YAW_MOTOR).last_ecd) / ECD_MAX;
//		
//  // 陀螺仪
////	gimbal.pitch.now = IMU_data.AHRS.pitch; // 角度转为弧度
//  gimbal.pitch_speed = -IMU_data.gyro[0]; //获取y转轴（对应pitch轴）角速度，做闭环用。		
		
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

void gimbal_set_pitch(float pitch)
{
  gimbal.pitch_status = LOCATION; // 以位置模式控制

  if (fabs(pitch) < 20)
    gimbal.pitch.set = pitch;
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
  // 位置环 （位置控制模式下）
  if (gimbal.yaw_status == LOCATION)
  {
	 if(Global.mode==FLSP)
			gimbal.set_yaw_speed = pid_cal(&yaw_location_pid_flsp, gimbal.yaw.now, 0.0f);
		else
			gimbal.set_yaw_speed = pid_cal(&yaw_location_pid, gimbal.yaw.now, gimbal.yaw.set);
  }
  else
    gimbal.set_yaw_speed = gimbal.yaw_speed;


  if (gimbal.pitch_status == LOCATION)
  {
    gimbal.set_pitch_speed = pid_cal(&pitch_location_pid, gimbal.pitch.now, gimbal.pitch.set);
  }
  else
    gimbal.set_pitch_speed=gimbal.pitch_speed;

    
  // 速度环
	  if(Global.mode==FLSP)
		set_motor(pid_cal(&yaw_speed_pid_flsp, gimbal.yaw_speed, gimbal.set_yaw_speed), YAW_MOTOR);
	else	
		set_motor(pid_cal(&yaw_speed_pid, gimbal.yaw_speed, gimbal.set_yaw_speed), YAW_MOTOR);
	
  set_motor(pid_cal(&pitch_speed_pid, gimbal.pitch_speed, gimbal.set_pitch_speed), PITCH_MOTOR);
}

void gimbal_clear_cnt(void)
{
  clear_motor_cnt(YAW_MOTOR);
}
