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
#include "control_setting.h"
#include "NUC_communication.h"
#include "Global_status.h"

#include "IMU_updata.h"
#include "CAN_receive&send.h"

#include "Stm32_time.h"
#include "stm32f4xx_hal.h"

#include "math.h"
struct gimbal_status gimbal;
/* 编码器pid*/
pid_t pitch_location_speed_pid;
pid_t pitch_location_pid;

pid_t yaw_location_speed_pid;
pid_t yaw_location_pid;
/* 陀螺仪pid*/
pid_t pitch_absolute_speed_pid;
pid_t pitch_absolute_pid;

pid_t yaw_absolute_speed_pid;
pid_t yaw_absolute_pid;
/* 自瞄pid*/
pid_t pitch_auto_speed_pid;
pid_t pitch_auto_pid;

pid_t yaw_auto_speed_pid;
pid_t yaw_auto_pid;
/*前馈参数*/
float KF = 50.0f;
// 目标值
float Target = 0.0f;
float Pre_Target = 0.0f;
// 实际值
float Now = 0.0f;
float forward_out;
/*停在坡上参数*/
float slope;
/*飞坡固定pitch的相关参数*/
float pitch_fly_angle;

// 云台初始化
void gimbal_init()
{
	/*编码器pid*/
	pid_set(&pitch_location_speed_pid, 8000.0f, 0.0, 0.0f, 30000.0f, 20000.0f);
	pid_set(&pitch_location_pid, 20.0f, 0.0f, 0.0f, 8000.0f, 0.01f);

	pid_set(&yaw_location_speed_pid, 0, 0, 0.0, 25000.0f, 3600.0f);
	pid_set(&yaw_location_pid, 0, 0, 0, 8000.0f, 0.0f);
	/* 陀螺仪pid*/
	pid_set(&pitch_absolute_pid, 11.0f, 0.0, 0.0f, 30000.0f, 20000.0f);
	pid_set(&pitch_absolute_speed_pid, 10000.0f, 0.0f, 0.0f, 20000.0f, 0.01f);
	//  pid_set(&pitch_absolute_pid, 0.0f, 0.0, 0.0f, 0.0f, 0.0f);
	//	pid_set(&pitch_absolute_speed_pid, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);
	pid_set(&yaw_absolute_pid, 16.0f, 0, 0.0, 25000.0f, 3600.0f);
	pid_set(&yaw_absolute_speed_pid, 15000, 0, 0000, 25000.0f, 0.0f);
	//	pid_set(&yaw_absolute_pid, 0, 0, 0.0, 0.0f, 0.0f);
	//	pid_set(&yaw_absolute_speed_pid, 0, 0, 0, 0.0f, 0.0f);
	/* 自瞄pid*/
	pid_set(&pitch_auto_pid, 18.0f, 0.00254, 0.0f, 30000.0f, 20000.0f); // 锟斤拷前pitch锟叫癸拷锟斤拷锟斤拷锟斤拷锟斤拷锟睫凤拷使锟斤拷
	pid_set(&pitch_auto_speed_pid, 11000.0f, 0.0f, 0.0f, 20000.0f, 0.01f);

	pid_set(&yaw_auto_pid, 18.0f, 0.014, 60.0, 27000.0f, 3600.0f); //
	pid_set(&yaw_auto_speed_pid, 17000, 0, 0000, 25000.0f, 0.0f);

	/*云台相关参数初始化*/
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

	/* 云台模式*/
	gimbal.yaw_status = ABSOLUTE; // 默认云台为陀螺仪控制
	gimbal.pitch_status = ABSOLUTE;
}

void gimbal_updata()
{
	// 设置云台电机
	decode_as_6020(YAW_MOTOR);
	decode_as_6020(PITCH_MOTOR);
	// 设置yaw轴数据
	gimbal.yaw.now = IMU_data.AHRS.yaw_rad_cnt;
	gimbal.yaw_speed = cos(IMU_data.AHRS.pitch) * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * IMU_data.gyro[0]; // 多角度融合
	// 设置pitch限位
	// 设置pitch轴数据
	if (Global.input.fly_status == 1)
	{
		gimbal.pitch_status = LOCATION;
	}
	else
		gimbal.pitch_status = ABSOLUTE;

	if (gimbal.pitch_status == LOCATION) // 获取编码器数值用于飞坡时固定pitch
	{
		gimbal.pitch.now = degree2rad(get_motor_data(PITCH_MOTOR).angle - gimbal.pitch.location_offset);
		pitch_fly_angle = gimbal.pitch.now;
		gimbal.pitch_speed = IMU_data.gyro[1]; // 获取y转轴（对应pitch轴）角速度，做闭环用。
	}
	else if (gimbal.pitch_status == ABSOLUTE)
	{
		gimbal.pitch.now = IMU_data.AHRS.pitch - degree2rad(gimbal.pitch.absoulte_offset); // 不要乱改哈，后续会加上绝对/相对的补偿。
		gimbal.pitch_speed = IMU_data.gyro[1];
	}
	// 停在坡上
	slope_calculation(IMU_data.AHRS.pitch, get_motor_data(PITCH_MOTOR).angle); // 计算底盘与地面的角度
}

void gimbal_pid_cal()
{
	gimbal_set_pitch(gimbal.pitch.set, 32.0f, 23.0f);
	/**********************************yaw轴行为控制***********************************************/
	if (gimbal.yaw_status == LOCATION) // 编码器 没用到 留为备用
	{
		gimbal.set_yaw_speed = pid_cal(&yaw_location_pid, gimbal.yaw.now, gimbal.yaw.set);
		set_motor(pid_cal(&yaw_location_speed_pid, gimbal.yaw_speed, gimbal.set_yaw_speed), YAW_MOTOR);
	}
	// yaw全为陀螺仪控制
	else if (gimbal.yaw_status == ABSOLUTE && Global.input.vision_status == 0) // 非自瞄
	{
		gimbal.set_yaw_speed = pid_cal(&yaw_absolute_pid, gimbal.yaw.now, gimbal.yaw.set);
		set_motor(pid_cal(&yaw_absolute_speed_pid, gimbal.yaw_speed, gimbal.set_yaw_speed) + get_motor_data(YAW_MOTOR).round_speed * KF, YAW_MOTOR);
	}
	else if (Global.input.vision_status == 1 && Global.input.anti_stauts == 0) // 自瞄
	{
		gimbal.yaw.set = auto_yaw;
		gimbal.set_yaw_speed = pid_cal(&yaw_auto_pid, gimbal.yaw.now, gimbal.yaw.set);
		set_motor(pid_cal(&yaw_auto_speed_pid, gimbal.yaw_speed, gimbal.set_yaw_speed) + get_motor_data(YAW_MOTOR).round_speed * KF, YAW_MOTOR);
	}
	else if (Global.input.vision_status == 1 && Global.input.anti_stauts == 1) // 反陀螺
	{
		gimbal.set_yaw_speed = pid_cal(&yaw_absolute_pid, gimbal.yaw.now, gimbal.yaw.set);
		set_motor(pid_cal(&yaw_absolute_speed_pid, gimbal.yaw_speed, gimbal.set_yaw_speed) + get_motor_data(YAW_MOTOR).round_speed * KF, YAW_MOTOR);
	}
	else
		gimbal.set_yaw_speed = gimbal.yaw_speed;

	/**********************************pitch轴行为控制***********************************************/

	// 飞坡模式 编码器控制
	if (gimbal.pitch_status == LOCATION)
	{
		gimbal.set_pitch_speed = pid_cal(&pitch_location_pid, gimbal.pitch.now, pitch_fly_angle);
		set_motor(pid_cal(&pitch_location_speed_pid, gimbal.pitch_speed, gimbal.set_pitch_speed), PITCH_MOTOR);
	}
	// 正常模式 陀螺仪控制
	else if (gimbal.pitch_status == ABSOLUTE && Global.input.vision_status == 0) // 非自瞄
	{

		gimbal.set_pitch_speed = pid_cal(&pitch_absolute_pid, gimbal.pitch.now, gimbal.pitch.set);
		set_motor(pid_cal(&pitch_absolute_speed_pid, gimbal.pitch_speed, gimbal.set_pitch_speed), PITCH_MOTOR);
	}
	else if (gimbal.pitch_status == ABSOLUTE && Global.input.vision_status == 1) // 自瞄
	{

		gimbal.pitch.set = auto_pitch;
		gimbal.set_pitch_speed = pid_cal(&pitch_auto_pid, gimbal.pitch.now, gimbal.pitch.set);
		set_motor(pid_cal(&pitch_auto_speed_pid, gimbal.pitch_speed, gimbal.set_pitch_speed), PITCH_MOTOR);
	}
	else
		gimbal.set_pitch_speed = gimbal.pitch_speed;
}
// 零点设置，在main.c中被调用
void gimbal_set_offset(float ab_pitch, float ab_yaw, float lo_pitch, float lo_yaw)
{
	gimbal.pitch.absoulte_offset = ab_pitch;
	gimbal.yaw.absoulte_offset = ab_yaw;
	gimbal.pitch.location_offset = lo_pitch;
	gimbal.yaw.location_offset = lo_yaw;
}
// 设置pitch限位
void gimbal_set_pitch(float pitch, float up_angle, float down_angle)
{

	if (rad2degree(pitch) > up_angle)
	{
		gimbal.pitch.set = degree2rad(up_angle);
	}
	else if (rad2degree(pitch) < -down_angle)
	{
		gimbal.pitch.set = degree2rad(-down_angle);
	}
}

float slope_calculation(float IMU_pitch, float LOCATION_pitch)
{
	slope = rad2degree(IMU_pitch) - LOCATION_pitch - 48.5;
	return slope;
}
//弹舱盖开启
void cover_open(){
	set_servo_angle(PIN_3, 42);
}
//弹舱盖关闭
void cover_close(){
	set_servo_angle(PIN_3, 170);
}
/*********************以下函数没有被用到，保留以防以后用到***********************/

void gimbal_clear_cnt(void)
{
	clear_motor_cnt(YAW_MOTOR);
}
// 对数函数
float lnx(float a, float b, float c, float d, float x)
{
	return a * logf(b * fabs(x) + c) + d;
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

// 设定速度
void gimbal_set_speed(float pitch, float yaw)
{
	gimbal.yaw_status = gimbal.pitch_status = SPEED; // 以速度模式控制

	gimbal.set_pitch_speed = pitch;
	gimbal.set_yaw_speed = yaw;
}

// 未成形的前馈
float feedforward_contorl(float target)
{
	float f_out;
	f_out = (target - Pre_Target) * KF;
	Pre_Target = Target;
	return f_out;
}

/*奇怪的多余函数，回去实验，没用就删掉*/
void gimbal_set_yaw_speed(float yaw)
{
	gimbal.yaw_status = SPEED;
	gimbal.set_yaw_speed = yaw;
}
