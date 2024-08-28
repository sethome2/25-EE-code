/**
 * @file gimbal.h
 * @author sethome
 * @brief
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#define __GIMBAL_H__
#ifdef __GIMBAL_H__

#include "pid.h"
#include "stdint.h"
// yaw是5，pitch是6（新东西）
// yaw是6，pitch是5（老东西）
// 云台电机数据
#define PITCH_MOTOR CAN_2_6
#define YAW_MOTOR CAN_2_5
#define PI 3.1415926f

extern pid_t yaw_absolute_speed_pid;
extern pid_t yaw_absolute_pid;

enum gimbal_status_e
{
    LOCATION = 0,
    SPEED,
    ABSOLUTE,
    zero_force,
};

struct gimbal_status
{
    // 设定云台控制模式
    enum gimbal_status_e pitch_status;
    enum gimbal_status_e yaw_status;

    struct
    {
        float set, now, last, absoulte_offset, location_offset;
        float stable;
    } pitch;
    float pitch_speed;
    float set_pitch_speed;

    struct
    {
        float set, now, last, absoulte_offset, location_offset;
        float stable;
    } yaw;
    float yaw_speed;
    float set_yaw_speed;
};

extern struct gimbal_status gimbal;

// 外部调用
void gimbal_init(void);    // 初始化云台
void gimbal_updata(void);  // 更新云台数据
void gimbal_pid_cal(void); // 云台PID计算

void gimbal_set_offset(float ab_pitch, float ab_yaw, float lo_pitch, float lo_yaw); // 初始化零点
void gimbal_set_pitch(float pitch, float up_angle, float down_angle);               // 设定picth角度,限幅用

float slope_calculation(float IMU_pitch, float LOCATION_pitch); // 计算地盘与地面的角度

void gimbal_clear_cnt(void);
void gimbal_set(float pitch, float yaw);       // 设置角度
void gimbal_set_speed(float pitch, float yaw); // 设定速度
float feedforward_contorl(float target);       // 前馈

void gimbal_set_yaw_speed(float yaw); // 设定yaw速度
void cover_open(void);                // 开启弹舱盖
void cover_close(void);               // 关闭弹舱盖
// end of file

#endif
