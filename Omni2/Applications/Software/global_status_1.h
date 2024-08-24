/**
 * @file global_status.h
 * @author sethome
 * @brief 全局状态机
 * @version 0.1
 * @date 2022-03-25
 *
 * @copyright sethome Copyright (c) 2022
 *
 */

#include "stdint.h"

#define __GLOBAL_STATUS_H__
#ifdef __GLOBAL_STATUS_H__

// 错误码
enum err_e
{
    GIMBAL_ERR = 0,
    CHASSIS_ERR,
    SHOOT_ERR,
    CAP_ERR,
    REMOTE_ERR,
    PC_ERR,
};

// 适合十几个简单状态的情况
struct GlobalStatus_t
{
    uint8_t err[9];

    enum mode_e
    {
        LOCK = 0,
        FLOW,
        TANK,
        SPIN,
				EDGE,
				FLSP,
    } mode;

    enum cap_e
    {
        STOP = 0, // 此时电容应在充电
        FULL,     // 全力响应
    } cap;

    enum team_e
    {
        BLUE_TEAM = 0,
        RED_TEAM,
    } team;

    struct
    {
        /* data */
        float x, y, r; // 底盘移动
        float pitch, yaw;

        uint8_t shooter_status;
        int shoot_fire;
				uint8_t shoot_num;
        uint8_t vision_status;
        uint8_t lid;
			
        float	Global_test_data;
						
	
    } input;
		
		
    struct
    {
				uint8_t trigger_begin;
				uint8_t	trigger_kill;
				uint16_t	trigger_kill_cnt;

    } shoot;		
		
		
		
		
};

extern struct GlobalStatus_t Global;

void Global_set_err(enum err_e err, uint8_t status);

#endif
// end of file
