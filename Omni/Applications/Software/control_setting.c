#include "CAN_receive&send.h"
#include "IMU_updata.h"
#include "DBUS_remote_control.h"
#include "PWM_control.h"
#include "LED_control.h"
#include "AHRS_MiddleWare.h"
#include "cap_ctl.h"
#include "Laser.h"
#include "can.h"
#include "freertos.h"
#include "cmsis_os2.h"

#include "referee_handle_pack.h"
#include "referee_usart_task.h"
#include "usbd_cdc_if.h"
#include "USB_VirCom.h"

#include "ui.h"
#include "Global_status.h"
#include "Error_detect.h"
#include "chassis_move.h"
#include "gimbal.h"
#include "shoot.h"
#include "math.h"
#include "RampFunc.h"
#include "NUC_communication.h"
#include "control_setting.h"
#include "Stm32_time.h"
#include "usb_device.h"

extern float DBUStime1;

uint8_t is_input_for_rc = 1;
uint32_t Time_LEAN_delay = 0;
uint32_t Time_SPIN_delay = 0;
uint32_t Time_TANK_delay = 0;
uint32_t Time_delay_lid = 0;
uint32_t Time_ANTI_delay = 0;
uint32_t Time_delay_friction_wheel = 0;
uint32_t Time_delay_resetcan = 0;
uint32_t Time_delay_Switch = 0;
uint8_t coverop = 0;

float KS;

/******************************遥控器拨杆任务**************************************************/
void Remote_Control()
{
	if ((Get_sys_time_ms() - DBUStime1) > 1000.0f)
	{
		image_to_RC(&RC_data); // 切换到图传链路
	}
	// 左上右上  键鼠模式
	if (switch_is_up(RC_L_SW) && switch_is_up(RC_R_SW))
	{
		Global.input.ctl = PC;
	}
	else
		Global.input.ctl = RC;
	// 左中右中   底盘跟随
	if (switch_is_mid(RC_L_SW) && switch_is_mid(RC_R_SW))
	{
		Global.mode = FLOW;
	}

	// 左下右下   锁死
	if (switch_is_down(RC_L_SW) && switch_is_down(RC_R_SW))
	{
		Global.mode = LOCK;
	}
	// 左上右下   TUENSPIN
	if (switch_is_up(RC_L_SW) && switch_is_down(RC_R_SW))
	{
		Global.mode = TUSP;
	}

	// 左中右下   SPIN
	if (switch_is_mid(RC_L_SW) && switch_is_down(RC_R_SW))
	{
		Global.mode = SPIN;
	}

	// 左下右中   AUTO_AIM
	if (switch_is_down(RC_L_SW) && switch_is_mid(RC_R_SW))
	{
		Global.input.vision_status = 1;
	}
	else
		Global.input.vision_status = 0;
}
/******************************RC遥控器操作逻辑**************************************************/
void Remote_Control_RC()
{
	// 移动
	Global.input.r = RC_data.rc.ch[2] / 110.0f;
	Global.input.x = RC_data.rc.ch[0] / 110.0f;
	Global.input.y = RC_data.rc.ch[1] / 80.0f;
	// 自瞄
	if (Global.input.vision_status == 1)
	{
		Global.input.yaw = 0;
		Global.input.pitch = 0;
		auto_ctrl();
	}
	else
	{
		fromNUC.pitch = 0;
		fromNUC.yaw = 0;
		Global.input.yaw = RC_data.rc.ch[2] / 6000.0f; // 回头键鼠部分也要减小
		Global.input.pitch = -RC_data.rc.ch[3] / 7000.0f;
	}
	// CH4 波轮
	// 摩擦轮逻辑
	if (RC_data.rc.ch[4] == 0)
	{
		Global.input.shooter_status = 0;
	}
	if ((RC_data.rc.ch[4] > 300 && RC_data.rc.ch[4] < 660))
	{
		Global.input.shoot_num = 1;		 // 连发
		Global.input.shooter_status = 1; // 摩擦轮启动
	}
	// 发弹逻辑
	if (RC_data.rc.ch[4] > 600 && RC_data.rc.ch[4] <= 660)
	{
		Global.input.shoot_RC = 80;
	}
	else if (RC_data.rc.ch[4] > 6000 && RC_data.rc.ch[4] <= 7000)
	// 对于这个抽象操作的解释：拨轮上拨0-7000，下拨0-660,所以上下两个逻辑
	{
		Global.input.shoot_RC = 100;
	}
	else
	{
		Global.input.shoot_RC = 0; // 没有拨到位置不发子弹
	}
	if (Global.mode == LOCK) // 切换自锁模式来切换弹仓盖的开合
		cover_open();
	else
		cover_close();
}

/******************************PC操作逻辑**************************************************/
void Remote_Control_PC()
{
	// 自瞄状态检测
	Global.input.vision_status = RC_data.mouse.press_r;
	// 云台
	if (Global.input.vision_status == 1 && Global.input.anti_stauts == 0) // 自瞄控制
	{
		Global.input.yaw = 0;
		Global.input.pitch = 0;
		auto_ctrl();
	}
	else if (Global.input.vision_status == 1 && Global.input.anti_stauts == 1) // 反陀螺 操作手接管yaw
	{
		Global.input.pitch = 0;
		Global.input.yaw = MOUSE_X_MOVE_SPEED / 4500.0f;
		auto_ctrl();
	}
	else // 手动模式 操作手接管yaw+pitch
	{
		vision_reset();
		Global.input.yaw = MOUSE_X_MOVE_SPEED / 4500.0f;
		if (Global.input.fly_status != 1) // 飞坡模式中不记录pitch轴数据
			Global.input.pitch = MOUSE_Y_MOVE_SPEED / 3000.0f;
	}
	/*************底盘模式切换****************/
	// 开启小陀螺模式
	if (IF_KEY_PRESSED_Q)
	{
		if (Get_sys_time_ms() - Time_SPIN_delay > 350) // 键盘防抖
		{
			if (Global.mode == SPIN)
				Global.mode = FLOW;
			else if (Global.mode != SPIN)
			{
				Global.mode = SPIN;
				Global.input.fly_status = 0;
			}
			Time_SPIN_delay = Get_sys_time_ms();
		}
	}

	// 开启超级电容加速
	if (IF_KEY_PRESSED_SHIFT)
		Global.cap = FULL;
	else
		Global.cap = STOP;

	/*************发射模式控制****************/
	Global.input.shoot_fire = RC_data.mouse.press_l;
	// 摩擦轮开关
	if (IF_KEY_PRESSED_R)
	{
		if (Get_sys_time_ms() - Time_delay_friction_wheel > 350)
		{
			if (Global.input.shooter_status == 0) // 切换状态
			{
				Global.input.shooter_status = 1; // 发射标志置位
			}
			else
			{
				Global.input.shooter_status = 0; // 发射机构关闭
			}

			Time_delay_friction_wheel = Get_sys_time_ms();
		}
	}
	// 单发连发
	if (IF_KEY_PRESSED_E)
	{
		if (Get_sys_time_ms() - Time_delay_Switch > 350)
		{
			if (Global.input.shoot_num != 1) // 切换状态
			{
				Global.input.shoot_num = 1; // 连发模式
				shoot.trigger_status = SPEEDS;
			}
			else if (Global.input.shoot_num == 1)
			{
				Global.input.shoot_num = 0; // 单发模式
				shoot.trigger_status = LOCATIONS;
			}
			Time_delay_Switch = Get_sys_time_ms();
		}
	}

	/*************底盘行进****************/
	if (REFEREE_DATA.Chassis_Power_Limit <= 60)
	{
		KS = 1.0f;
	}
	else if (REFEREE_DATA.Chassis_Power_Limit > 60 && REFEREE_DATA.Chassis_Power_Limit <= 80)
	{
		KS = 1.1f;
	}
	else if (REFEREE_DATA.Chassis_Power_Limit > 80 && REFEREE_DATA.Chassis_Power_Limit <= 120)
	{
		KS = 1.2f;
	}

	if (IF_KEY_PRESSED_W)
	{
		if (Global.input.fly_status == 1)
		{

			if (Global.input.y < 1.0f)
				Global.input.y += 0.0025f;
			else if (Global.input.y >= 1.0f && Global.input.y < 3.0f)
				Global.input.y += 0.004f;
			else if (Global.input.y >= 3.0f && Global.input.y < 5.0f)
				Global.input.y += 0.005f;
			else if (Global.input.y >= 5.0f && Global.input.y < 9.0f)
				Global.input.y += 0.008f;
		}
		else
		{

			if (Global.cap == FULL)
			{
				Global.input.y += 0.04;
				if (Global.input.y >= 3.0)
					Global.input.y = 3.0;
			}
			else
			{

				if (Global.input.y < 0.5)
					Global.input.y += 0.004f;
				if (Global.input.y > 0.5 && Global.input.y < 0.8)
					Global.input.y += 0.025f;
				if (Global.input.y > 0.8 && Global.input.y < 1.6 * KS)
					Global.input.y += 0.04f;
				if (Global.input.y > 1.6 * KS)
					Global.input.y = 1.6 * KS;

				if (Global.input.lid == 1)
					Global.input.y = 0.7;
			}
		}
	}
	else if (IF_KEY_PRESSED_S)
	{
		if (Global.cap == FULL)
		{
			if (Global.input.y > -1.0f)
				Global.input.y -= 0.0025f;
			else if (Global.input.y <= -1.0f && Global.input.y > -3.0f)
				Global.input.y -= 0.004f;
			else if (Global.input.y <= -3.0f && Global.input.y > -5.0f)
				Global.input.y -= 0.005f;
			else if (Global.input.y <= -5.0f && Global.input.y > -8.0f)
				Global.input.y -= 0.007f;
		}
		else
		{
			if (Global.input.y > -1.6f * KS)
				Global.input.y -= 0.02f;
			if (Global.input.lid == 1)
				Global.input.y = -0.7;
		}
	}
	else
		Global.input.y = 0;

	if (IF_KEY_PRESSED_A)
	{
		if (Global.cap == FULL)
		{
			if (Global.input.x > -2.0f)
				Global.input.x -= 0.08f;
		}
		else
		{
			if (Global.input.x > -1.8f)
				Global.input.x -= 0.02f;
			if (Global.input.lid == 1)
				Global.input.x = -0.7;
		}
	}
	else if (IF_KEY_PRESSED_D)
	{
		if (Global.cap == FULL)
		{
			if (Global.input.x < 2.0f)
				Global.input.x += 0.08f;
		}
		else
		{

			if (Global.input.x < 1.8f)
				Global.input.x += 0.02f;
			if (Global.input.lid == 1)
				Global.input.x = 0.7;
		}
	}
	else
		Global.input.x = 0;

	if (IF_KEY_PRESSED_V) // 保留
	{
	}

	if (IF_KEY_PRESSED_Z) // UI初始化
	{
		Global.input.ui_init = 1;
	}
	// 反陀螺

	if (IF_KEY_PRESSED_X)
	{
		if (Get_sys_time_ms() - Time_ANTI_delay > 350)
		{
			if (Global.input.anti_stauts == 0)
				Global.input.anti_stauts = 1;
			else
				Global.input.anti_stauts = 0;
			Time_ANTI_delay = Get_sys_time_ms();
		}
	}

	// 开弹舱盖
	if (IF_KEY_PRESSED_C)
	{

		if (Get_sys_time_ms() - Time_delay_lid > 350)
		{
			if (Global.input.lid != 1)
				Global.input.lid = 1;
			else
				Global.input.lid = 0;
			Time_delay_lid = Get_sys_time_ms();
		}
		if (Global.input.lid == 1)
		{
			cover_open();
		}
		else
		{
			cover_close();
		}
	}

	if (IF_KEY_PRESSED_G)
	{
		if (Get_sys_time_ms() - Time_delay_friction_wheel > 350)
		{
			if (Global.input.fly_status == 0) // 切换状态
			{
				Global.input.fly_status = 1; // 发射标志置位
				gimbal.pitch_status = LOCATION;
			}
			else if (IF_KEY_PRESSED_Q)
			{
				Global.input.fly_status = 0; // 发射机构关闭
				gimbal.pitch_status = ABSOLUTE;
			}
			else
			{
				Global.input.fly_status = 0; // 发射机构关闭
				gimbal.pitch_status = ABSOLUTE;
			}

			Time_delay_friction_wheel = Get_sys_time_ms();
		}
	}
}

void remote_control_task()
{
	Remote_Control();
	if (Global.input.ctl == PC)
		Remote_Control_PC();
	else if (Global.input.ctl == RC)
		Remote_Control_RC();
	else
	{
	}
}