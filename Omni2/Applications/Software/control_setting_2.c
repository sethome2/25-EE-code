#include "CAN_receive&send.h"
#include "IMU_updata.h"
#include "DBUS_remote_control.h"
#include "PWM_control.h"
#include "LED_control.h"
#include "AHRS_MiddleWare.h"
#include "cap_ctl.h"
#include "Laser.h"

#include "RM_Cilent_UI.h"
#include "referee_handle_pack.h"
#include "referee_usart_task.h"
#include "usbd_cdc_if.h"
#include "USB_VirCom.h"

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


uint8_t is_input_for_rc = 1;
uint32_t Time_LEAN_delay = 0;
uint32_t Time_SPIN_delay = 0;
uint32_t Time_TANK_delay = 0;

uint32_t Time_delay_friction_wheel = 0;
uint32_t Time_delay_Switch = 0;
////键鼠操作灵敏度，亲测不能使用，鬼知道为什么
//uint16_t yaw_sen = 2800.0f;
//uint16_t pitch_sen= 1700.0f;

/******************************遥控器拨杆任务**************************************************/	
void remote_control_task()
{

  //左上右上  键鼠模式
	if (switch_is_up(RC_L_SW) && switch_is_up(RC_R_SW))
		 Global.input.ctl=PC;
	
	//左中右中   底盘跟随
	if (switch_is_mid(RC_L_SW) && switch_is_mid(RC_R_SW))
	{
		Global.mode = FLOW;	
		Global.input.ctl=RC;
	}
	
	//左下右下   锁死
	if(switch_is_down(RC_L_SW) && switch_is_down(RC_R_SW))
	  Global.mode = LOCK;	
	
//	//左中右上   TANK
//	if (switch_is_mid(RC_L_SW) && switch_is_up(RC_R_SW))
//	{
//		Global.mode = TANK;	
//		Global.input.ctl=RC;
//	}	
	
	//左中右下   SPIN
	if (switch_is_mid(RC_L_SW) && switch_is_down(RC_R_SW))
	{
		Global.mode = SPIN;	
		Global.input.ctl=RC;
	}	

	
	//左下右中   AUTO_AIM
	if (switch_is_down(RC_L_SW) && switch_is_mid(RC_R_SW))
	{
		Global.input.vision_status = 1;
		Global.input.ctl=RC;		
	}
	else
		Global.input.vision_status = 0;			
	
	
/******************************RC遥控器操作逻辑**************************************************/	
	if (Global.input.ctl==RC) // rc input
	{
		// 移动
		Global.input.r = RC_data.rc.ch[2] / 110.0f;
		Global.input.x = RC_data.rc.ch[0] / 110.0f;
		Global.input.y = RC_data.rc.ch[1] / 110.0f;

		// 云台
		/*自瞄控制，思路很简单，因为视觉回传的是偏差，那我的目标值就是当前yaw+偏差=目标yaw（实时），
		为什么不是+=呢，因为这个方式，在机器人转的过程中还有新的数据传入，此时的SET会>目标值，
		而且必须要除以一个数值，导致最后会无限逼近而不是完全收敛
		*/
		if (Global.input.vision_status == 1)
		{
		}
		else
		{
			fromNUC.pitch = 0;
			fromNUC.yaw = 0;
			Global.input.yaw = RC_data.rc.ch[2] / 6000.0f;//回头键鼠部分也要减小
			Global.input.pitch = -RC_data.rc.ch[3] /7000.0f;
		}
		
		// CH4 波轮
		// 摩擦轮逻辑
		if (RC_data.rc.ch[4] == 0)
		{
			Global.input.shooter_status = 0;
		}
		if ((RC_data.rc.ch[4] > 300 && RC_data.rc.ch[4] < 660))
		{
			Global.input.shooter_status = 1;
		}
		
		
	// 发弹逻辑
		if (RC_data.rc.ch[4] > 600 && RC_data.rc.ch[4] <= 660)
		{
			trigger_anti_kill_and_set_speed(3000.0f);//3000
		}
		else if (RC_data.rc.ch[4] > 6000 && RC_data.rc.ch[4] <= 7000)
	//对于这个抽象操作的解释：拨轮上拨0-7000，下拨0-660,所以上下两个逻辑
		{
			trigger_anti_kill_and_set_speed(3000.0f);//3000
		}
		else
		{
			trigger_anti_kill_and_set_speed(0);
		}			
	}
/******************************PC操作逻辑**************************************************/
	else if (Global.input.ctl==PC) // keyborad&mouuse_input_begin
	{
		//自瞄状态检测
		Global.input.vision_status = RC_data.mouse.press_r;
		// 云台
		if (Global.input.vision_status == 1)//自瞄控制
		{
			auto_ctrl();
		}
		else		//操作手接管yaw+pitch
		{
		//还不如简单的叠加手感好，狗屎，下次答疑不录象全靠回忆给我说我砍死你
			vision_reset();
			Global.input.yaw = MOUSE_X_MOVE_SPEED/7000.0f;
			Global.input.pitch =MOUSE_Y_MOVE_SPEED/6000.0f;	
		}
		/*************底盘模式切换****************/
		//开启小陀螺模式
		if (IF_KEY_PRESSED_Q)
		{
			if (Get_sys_time_ms() - Time_SPIN_delay > 350)//键盘防抖
			{
				if (Global.mode == SPIN)
					Global.mode = FLOW;
				else if (Global.mode != SPIN)
					Global.mode = SPIN;
				Time_SPIN_delay = Get_sys_time_ms();
			}
		}

		//开启45度\格斗模式
		 if (IF_KEY_PRESSED_CTRL)
		 {
			if (Get_sys_time_ms() - Time_LEAN_delay > 350)
			{
				if (Global.mode == LEAN)
					Global.mode = FLOW;
				else if (Global.mode != LEAN)
					Global.mode = LEAN;
				Time_LEAN_delay = Get_sys_time_ms();
			}
		 }

		//开启坦克模式
		if (IF_KEY_PRESSED_F)
		{
			if (Get_sys_time_ms() - Time_TANK_delay > 350)
			{
				if (Global.mode == TANK)
					Global.mode = FLOW;
				else if (Global.mode != TANK)
					Global.mode = TANK;
				Time_TANK_delay = Get_sys_time_ms();
			}
		}

		//开启超级电容加速
		if (IF_KEY_PRESSED_SHIFT)
		{
			Global.cap = FULL;
		}
		else
		{
			Global.cap = STOP;
		}

		/*************发射模式控制****************/
		Global.input.shoot_fire = RC_data.mouse.press_l;
		// 摩擦轮开关
		if (IF_KEY_PRESSED_R)		
		{
			if (Get_sys_time_ms() - Time_delay_friction_wheel > 350)
			{
				if (Global.input.shooter_status == 0) // 切换状态
				{
					Global.input.shooter_status = 1;//发射标志置位
				}
				else
				{
					Global.input.shooter_status = 0;//发射机构关闭
				}

				Time_delay_friction_wheel = Get_sys_time_ms();
			}
		}		
		//单发连发
		if (IF_KEY_PRESSED_E)
		{
			if (Get_sys_time_ms() - Time_delay_Switch > 350)
			{
				if (Global.input.shoot_num != 1) // 切换状态
				{
					Global.input.shoot_num = 1;
				}
				else if (Global.input.shoot_num == 1)
				{
					Global.input.shoot_num = 0;
				}
				Time_delay_Switch = Get_sys_time_ms();
			}
		}		
		//开炮！！！！
		if (Global.input.shoot_fire)	
		{
			if(Global.input.anti_stauts == 0)//反陀螺模式是否开启
				shoot_trigger_online();//正常发弹控制
			else if(Global.input.anti_stauts == 1)
			{
				trigger_anti_kill_and_set_speed(8000.0f);//反陀螺，使用速度环控制，暂时不改动
			}
			else
				trigger_anti_kill_and_set_speed(0);
		}	
		else
		{
			trigger_anti_kill_and_set_speed(0);	
		}		
	

		//移动控制
		if (IF_KEY_PRESSED_W)
		{
			if (Global.cap == FULL)
			{
				if (Global.input.y < 2.6f)
					Global.input.y += 0.2f;
				// Global.input.y = 10.0f;
			}
			else
			{
				if (Global.input.y < 1.6f)
					Global.input.y += 0.2f;
				// Global.input.y = 8.0f;
			}
		}
		else if (IF_KEY_PRESSED_S)
		{
			if (Global.cap == FULL)
			{
				if (Global.input.y > -2.6f)
					Global.input.y -= 0.2f;
				// Global.input.y = -10.0f;
			}
			else
			{
				if (Global.input.y > -1.6f)
					Global.input.y -= 0.2f;
				// Global.input.y = -8.0f;
			}
		}
		else
			Global.input.y = 0;

		if (IF_KEY_PRESSED_A)
		{
			if (Global.cap == FULL)
			{
				if (Global.input.x > -2.6f)
					Global.input.x -= 0.2f;
				// Global.input.x = -10.0f;
			}
			else
			{
				if (Global.input.x > -1.6f)
					Global.input.x -= 0.2f;
				// Global.input.x = -8.0f;
			}
		}
		else if (IF_KEY_PRESSED_D)
		{
			if (Global.cap == FULL)
			{
				if (Global.input.x < 2.6f)
					Global.input.x += 0.2f;
				// Global.input.x = 10.0f;
			}
			else
			{
				if (Global.input.x < 1.6f)
					Global.input.x += 0.2f;
				// Global.input.x = 8.0f;
			}
		}
		else
			Global.input.x = 0;



		if (IF_KEY_PRESSED_V) //按V发送UI串口包，初始化用
		{
			press_refrsh();
		}

		if (IF_KEY_PRESSED_Z) //按Z确定，重新初始化
		{
			UI_task_init(); // UI图层初始化
		}
		//反陀螺
			
		if(IF_KEY_PRESSED_X)
			Global.input.anti_stauts = 1;
		else
			Global.input.anti_stauts = 0;		
		
	}
}
