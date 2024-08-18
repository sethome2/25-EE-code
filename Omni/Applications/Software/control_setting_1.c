#include "usbd_cdc_if.h"
#include "CAN_receive&send.h"
#include "DBUS_remote_control.h"
#include "LED_control.h"
#include "IMU_updata.h"
#include "PWM_control.h"
#include "Stm32_time.h"

#include "TF_MINI_PLUS_LaserRanging.h"
#include "chassis_move.h" //普通底盘
#include "gimbal.h"
#include "shoot.h"
#include "math.h"
#include "small_tools.h"
#include "USB_VirCom.h"

#include "Global_status.h"
#include "Error_detect.h"
#include "NUC_communication.h"
#include "PC_communication.h"

#include "MYACTUATOR_RMD_X.h"
#include "referee.h"
#include "RM_Cilent_UI.h"
#include "cap_ctl.h"
#include "referee_handle_pack.h"
#include "referee_usart_task.h"
#include "fifo.h"
#include "AHRS_MiddleWare.h"
#include "CAN_receive&send.h"
#include "Laser.h"

	uint8_t is_input_for_rc = 1;

	unsigned long long Time_Test_Lean = 0;
	unsigned long long Time_Test_SPIN = 0;
	unsigned long long Time_Test_TANK = 0;
	unsigned long long Time_Test_FLSP = 0;
	unsigned long long Time_Test_friction_wheel = 0;
	unsigned long long Time_Test_Switch = 0;	
	unsigned long long Time_Test_Lid = 0;
  unsigned long long last_shoot_time = 0;
	unsigned long long last_shootback_time = 0;
	
	
void flash_LED_Task()
{
		if (Global.team == RED_TEAM)
			led_show(RED);
		else if (Global.team == BLUE_TEAM)
			led_show(BLUE);

		osDelay(250);
		
		uint8_t err_flag = 0;
		if (Global.err[CHASSIS_ERR] == 1)
		{
			led_show(PINK);
			err_flag++;
			osDelay(150);
		}
		if (Global.err[GIMBAL_ERR] == 1)
		{
			led_show(YELLOW);
			err_flag++;
			osDelay(150);
		}
		if (Global.err[SHOOT_ERR] == 1)
		{
			led_show(PURPLE);
			err_flag++;
			osDelay(150);
		}
		if (Global.err[REMOTE_ERR] == 1)
		{
			led_show(ORANGE);
			err_flag++;
			osDelay(150);
		}

		if (err_flag == 0)
		{
			led_show(GREEN);
			osDelay(250);
		}	
	
}
	
	
void remove_control_task()
{
		// 键鼠切换
		if (switch_is_down(RC_L_SW))
		{
			is_input_for_rc = 0;
		}
		else
			is_input_for_rc = 1;

		if (is_input_for_rc == 1) // rc input
		{
			// 移动
			if(abs(RC_data.rc.ch[2])>3)
					Global.input.r = RC_data.rc.ch[2] / 165.0f;
					Global.input.x = RC_data.rc.ch[0] / 165.0f;
					Global.input.y = RC_data.rc.ch[1] / 165.0f;
//				Global.input.x = RC_data.rc.ch[0] / 660.0f;
//				Global.input.y = RC_data.rc.ch[1] / 660.0f;
//				Global.input.r = RC_data.rc.ch[2] / 660.0f;
			

			// 模式切换
			//左中右上小陀螺
			if (switch_is_mid(RC_L_SW)&&switch_is_up(RC_R_SW))
				Global.mode = SPIN;
				//左中右中跟随模式
			if (switch_is_mid(RC_L_SW)&&switch_is_mid(RC_R_SW))				
				Global.mode = FLOW;
			//左中右下45°格斗模式
			if (switch_is_mid(RC_L_SW)&&switch_is_down(RC_R_SW))
				Global.mode = EDGE; 
			//左上右上自锁
			if (switch_is_up(RC_L_SW)&&switch_is_up(RC_R_SW))
				Global.mode = LOCK; 
			//左上右中坦克
			if (switch_is_up(RC_L_SW)&&switch_is_mid(RC_R_SW))
				Global.mode = TANK; 
			
			
					// 是否开启自瞄
			if (switch_is_mid(RC_L_SW))
				Global.input.vision_status = 0;
			if (switch_is_down(RC_L_SW))
				Global.input.vision_status = 1;	
			
			// 云台
			if (Global.input.vision_status == 1) //自瞄控制
			{
				if (fabs(fromNUC.yaw) > 0.0f)
					Global.input.yaw = fromNUC.yaw / 130.0f;
				else
					Global.input.yaw = 0.0f;

				if (fabs(fromNUC.pitch) > 0.0f)
					Global.input.pitch = -fromNUC.pitch / 1000.0f;
				else
					Global.input.pitch = 0.0f;
			}
			else
			{
				fromNUC.pitch=0;
				fromNUC.yaw=0;
				Global.input.yaw = RC_data.rc.ch[2] / 400.0f;
				Global.input.pitch = -RC_data.rc.ch[3] / 15000.0f;								
			}


			
			// CH4 波轮
			// 摩擦轮逻辑
			if (RC_data.rc.ch[4] == 0)
			{ 
				Global.input.shooter_status = 0;
			}
			if (RC_data.rc.ch[4] > 400 && RC_data.rc.ch[4] < 600)
			{
				Global.input.shooter_status = 1;
			}


				//拨弹状态			
				if (Get_sys_time_ms() - last_shoot_time > 100000)
				{
					if (abs(motor_data[17].speed_rpm)>600) // 切换状态
					{
						Global.shoot.trigger_begin = 1;						
					}
					else
					{
						Global.shoot.trigger_begin  = 0;							
					}

					last_shoot_time = Get_sys_time_ms();
				}			
			
			
			// 发弹逻辑
//			if (RC_data.rc.ch[4] > 650 && RC_data.rc.ch[4] <= 660)
				if (RC_data.rc.ch[4] == 660)
			{	
				trigger_set_speed(5000.0f);	
				
			}
			else if (RC_data.rc.ch[4] > 6500 && RC_data.rc.ch[4] <= 8500) // 大于一半，连发
			{	
				trigger_set_speed(10000.0f);	
			}
			else
				trigger_set_speed(0.0f);
		}

		
		
		// keyborad&mouuse input

		if (is_input_for_rc != 1) // keyborad&mouuse_input_begin
		{
			//移动控制
			if (IF_KEY_PRESSED_W)
			{
				if (Global.cap == FULL)
				{
					if (Global.input.y < 4.0f)
						Global.input.y += 0.04f;
				}
				else if(Global.input.lid == 1)
				{
					if (Global.input.y < 1.2f)
						Global.input.y += 0.04f;				
				}
				else
				{
					if (Global.input.y < 4.0f)
						Global.input.y += 0.04f;
				}
			}
			else if (IF_KEY_PRESSED_S)
			{
				if (Global.cap == FULL)
				{
					if (Global.input.y > -4.0f)
						Global.input.y -= 0.04f;
				}
				else if(Global.input.lid == 1)
				{
					if (Global.input.y > -1.2f)
						Global.input.y -= 0.04f;				
				}				
				else
				{
					if (Global.input.y > -4.0f)
						Global.input.y -= 0.04f;
				}
			}
			else
				Global.input.y = 0;

			
			
			if (IF_KEY_PRESSED_A)
			{
				if (Global.cap == FULL)
				{
					if (Global.input.x > -4.0f)
						Global.input.x -= 0.04f;
				}
				else if(Global.input.lid == 1)
				{
					if (Global.input.x > -1.2f)
						Global.input.x -= 0.04f;				
				}				
				else
				{
					if (Global.input.x > -4.0f)
						Global.input.x -= 0.04f;
				}
			}
			else if (IF_KEY_PRESSED_D)
			{
				if (Global.cap == FULL)
				{
					if (Global.input.x < 4.0f)
						Global.input.x += 0.04f;
				}
				else if(Global.input.lid == 1)
				{
					if (Global.input.x < 1.2f)
						Global.input.x += 0.04f;				
				}				
				else
				{
					if (Global.input.x < 4.0f)
						Global.input.x += 0.04f;
				}
			}
			else
				Global.input.x = 0;

			
			//云台控制
		  if (Global.input.vision_status == 1) //自瞄控制
			{
				if (fabs(fromNUC.yaw) > 0.0f)
					Global.input.yaw = fromNUC.yaw / 160.0f;//130  传统自瞄
				else
					Global.input.yaw = 0.0f;

				if (fabs(fromNUC.pitch) > 0.0f)
					Global.input.pitch = -fromNUC.pitch / 1250.0f;//1000  传统自瞄
				else
					Global.input.pitch = 0.0f;
				auto_aim_ui_update(1);				
			}
			else
			{
				fromNUC.pitch=0;
				fromNUC.yaw=0;
				Global.input.yaw = MOUSE_X_MOVE_SPEED / 25.0f;
				if (MOUSE_Z_MOVE_SPEED != 0)
					Global.input.pitch = MOUSE_Z_MOVE_SPEED / 650.0f;
				else
					Global.input.pitch = MOUSE_Y_MOVE_SPEED / 650.0f;
				auto_aim_ui_update(0);					
			}


			//置位
			Global.input.vision_status = RC_data.mouse.press_r;
			Global.input.shoot_fire = RC_data.mouse.press_l;


			//开启超级电容
			if (IF_KEY_PRESSED_SHIFT)
			{
					Global.cap = FULL;
					super_cap_speedup(1); //超级电容放电UI
			}
			else
			{
					Global.cap = STOP;
					super_cap_speedup(0); //超级电容充电UI
			}


			// 开弹舱盖
			if (IF_KEY_PRESSED_C)
			{
				if (Get_sys_time_ms() - Time_Test_Lid > 350) 
				{
					if (Global.input.lid != 1)
						Global.input.lid = 1;
					else
						Global.input.lid = 0;
					Time_Test_Lid = Get_sys_time_ms();
				}
			}

			if (Global.input.lid == 1)
			{
				set_servo_angle(PIN_1, 70); //打开模式
				lid_status(1);				//弹舱盖开启UI
			}
			else
			{
				set_servo_angle(PIN_1, 180); //关闭模式
				lid_status(0);				 //弹舱盖关闭UI
			}

			// 摩擦轮开关
			if (IF_KEY_PRESSED_R)
			{
				if (Get_sys_time_ms() - Time_Test_friction_wheel > 350)
				{
					if (Global.input.shooter_status == 0) // 切换状态
					{
						Global.input.shooter_status = 1;						
					}
					else
					{
						Global.input.shooter_status = 0;							
					}

					Time_Test_friction_wheel = Get_sys_time_ms();
				}
			}
			
			if(IF_KEY_PRESSED_E)
			{
				if (Get_sys_time_ms() - Time_Test_Switch  > 350)
				{
					if (Global.input.shoot_num !=1) // 切换状态
					{
						Global.input.shoot_num = 1;
						trigger_refresh(2);								
					}	
					else if(Global.input.shoot_num ==1)
					{
						Global.input.shoot_num = 0;
						trigger_refresh(1);								
					}	
					Time_Test_Switch = Get_sys_time_ms();
				}
			}			
		
			
			//开火
			if (Global.input.shoot_fire)
			{
				shoot_trigger_online();	
				
//				shoot_Bullets(-0.01);
			}
			else
			{
				trigger_set_speed(0);		
//				trigger_refresh(0);			    	
			}

		
			if(IF_KEY_PRESSED_X)//手动反转拨弹轮
			{
				if(Get_sys_time_ms()-last_shootback_time>1000)
				{
					shoot_set_back();
					last_shootback_time=Get_sys_time_ms();
				}
			}

		
			
			//开启45度\格斗模式
			if (IF_KEY_PRESSED_CTRL)
			{
				if (Get_sys_time_ms() - Time_Test_Lean > 350)
				{
					if (Global.mode == EDGE)
						Global.mode = FLOW;
					else if (Global.mode != EDGE)
						Global.mode = EDGE;
					Time_Test_Lean = Get_sys_time_ms();
				}
			}

			//开启小陀螺模式
			if (IF_KEY_PRESSED_Q)
			{
				if (Get_sys_time_ms() - Time_Test_SPIN > 350)
				{
					if (Global.mode == SPIN)
						Global.mode = FLOW;
					else if (Global.mode != SPIN)
						Global.mode = SPIN;
					Time_Test_SPIN = Get_sys_time_ms();
				}
			}

			//开启坦克模式
			if (IF_KEY_PRESSED_F)
			{
				if (Get_sys_time_ms() - Time_Test_TANK > 350)
				{
					if (Global.mode == TANK)
						Global.mode = FLOW;
					else if (Global.mode != TANK)
						Global.mode = TANK;
					Time_Test_TANK = Get_sys_time_ms();
				}
			}

			//开启飞坡模式  
			if (IF_KEY_PRESSED_B)
			{
				if (Get_sys_time_ms() - Time_Test_FLSP > 350)
				{
					if (Global.mode == FLSP)
						Global.mode = FLOW;
					else if (Global.mode != FLSP)
						Global.mode = FLSP;
					Time_Test_FLSP = Get_sys_time_ms();
				}
			}

			if (IF_KEY_PRESSED_V) //按V发送UI串口包，初始化用
			{
				press_refrsh();			
			}

			
			if (IF_KEY_PRESSED_G) //按G确定，开始刷新
			{
				press_refrsh_flag = 1;
			}
			
			
			if (IF_KEY_PRESSED_Z) //按Z确定，重新初始化
			{
				  press_refrsh_flag = 0;					
					UI_task_init();            // UI图层初始化	
			}
							
		}	
	
}
	
