#include "referee_usart_task.h"
#include "usart.h"
#include "crcs.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "Gimbal_Task.h"
#include "bsp_cap.h"
#include "remote_control.h"
#include "arm_math.h"
#include "Nmanifold_usart_task.h"
#include "CAN_receive.h"
#include "ChassisSwitch_Task.h"
#include "ChassisControl_Task.h"
#include "ChassisDetect_Task.h"

/* Private define ------------------------------------------------------------*/
#define Max(a,b) ((a) > (b) ? (a) : (b))
//#define PUSH_DELAY 100
//#define Robot_ID_Current Robot_ID_Blue_Infantry3
/* Private variables ---------------------------------------------------------*/
/* 裁判系统串口双缓冲区 */
uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

/* 裁判系统接收数据队列 */
fifo_s_t Referee_FIFO;
uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol解析包结构体 */
unpack_data_t Referee_Unpack_OBJ;


/* 动态UI数据变量 */
uint8_t Robot_ID_Current;
UI_String_t UI_String1;
float   UI_Gimbal_Pitch = 0.0f; //云台Pitch轴角度
float   UI_Gimbal_Roll = 0.0f; //云台Pitch轴角度
float   UI_Chassis_Follow_Gimbal_Angle=0.0f;//底盘跟随云台角度
uint8_t UI_Chassis_Fall = 0; //底盘倒下

uint8_t UI_Capacitance  = 10;   //电容剩余容量
uint8_t UI_fric_is_on   = 0;    //摩擦轮是否开启

/* 动态UI控制变量 */
uint16_t UI_Push_Cnt = 0;
uint8_t UI_Flash_Flag = 1;

/* 中央标尺高度变量 */
uint16_t y01 = 455;
uint16_t y02 = 420;
uint16_t y03 = 280;
uint16_t y04 = 230;
/*小坦克位置补偿*/
uint16_t tank_x =600;
uint16_t tank_y =200;
/*血条中心位置*/
uint16_t HP_x = 600;
uint16_t HP_y = 750;


uint8_t PUSH_DELAY_DYNAMIC=40;
uint8_t PUSH_DELAY_STATIC=80;
void referee_unpack_task(void const * argument)
{
	vTaskDelay(200);
	while(1)
	{
		Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);	
#ifndef	Robot_ID_Current
		Robot_ID_Current = Game_Robot_State.robot_id;
#endif
    UI_Capacitance=cap_data.cap_per * 100;
		UI_Chassis_Follow_Gimbal_Angle = (motor_gimbal.ecd-chassis_ctrl.follow_gimbal_zero) / 1303.7973f;
		UI_Chassis_Fall = (chassis_ctrl.ctrl_mode == MODE_STOP) || (chassis_ctrl.ctrl_mode == MODE_WEAK);
		vTaskDelay(10);
	}
}

void referee_usart_task(void const * argument)
{
	float Capacitance_X;
	
	/* 裁判系统初始化 */
	vTaskDelay(300);
	
	/* new UI */
	while(1)
	{
		UI_Flash_Flag = chassis_ctrl.flag_flash_ui;
//静态UI预绘制 中央标尺1
		if(UI_Flash_Flag)
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "001", UI_Graph_Add, 0, UI_Color_Green, 1,  840,   y01,  920,   y01); //第一行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[1], "002", UI_Graph_Add, 0, UI_Color_Green, 1,  950,   y01,  970,   y01); //第一行十字横
			UI_Draw_Line(&UI_Graph7.Graphic[2], "003", UI_Graph_Add, 0, UI_Color_Green, 1, 1000,   y01, 1080,   y01); //第一行右横线
			UI_Draw_Line(&UI_Graph7.Graphic[3], "004", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y01-10,  960,y01+10); //第一行十字竖
			UI_Draw_Line(&UI_Graph7.Graphic[4], "005", UI_Graph_Add, 0, UI_Color_Green, 1,  870,   y02,  930,   y02); //第二行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[5], "006", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y02,  960,   y02); //第二行中心点
			UI_Draw_Line(&UI_Graph7.Graphic[6], "007", UI_Graph_Add, 0, UI_Color_Green, 1,  989,   y02,  1049,  y02); //第二行左横线
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			vTaskDelay(PUSH_DELAY_STATIC);
		}
//静态UI预绘制 中央标尺2
		if(UI_Flash_Flag)
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "008", UI_Graph_Add, 0, UI_Color_Green, 1,  900,   y03,  940,   y03); //第三行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[1], "009", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y03,  960,   y03); //第三行中心点
			UI_Draw_Line(&UI_Graph7.Graphic[2], "010", UI_Graph_Add, 0, UI_Color_Green, 1,  980,   y03, 1020,   y03); //第三行右横线
			UI_Draw_Line(&UI_Graph7.Graphic[3], "011", UI_Graph_Add, 0, UI_Color_Green, 1,  930,   y04,  950,   y04); //第四行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[4], "012", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y04,  960,   y04); //第四行中心点
			UI_Draw_Line(&UI_Graph7.Graphic[5], "013", UI_Graph_Add, 0, UI_Color_Green, 1,  970,   y04,  990,   y04); //第四行右横线
			UI_Draw_Line(&UI_Graph7.Graphic[6], "014", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y04-10,  960,y04-30); //第四行下竖线
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			vTaskDelay(PUSH_DELAY_STATIC);
		}
//静态UI预绘制 车体宽度线
		if(UI_Flash_Flag)
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "101", UI_Graph_Add, 1, UI_Color_Yellow, 2,  735,  180,  615,  0);
			UI_Draw_Line(&UI_Graph7.Graphic[1], "102", UI_Graph_Add, 1, UI_Color_Yellow, 2,  1920-735,  180,  1920-615,  0);
			UI_Draw_Line(&UI_Graph7.Graphic[2], "103", UI_Graph_Add, 1, UI_Color_Yellow, 2,  600, 345, 540, 250);
			UI_Draw_Line(&UI_Graph7.Graphic[3], "104", UI_Graph_Add, 1, UI_Color_Yellow, 2,  1920-600, 345, 1920-540, 250);
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			vTaskDelay(PUSH_DELAY_STATIC);
		}
		
//动态UI预绘制 底盘方向显示
		if(UI_Flash_Flag)
		{
			UI_Draw_Circle (&UI_Graph7.Graphic[0], "204", UI_Graph_Add, 2, UI_Color_Yellow, 3,960,100,30);   //底盘圆
			UI_Draw_Line   (&UI_Graph7.Graphic[1], "205", UI_Graph_Add, 2, UI_Color_Orange, 5	, 960-70*arm_cos_f32(UI_Chassis_Follow_Gimbal_Angle)
																																												, 100+70*arm_sin_f32(UI_Chassis_Follow_Gimbal_Angle)
																																												, 960+70*arm_cos_f32(UI_Chassis_Follow_Gimbal_Angle)
																																												, 100-70*arm_sin_f32(UI_Chassis_Follow_Gimbal_Angle));
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);                                                                                                                                                                                                                                                                            
			vTaskDelay(PUSH_DELAY_STATIC);
		}  
//动态UI预绘制 图形
		if(UI_Flash_Flag)
		{
			UI_Draw_Float (&UI_Graph7.Graphic[0], "201", UI_Graph_Add, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, 0.000f);   //Pith轴角度
			UI_Draw_Line  (&UI_Graph7.Graphic[1], "202", UI_Graph_Add, 2, UI_Color_Orange, 20, 960-200, 200, 960+200, 200);      //电容容量
			UI_Draw_Int   (&UI_Graph7.Graphic[2], "211", UI_Graph_Add, 2, UI_Color_Green , 22, 3, 1160,200,100);
			UI_Draw_Line  (&UI_Graph7.Graphic[3], "212", UI_Graph_Add, 2, UI_Color_Green , 2 , 960-300, 600, 960+300, 600);
			UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Add, 2, UI_Color_White , 20, 0, 4, HP_x+15, HP_y, 0);
			UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Add, 2, UI_Color_White , 20, 0, 4, HP_x+45, HP_y, 0);
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			vTaskDelay(PUSH_DELAY_STATIC);
		}
//动态UI预绘制 字符串1
		if(UI_Flash_Flag)
		{
			UI_Draw_String(&UI_String.String,"203", UI_Graph_Add, 2, UI_Color_Black,  22, 8, 3,  400, 632, "Fric OFF"); //摩擦轮是否开启
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			vTaskDelay(PUSH_DELAY_STATIC);
		}		
//动态UI预绘制 字符串-底盘倒地
		if(UI_Flash_Flag)
		{
			if(UI_Chassis_Fall)UI_Draw_String(&UI_String1.String, "300", UI_Graph_Add, 3, UI_Color_Yellow,  22, 5, 3,  500, 700, "FALL\n"); 
			else UI_Draw_String(&UI_String1.String, "300", UI_Graph_Add, 3, UI_Color_White,  22, 5, 3,  500, 700, "SAFE\n"); 
			UI_PushUp_String(&UI_String1, Robot_ID_Current);
			vTaskDelay(PUSH_DELAY_STATIC);
		}

////////////////////////////////////////////////////////////////////////////////////
//动态UI更新 底盘方向显示         
		if(!UI_Flash_Flag)
		{
			UI_Draw_Circle (&UI_Graph7.Graphic[0], "204", UI_Graph_Change, 2, UI_Color_Yellow, 3,960,100,25);   //底盘圆
			UI_Draw_Line   (&UI_Graph7.Graphic[1], "205", UI_Graph_Change, 2, UI_Color_Orange, 5	, 960-70*arm_cos_f32(UI_Chassis_Follow_Gimbal_Angle)
																																												, 100+70*arm_sin_f32(UI_Chassis_Follow_Gimbal_Angle)
																																												, 960+70*arm_cos_f32(UI_Chassis_Follow_Gimbal_Angle)
																																												, 100-70*arm_sin_f32(UI_Chassis_Follow_Gimbal_Angle));                                                                                                                                                                                                                                                                        
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);                                                                                                                                                                                                                                                                                                                                                                                               
			vTaskDelay(PUSH_DELAY_DYNAMIC);
		}
		
//动态UI更新 图形
		if(!UI_Flash_Flag && UI_Push_Cnt % 3 == 0)
		{
			/* Pitch轴当前角度 */
			UI_Draw_Float(&UI_Graph7.Graphic[0], "201", UI_Graph_Change, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, UI_Gimbal_Pitch);
			
			/* 超级电容容量 */
			UI_Capacitance = Max(UI_Capacitance, 5);
			Capacitance_X  = 760.0f + 4.0f * UI_Capacitance;
			if(50 < UI_Capacitance && UI_Capacitance <= 100) 
			{
			 UI_Draw_Line(&UI_Graph7.Graphic[1], "202", UI_Graph_Change, 2, UI_Color_Green , 20,760 , 200, Capacitance_X, 200);
			 UI_Draw_Int   (&UI_Graph7.Graphic[2], "211", UI_Graph_Change, 2, UI_Color_Green , 22, 3, 1160,200,UI_Capacitance);
			}
			if(35 < UI_Capacitance && UI_Capacitance <=  50) 
			{
			 UI_Draw_Line(&UI_Graph7.Graphic[1], "202", UI_Graph_Change, 2, UI_Color_Yellow, 20,760 , 200, Capacitance_X, 200);
			 UI_Draw_Int   (&UI_Graph7.Graphic[2], "211", UI_Graph_Change, 2, UI_Color_Yellow , 22, 3, 1160,200,UI_Capacitance);
			}
			if(0  < UI_Capacitance && UI_Capacitance <=  35) 
			{
			 UI_Draw_Line(&UI_Graph7.Graphic[1], "202", UI_Graph_Change, 2, UI_Color_Orange, 20,760 , 200, Capacitance_X, 200);
			 UI_Draw_Int   (&UI_Graph7.Graphic[2], "211", UI_Graph_Change, 2, UI_Color_Orange , 22, 3, 1160,200,UI_Capacitance);
			}
//			if(AutoAim_Data_Receive .Aimed_ID !=0)
//				UI_Draw_Line   (&UI_Graph7.Graphic[4], "213", UI_Graph_Change, 2, UI_Color_Green  , 20,1200,200,1220,200);
//			else
//				UI_Draw_Line   (&UI_Graph7.Graphic[4], "213", UI_Graph_Change, 2, UI_Color_White , 20,1400,200,1420,200);
			UI_Draw_Line  (&UI_Graph7.Graphic[3], "212", UI_Graph_Change, 2, UI_Color_Green , 1 , 960-300.0f*arm_cos_f32(UI_Gimbal_Roll/180.0f*3.14159f) , 600-300*arm_sin_f32(UI_Gimbal_Roll/180.0f*3.14159f), 960+300*arm_cos_f32(UI_Gimbal_Roll/180.0f*3.14159f), 600+300*arm_sin_f32(UI_Gimbal_Roll/180.0f*3.14159f));
      
			if(Robot_ID_Current>=100)//我方为蓝方
			{
				switch(AutoAim_Data_Receive.Aimed_ID%9)
				{ 
					
					case 0:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_White  , 20, 0, 4, HP_x+15, HP_y, 0);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_White , 20, 0, 4, HP_x+45, HP_y, 0);
					break ;
					case 1:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_Main   , 20, 0, 4, HP_x+15, HP_y, 1);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_Orange , 20, 0, 4, HP_x+45, HP_y, Game_Robot_HP.red_1_robot_HP );
					break ;
					case 2:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_Main , 20, 0, 4, HP_x+15, HP_y, 2);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_Orange , 20, 0, 4, HP_x+45, HP_y, Game_Robot_HP.red_2_robot_HP );
					break ;
					case 3:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_Main  , 20, 0, 4, HP_x+15, HP_y, 3);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_Orange  , 20, 0, 4, HP_x+45, HP_y, Game_Robot_HP.red_3_robot_HP );
					break ;
					case 4:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_Main  , 20, 0, 4, HP_x+15, HP_y, 4);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_Orange  , 20, 0, 4, HP_x+45, HP_y, Game_Robot_HP.red_4_robot_HP );
					break ;
					case 5:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_Main  , 20, 0, 4, HP_x+15, HP_y, 5);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_Orange  , 20, 0, 4, HP_x+45, HP_y, Game_Robot_HP.red_5_robot_HP );
					break ;
					case 6:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_Main  , 20, 0, 4, HP_x+15, HP_y, 7);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_Orange  , 20, 0, 4, HP_x+45, HP_y, Game_Robot_HP.red_7_robot_HP );
					break ;
					case 7:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_Main  , 20, 0, 4, HP_x+15, HP_y, 8);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_White  , 20, 0, 4, HP_x+45, HP_y, 0);
					break ;
					case 8:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_Main  , 20, 0, 4, HP_x+15, HP_y, 9);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_Orange  , 20, 0, 4, HP_x+45, HP_y, Game_Robot_HP.red_base_HP);
					break ;
					
				}
			}
			else
			{
				switch(AutoAim_Data_Receive.Aimed_ID%9)
				{ 
					case 0:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_White  , 20, 0, 4, HP_x+15, HP_y, 0);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_White , 20, 0, 4, HP_x+45, HP_y, 0);
					break ;
					case 1:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_Main   , 20, 0, 4, HP_x+15, HP_y, 1);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_Orange  , 20, 0, 4, HP_x+45, HP_y, Game_Robot_HP.blue_1_robot_HP );
					break ;
					case 2:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_Main , 20, 0, 4, HP_x+15, HP_y, 2);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_Orange , 20, 0, 4, HP_x+45, HP_y, Game_Robot_HP.blue_2_robot_HP );
					break ;
					case 3:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_Main  , 20, 0, 4, HP_x+15, HP_y, 3);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_Orange , 20, 0, 4, HP_x+45, HP_y, Game_Robot_HP.blue_3_robot_HP );
					break ;
					case 4:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_Main  , 20, 0, 4, HP_x+15, HP_y, 4);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_Orange , 20, 0, 4, HP_x+45, HP_y, Game_Robot_HP.blue_4_robot_HP );
					break ;
					case 5:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_Main  , 20, 0, 4, HP_x+15, HP_y, 5);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_Orange , 20, 0, 4, HP_x+45, HP_y, Game_Robot_HP.blue_5_robot_HP );
					break ;
					case 6:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_Main  , 20, 0, 4, HP_x+15, HP_y, 7);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_Orange , 20, 0, 4, HP_x+45, HP_y, Game_Robot_HP.blue_7_robot_HP );
					break ;
					case 7:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_Main  , 20, 0, 4, HP_x+15, HP_y, 8);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_White , 20, 0, 4, HP_x+45, HP_y, 0);
					break ;
					case 8:
          UI_Draw_Float (&UI_Graph7.Graphic[5], "214", UI_Graph_Change, 2, UI_Color_Main  , 20, 0, 4, HP_x+15, HP_y, 9);
			    UI_Draw_Float (&UI_Graph7.Graphic[6], "215", UI_Graph_Change, 2, UI_Color_Orange , 20, 0, 4, HP_x+45, HP_y, Game_Robot_HP.blue_base_HP);
					break ;
				}
			}
			
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			vTaskDelay(PUSH_DELAY_DYNAMIC);
		}
		
//动态UI更新 字符串1
		if(!UI_Flash_Flag && UI_Push_Cnt % 3 == 1)
		{
			if(UI_fric_is_on == 1) 
			{
					if(Autoaim_Mode==0x00)
						{
							UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+21, 3,  100, 700, "Fric  ON\nAutoaim_mod  Normal \n");
						}
						else if(Autoaim_Mode==0x01)
						{
							UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+21, 3,  100, 700, "Fric  ON\nAutoaim_mod  XFu    \n");
						}
						else if(Autoaim_Mode==0x02)
						{
							UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+21, 3,  100, 700, "Fric  ON\nAutoaim_mod  DFu    \n");
						}
						else if(Autoaim_Mode==0x03)
						{
							UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+21, 3,  100, 700, "Fric  ON\nAutoaim_mod  Antitop\n");
						}	
			}
			if(UI_fric_is_on == 0) UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Black, 22, 8+4+9+8, 3,  100, 700, "Fric OFF\n   \n        \n       ");
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			vTaskDelay(PUSH_DELAY_DYNAMIC);
		}
		
//动态UI更新 字符串-底盘倒地		
		if(!UI_Flash_Flag && UI_Push_Cnt % 3 == 2)
		{
			if(UI_Chassis_Fall)UI_Draw_String(&UI_String1.String, "300", UI_Graph_Change, 3, UI_Color_Yellow,  22, 5, 3,  500, 700, "FALL\n"); 
			else UI_Draw_String(&UI_String1.String, "300", UI_Graph_Change, 3, UI_Color_White,  22, 5, 3,  500, 700, "SAFE\n"); 
			UI_PushUp_String(&UI_String1, Robot_ID_Current);
			vTaskDelay(PUSH_DELAY_DYNAMIC);
		}
/////////////////////////////////////////////////////////////////////////////////		
		UI_Flash_Flag=0;
		if(UI_Push_Cnt++ >= 300)UI_Push_Cnt=0;
	}
}

uint16_t this_time_rx_len = 0;
void USART6_IRQHandler_1(void)
{
		if(huart6.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    else if(USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
						fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[1], this_time_rx_len);
        }
        else
        {
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
						fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[1], this_time_rx_len);
        }
    }
}
