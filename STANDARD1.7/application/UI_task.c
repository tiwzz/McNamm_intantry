
#include "UI_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "RM_Cilent_UI.h"
#include "string.h"
#include "CAN_receive.h"
#include "stdio.h"
#include "remote_control.h"
#include "referee.h"
#include "gimbal_behaviour.h"
#include "shoot_task.h"
#include "chassis_task.h"
extern int anglesr;
extern ext_game_robot_state_t robot_state;
extern gimbal_behaviour_e gimbal_behaviour;
extern ext_bullet_remaining_t bullet_remaining_t;
extern chassis_move_t chassis_move;
extern int8_t R,QA,EA,turn_flags;
Graph_Data Aim[10];
String_Data strSHOOT,strCAP,strPILL;
void UI_task(void const *pvParameters)
{
	//剩余发弹量
			int16_t pill;
			char tmp1[30]={0},tmp2[30]={0},tmp3[30]={0};
			memset(&strCAP,0,sizeof(strCAP));
			memset(&strSHOOT,0,sizeof(strSHOOT));
			memset(&strPILL,0,sizeof(strPILL));
			for(int k=0;k<8;k++)
			{
				memset(&Aim[k],0,sizeof(Aim[k]));
			} 										//清空图形数据
			Line_Draw(&Aim[0],"AL1",UI_Graph_ADD,5,UI_Color_Green,3,960,300,960,540);
			Line_Draw(&Aim[1],"AL2",UI_Graph_ADD,5,UI_Color_Green,2,940,520,980,520);
			Line_Draw(&Aim[2],"AL3",UI_Graph_ADD,5,UI_Color_Black,3,930,478,990,478);
			Line_Draw(&Aim[3],"AL4",UI_Graph_ADD,5,UI_Color_Yellow,2,920,380,1000,380);
			Line_Draw(&Aim[8],"AL9",UI_Graph_ADD,6,UI_Color_Purplish_red,2,920,330,1000,330);
			//可通过宽度
			Line_Draw(&Aim[4],"AL5",UI_Graph_ADD,6,UI_Color_Green,2,440,0,627,443);
			Line_Draw(&Aim[5],"AL6",UI_Graph_ADD,6,UI_Color_Green,2,1480,0,1293,443);
			Line_Draw(&Aim[6],"AL7",UI_Graph_ADD,5,UI_Color_Orange,3,920,400,1000,400);
			Line_Draw(&Aim[7],"AL8",UI_Graph_ADD,7,UI_Color_Purplish_red,10,1700,600,1700,700);
			Circle_Draw(&Aim[9],"CL9",UI_Graph_ADD,5,UI_Color_Orange,5,1700,600,100);
			Char_Draw(&strCAP,"CAP",UI_Graph_ADD,8,UI_Color_Green,20,strlen(tmp1),2,860,100,tmp1);
			Char_Draw(&strSHOOT,"SHOOT",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp2),2,860,300,tmp2);
			Char_Draw(&strPILL,"PILL",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp3),2,10,760,tmp3);
			Char_ReFresh(strCAP);
			vTaskDelay(25);
			Char_ReFresh(strSHOOT);
			vTaskDelay(25);
			Char_ReFresh(strPILL);
			vTaskDelay(25);
			UI_ReFresh(7,Aim[0],Aim[1],Aim[2],Aim[3],Aim[4],Aim[5],Aim[6]);
			vTaskDelay(25);
			UI_ReFresh(5,Aim[7],Aim[9],Aim[8]);	
				
	while(1)
	{
		pill=bullet_remaining_t.bullet_remaining_num_17mm;
		if(rc_ctrl.key.v&KEY_PRESSED_OFFSET_Z)
		{
			Line_Draw(&Aim[0],"AL1",UI_Graph_ADD,5,UI_Color_Green,3,960,300,960,540);
			Line_Draw(&Aim[1],"AL2",UI_Graph_ADD,5,UI_Color_Green,2,940,520,980,520);
			Line_Draw(&Aim[2],"AL3",UI_Graph_ADD,5,UI_Color_Black,3,930,478,990,478);
			Line_Draw(&Aim[3],"AL4",UI_Graph_ADD,5,UI_Color_Yellow,2,920,380,1000,380);
			Line_Draw(&Aim[8],"AL9",UI_Graph_ADD,6,UI_Color_Purplish_red,2,920,430,1000,430);
			//可通过宽度
			Line_Draw(&Aim[4],"AL5",UI_Graph_ADD,6,UI_Color_Green,2,440,0,627,443);
			Line_Draw(&Aim[5],"AL6",UI_Graph_ADD,6,UI_Color_Green,2,1480,0,1293,443);
			Line_Draw(&Aim[6],"AL7",UI_Graph_ADD,5,UI_Color_Orange,3,920,400,1000,400);
			Line_Draw(&Aim[7],"AL8",UI_Graph_ADD,7,UI_Color_Purplish_red,10,1700,600,1700,700);
			Circle_Draw(&Aim[9],"CL9",UI_Graph_ADD,5,UI_Color_Orange,5,1700,600,100);
			Char_Draw(&strCAP,"CAP",UI_Graph_ADD,8,UI_Color_Green,20,strlen(tmp1),2,860,100,tmp1);
			Char_Draw(&strSHOOT,"SHOOT",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp2),2,860,300,tmp2);
			Char_Draw(&strPILL,"PILL",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp3),2,10,760,tmp3);
			Char_ReFresh(strCAP);
			vTaskDelay(25);
			Char_ReFresh(strSHOOT);
			vTaskDelay(25);
			Char_ReFresh(strPILL);
			vTaskDelay(25);
			UI_ReFresh(7,Aim[0],Aim[1],Aim[2],Aim[3],Aim[4],Aim[5],Aim[6]);
			vTaskDelay(25);
			UI_ReFresh(5,Aim[7],Aim[9],Aim[8]);	
		}

		sprintf(tmp1,"Cap:%.1f(%.2fV)",((get_cap.capvot/100.0-13.5)/10.5*100),get_cap.capvot/100.0);
		
		if(get_cap.capvot > 1800.0f)
				Char_Draw(&strCAP,"CAP",UI_Graph_Change,8,UI_Color_Green,20,strlen(tmp1),2,860,100,tmp1);
		else
				Char_Draw(&strCAP,"CAP",UI_Graph_Change,8,UI_Color_Orange,20,strlen(tmp1),2,860,100,tmp1);
		Char_ReFresh(strCAP);
		vTaskDelay(25);
		if(R)
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
			sprintf(tmp2,"SHOOT:ON");
			Char_Draw(&strSHOOT,"SHOOT",UI_Graph_Change,9,UI_Color_Orange,30,strlen(tmp2),2,860,300,tmp2);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
			sprintf(tmp2,"SHOOT:OFF");
			Char_Draw(&strSHOOT,"SHOOT",UI_Graph_Change,9,UI_Color_Green,30,strlen(tmp2),2,860,300,tmp2);
		}
		Char_ReFresh(strSHOOT);
		vTaskDelay(25);
		sprintf(tmp3,"PILL:%d",pill);
		Char_Draw(&strPILL,"PILL",UI_Graph_Change,9,UI_Color_Green,20,strlen(tmp3),2,10,760,tmp3);
		Char_ReFresh(strPILL);
		vTaskDelay(25);
		Line_Draw(&Aim[4],"AL5",UI_Graph_Change,6,UI_Color_Green,2,440-anglesr,0,627-anglesr,443);
		Line_Draw(&Aim[5],"AL6",UI_Graph_Change,6,UI_Color_Green,2,1480+anglesr,0,1293+anglesr,443);
		if(turn_flags){	
		Line_Draw(&Aim[7],"AL8",UI_Graph_Change,7,UI_Color_Purplish_red,10,1700,600,1700,500);
		}
		else
		Line_Draw(&Aim[7],"AL8",UI_Graph_Change,7,UI_Color_Green,10,1700,600,1700,700);	
		vTaskDelay(25);
		UI_ReFresh(5,Aim[4],Aim[5],Aim[7]);
		vTaskDelay(20);
	}
}






