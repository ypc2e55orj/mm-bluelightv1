/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :2017/6/27	                                       */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :RX631 48P                                             */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
#include "math.h"
#include "sci.h"
#include "parameters.h"
#include "glob_var.h"
#include "run.h"
#include "interface.h"
#include "DataFlash.h"
#include "fast.h"
#include "search.h"
#include "misc.h"
#include "adjust.h"
#include "init.h"

#include "../driver/motor.h"

void adjust(void)
{

	int i = 0;
	int flash_time = 0;
	char flash = 0;
	short ad_mode = 1;

	while (1)
	{
		switch (ad_mode)
		{

		case 1:
			/****************************************
			 *MODE LED STATE				*
			 *					*
			 *	D3	D4	D5	D6	*
			 *	O	X	X	X	*
			 *					*
			 *****************************************/

			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				ANIMATE();
				gyro_get_ref();
				ANIMATE();
				run_mode = TURN_MODE;
				I_tar_ang_vel = 0;
				tar_ang_vel = 0;
				ang_vel = 0;
				driver::motor::enable();
				while (1)
				{
					// A/D sensor
					SCI_printf("sen_r.value: %d\n\r", sen_r.value);
					SCI_printf("sen_l.value: %d\n\r", sen_l.value);
					SCI_printf("sen_fr.value: %d\n\r", sen_fr.value);
					SCI_printf("sen_fl.value: %d\n\r", sen_fl.value);
					SCI_printf("sen_r.th_wall: %d\n\r", sen_r.th_wall);
					SCI_printf("sen_l.th_wall: %d\n\r", sen_l.th_wall);
					SCI_printf("sen_fr.th_wall: %d\n\r", sen_fr.th_wall);
					SCI_printf("sen_fl.th_wall: %d\n\r", sen_fl.th_wall);
					SCI_printf("con_wall.omega: %f\n\r", con_wall.omega);
					SCI_printf("con_fwall.omega: %f\n\r", con_fwall.omega);
					// motor
					SCI_printf("accel: %f\n\r", accel);
					SCI_printf("tar_speed: %f\n\r", tar_speed);
					SCI_printf("I_tar_speed: %f\n\r", I_tar_speed);
					SCI_printf("max_speed: %f\n\r", max_speed);
					SCI_printf("ang_acc: %f\n\r", ang_acc);
					SCI_printf("tar_ang_vel: %f\n\r", tar_ang_vel);
					SCI_printf("I_tar_ang_vel: %f\n\r", I_tar_ang_vel);
					SCI_printf("max_ang_vel: %f\n\r", max_ang_vel);
					SCI_printf("tar_degree: %f\n\r", tar_degree);
					SCI_printf("max_degree: %f\n\r", max_degree);
					SCI_printf("speed_r: %f\n\r", speed_r);
					SCI_printf("speed_l: %f\n\r", speed_l);
					SCI_printf("V_bat: %f\n\r", V_bat);
					SCI_printf("V_r: %f\n\r", V_r);
					SCI_printf("V_l: %f\n\r", V_l);
					SCI_printf("Duty_r: %f\n\r", Duty_r);
					SCI_printf("Duty_l: %f\n\r", Duty_l);
					// gyro
					SCI_printf("degree: %f\n\r", degree);
					;
					SCI_printf("gyro: %f\n\r", ang_vel);
					// encoder
					SCI_printf("locate_r: %d\n\r", locate_r);
					SCI_printf("locate_l: %d\n\r", locate_l);

					wait_ms(100);
					// 画面クリアシーケンス
					SCI_printf("\x1b[2J");	 // クリアスクリーン[CLS]
					SCI_printf("\x1b[0;0H"); // カーソルを0,0に移動
				}
			}

			break;

		case 2:
			/****************************************
			 *MODE LED STATE				*
			 *					*
			 *	D3	D4	D5	D6	*
			 *	X	O	X	X	*
			 *					*
			 *****************************************/

			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				ANIMATE();
				gyro_get_ref();
				ANIMATE();
				log_flag = 1;
				log_timer = 0;
				len_mouse = 0;
				con_wall.enable = true;
				straight(SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
				log_flag = 0;
				driver::motor::disable();
				driver::motor::brake();
				ANIMATE();
				wait_ms(500);
			}

			break;

		case 3:
			/****************************************
			 *MODE LED STATE				*
			 *					*
			 *	D3	D4	D5	D6	*
			 *	O	O	X	X	*
			 *					*
			 *****************************************/
			// マップの表示
			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				ANIMATE();
				gyro_get_ref();
				ANIMATE();
				log_flag = 1;
				log_timer = 0;
				log_flag = 0;
				turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
				driver::motor::disable();
				driver::motor::brake();
				ANIMATE();
				wait_ms(500);
			}

			break;

		case 4:
			/****************************************
			 *MODE LED STATE				*
			 *					*
			 *	D3	D4	D5	D6	*
			 *	X	X	O	X	*
			 *					*
			 *****************************************/

			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				ANIMATE();
				wait_ms(500);
				ANIMATE();
				log_flag = 1;
				log_timer = 0;
				len_mouse = 0;
				straight(SLA_SECTION_PRE,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
				slalom_turn(90,SLA_ACCEL,SLA_SPEED,LEFT,SEARCH_SPEED);				//左に曲がって
				straight(SLA_SECTION_POST,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
				driver::motor::disable();
				driver::motor::brake();
				log_flag = 1;
				log_timer = 0;
				len_mouse = 0;
				ANIMATE();
				wait_ms(500);

			}
			break;

		case 5:
			/****************************************
			 *MODE LED STATE				*
			 *					*
			 *	D3	D4	D5	D6	*
			 *	O	X	O	X	*
			 *					*
			 *****************************************/

			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				ANIMATE();
				gyro_get_ref();
				ANIMATE();
				log_flag = 1;
				log_timer = 0;
				while (true)
				{
					adjust_fwall();
					ANIMATE();
				}
				log_flag = 0;
				driver::motor::disable();
				driver::motor::brake();
				ANIMATE();
				wait_ms(500);
			}

			break;

		case 6:
			/****************************************
			 *MODE LED STATE				*
			 *					*
			 *	D3	D4	D5	D6	*
			 *	X	O	O	X	*
			 *					*
			 *****************************************/
			// マップ表示
			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				ANIMATE();
				map_copy();
				map_view();
				wait_ms(500);
			}

			break;

		case 7:
			/****************************************
			 *MODE LED STATE				*
			 *					*
			 *	D3	D4	D5	D6	*
			 *	O	O	O	X	*
			 *					*
			 *****************************************/
			// ログ出力
			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				ANIMATE();
				SCI_printf("time[msec],len_mouse[mm],tar_speed[mm/s],speed[mm/s],Duty_R[%%],Duty_L[%%],V_battery[mV],tar_degree[deg*10],degree[deg*10],tar_ang_vel[1000*rad/s],ang_vel[1000*rad/s],I_tar_ang_vel[rad],ang_acc[1000*rad/ss]\n\r");
				for (i = 0; i < LOG_CNT; i++)
				{

					SCI_printf("%d,", i);										 // time[msec]
					SCI_printf("%d,", log_buffer[0][i]);		 // len_mouse[mm]
					SCI_printf("%d,", log_buffer[1][i]);		 // tar_speed[mm/s]
					SCI_printf("%d,", log_buffer[2][i]);		 // speed[mm/s]
					SCI_printf("%d,", log_buffer[3][i]);		 // Duty_R[%]
					SCI_printf("%d,", log_buffer[4][i]);		 // Duty_L[%]
					SCI_printf("%d,", log_buffer[5][i]);		 // V_battery[mV]
					SCI_printf("%d,", log_buffer[6][i]);		 // tar_degree[deg*10]
					SCI_printf("%d,", log_buffer[7][i]);		 // degree[deg*10]
					SCI_printf("%d,", log_buffer[8][i]);		 // tar_ang_vel[1000*rad/s]
					SCI_printf("%d,", log_buffer[9][i]);		 // ang_vel[1000*rad/s]
					SCI_printf("%d,", log_buffer[10][i]);		 // I_tar_ang_vel[rad]
					SCI_printf("%d\n\r", log_buffer[11][i]); // ang_acc[rad]
				}
				wait_ms(500);
			}
			break;
		// mode0‾7以外の場合。何もしない。
		default:
			break;
		}

		// モード切り替え用処理
		if (speed > 0.1)
		{
			if (ad_mode == 7)
			{
				ad_mode = 1;
			}
			else
			{
				ad_mode++;
			}
			wait_ms(1);
			BEEP();
		}

		if (speed < -0.1)
		{
			if (ad_mode == 1)
			{
				ad_mode = 7;
			}
			else
			{
				ad_mode--;
			}
			wait_ms(1);

			BEEP();
		}
		if (flash_time > 0x00FF)
		{
			flash_time = 0;
			if (flash == 0x08)
			{
				flash = 0x00;
			}
			else
			{
				flash = 0x08;
			}
		}

		flash_time++;
		LED(ad_mode | flash);
	}
}
