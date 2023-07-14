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
#include "init.h"
#include "parameters.h"
#include "glob_var.h"
#include "run.h"
#include "interface.h"
#include "DataFlash.h"
#include "fast.h"
#include "search.h"
#include "misc.h"
#include "adjust.h"
#include "HM_StarterKit.h"

#include <iostream>

#include "../driver/motor.h"

void search_mode(bool is_slalom)
{
	degree = 0;
	timer = 0;
	log_timer = 0;
	gyro_get_ref();
	BEEP();
	mypos.x = mypos.y = 0; // 座標を初期化
	mypos.dir = north;		 // 方角を初期化
	log_flag = 1;
	log_timer = 0;
	if (is_slalom)
	{
		search_adachi_sla(GOAL_X, GOAL_Y); // ゴールまで足立法
	}
	else
	{
		search_adachi(GOAL_X, GOAL_Y);
	}
	adjust_turn();																						 // ゴールしたら180度回転する
	mypos.dir = static_cast<t_direction>((mypos.dir + 6) % 4); // 方角を更新
	map_write();
	BEEP();
	wait_ms(100);
	BEEP(); // ゴールしたことをアピール
	wait_ms(100);
	BEEP(); // ゴールしたことをアピール
	if (is_slalom)
	{
		search_adachi_sla(0, 0); // ゴールまで足立法
	}
	else
	{
		search_adachi(0, 0);
	}
	adjust_turn(); // 帰ってきたら180度回転
	driver::motor::brake();
	driver::motor::disable();
	map_write();
	log_flag = 0;
}

void fast_mode(bool is_slalom)
{
	map_copy();
	degree = 0;
	timer = 0;
	gyro_get_ref();
	BEEP();
	mypos.x = mypos.y = 0; // 座標を初期化
	mypos.dir = north;		 // 方角を初期化
	log_flag = 1;
	log_timer = 0;
	if (is_slalom)
	{
		fast_run_sla(GOAL_X, GOAL_Y); // ゴールまで足立法
	}
	else
	{
		fast_run(GOAL_X, GOAL_Y); // ゴールまで足立法
	}
	adjust_turn();																						 // ゴールしたら180度回転する
	mypos.dir = static_cast<t_direction>((mypos.dir + 6) % 4); // 方角を更新
	map_write();
	BEEP();
	wait_ms(100);
	BEEP(); // ゴールしたことをアピール
	wait_ms(100);
	BEEP(); // ゴールしたことをアピール
	if (is_slalom)
	{
		search_adachi_sla(0, 0); // スタート地点まで足立法で帰ってくる
	}
	else
	{
		search_adachi(0, 0);
	}
	adjust_turn();
	driver::motor::brake();
	driver::motor::disable();
	map_write();
	log_flag = 0;
}

void HM_StarterKit(void)
{

	unsigned long i = 0;

	std::cout << "HMStarterKit main()" << std::endl;

	init_all();
	// ブザー
	BEEP();
	// 最初は0しておく
	speed_r = 0;
	speed_l = 0;

	// 起動時のログはとらない
	log_flag = 0;
	short mode = 1;
	while (1)
	{
		I_tar_ang_vel = 0;
		I_ang_vel = 0;
		I_tar_speed = 0;
		I_speed = 0;

		switch (mode)
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
				BEEP();
				wait_ms(1000);
				BEEP();
				start_position();
				BEEP();
				search_mode(true);
				BEEP();
				for (i = 0; i < 4; i++)
				{
					wait_ms(2500);
					BEEP();
					fast_mode(true);
					BEEP();
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
				BEEP();
				wait_ms(1000);
				BEEP();
				start_position();
				BEEP();
				search_mode(true);
				BEEP();
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

			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				BEEP();
				wait_ms(1000);
				BEEP();
				start_position();
				BEEP();
				fast_mode(true);
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
				BEEP();
				wait_ms(1000);
				BEEP();
				start_position();
				BEEP();
				search_mode(false);
				BEEP();
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
				BEEP();
				wait_ms(1000);
				BEEP();
				start_position();
				BEEP();
				fast_mode(false);
				BEEP();
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

			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				BEEP();

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

			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				BEEP();

				wait_ms(500);
			}

			break;

		case 8:
			/****************************************
			 *MODE LED STATE				*
			 *					*
			 *	D3	D4	D5	D6	*
			 *	X	X	X	O	*
			 *					*
			 *****************************************/

			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				BEEP();

				wait_ms(500);
			}

			break;

		case 9:
			/****************************************
			 *MODE LED STATE				*
			 *					*
			 *	D3	D4	D5	D6	*
			 *	O	X	X	O	*
			 *					*
			 *****************************************/

			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				BEEP();

				wait_ms(500);
			}

			break;

		case 10:
			/****************************************
			 *MODE LED STATE				*
			 *					*
			 *	D3	D4	D5	D6	*
			 *	X	O	X	O	*
			 *					*
			 *****************************************/

			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				BEEP();

				wait_ms(500);
			}

			break;

		case 11:
			/****************************************
			 *MODE LED STATE				*
			 *					*
			 *	D3	D4	D5	D6	*
			 *	O	O	X	O	*
			 *					*
			 *****************************************/

			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				BEEP();

				wait_ms(500);
			}

			break;

		case 12:
			/****************************************
			 *MODE LED STATE				*
			 *					*
			 *	D3	D4	D5	D6	*
			 *	X	X	O	O	*
			 *					*
			 *****************************************/

			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				BEEP();

				wait_ms(500);
			}

			break;

		case 13:
			/****************************************
			 *MODE LED STATE				*
			 *					*
			 *	D3	D4	D5	D6	*
			 *	O	X	O	O	*
			 *					*
			 *****************************************/

			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				BEEP();

				wait_ms(500);
			}

			break;

		case 14:
			/****************************************
			 *MODE LED STATE				*
			 *					*
			 *	D3	D4	D5	D6	*
			 *	X	O	O	O	*
			 *					*
			 *****************************************/

			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				BEEP();

				wait_ms(500);
			}

			break;

		case 15:
			/****************************************
			 *MODE LED STATE				*
			 *					*
			 *	D3	D4	D5	D6	*
			 *	O	O	O	O	*
			 *					*
			 *****************************************/

			// センサーの前に手をかざしてスタート
			if (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
			{
				BEEP();
				while (sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4)
					;
				adjust();

				BEEP();
				wait_ms(500);
			}

			break;

		// mode0‾15以外の場合。何もしない。
		default:
			break;
		}

		// モード切り替え用処理
		if (speed > 0.1)
		{
			if (mode == 15)
			{
				mode = 1;
			}
			else
			{
				mode++;
			}
			wait_ms(1);
			BEEP();
		}

		if (speed < -0.1)
		{
			if (mode == 1)
			{
				mode = 15;
			}
			else
			{
				mode--;
			}
			wait_ms(1);
			BEEP();
		}
		LED(mode);
	}
}
