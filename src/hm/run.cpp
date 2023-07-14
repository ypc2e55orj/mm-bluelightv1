/***********************************************************************/
/*                                                                     */
/*  FILE        :spi.c			                               */
/*  DATE        :Tue, Jun 08, 2017                                     */
/*  DESCRIPTION :SPI Program                                           */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
#include "math.h"
#include "sci.h"
#include "init.h"
#include "parameters.h"
#include "glob_var.h"
#include "mytypedef.h"
#include "interface.h"
#include "run.h"
#include "misc.h"

#include "../driver/motor.h"

#include <stdio.h>

extern unsigned int timer;
float r_adjust_len, l_adjust_len;
void straight(float len, float acc, float max_sp, float end_sp)
{
	char r_wall_check = 0, l_wall_check = 0, hosei_f = 0;
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	// 走行モードを直線にする
	run_mode = STRAIGHT_MODE;
	// 壁制御を有効にする
	con_wall.enable = true;
	// 目標距離をグローバル変数に代入する
	len_target = len;
	// 目標速度を設定
	end_speed = end_sp;
	// 加速度を設定
	accel = acc;
	// 最高速度を設定
	max_speed = max_sp;

	// モータ出力をON
	driver::motor::enable();

	if (end_speed != 0 && len == SECTION)
	{
		r_wall_check = sen_r.is_wall;
		l_wall_check = sen_l.is_wall;
	}

	if (end_speed == 0)
	{ // 最終的に停止する場合
		// 減速処理を始めるべき位置まで加速、定速区間を続行
		while (((len_target - 10) - len_mouse) > 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
			;
		// 減速処理開始
		accel = -acc; // 減速するために加速度を負の値にする
		while (len_mouse < len_target - 1)
		{ // 停止したい距離の少し手前まで継続
			// 一定速度まで減速したら最低駆動トルクで走行
			if (tar_speed <= MIN_SPEED)
			{ // 目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				tar_speed = MIN_SPEED;
			}
		}
		accel = 0;
		tar_speed = 0;
		// 速度が0以下になるまで逆転する
		while (speed >= 0.0)
			;
	}
	else
	{
		// 減速処理を始めるべき位置まで加速、定速区間を続行
		while (((len_target - 10) - len_mouse) > 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
		{
			if (len == SECTION)
			{
				if (!sen_r.is_wall && r_wall_check && !hosei_f)
				{
					len_mouse = (len_mouse + RIGHT_WALL_NOWALL) / 2;
					hosei_f = 1;
				}
				if (!sen_l.is_wall && l_wall_check && !hosei_f)
				{
					len_mouse = (len_mouse + LEFT_WALL_NOWALL) / 2;
					hosei_f = 1;
				}
			}
		}

		// 減速処理開始
		accel = -acc; // 減速するために加速度を負の値にする
		while (len_mouse < len_target + 5)
		{ // 停止したい距離の少し手前まで継続
			if (!sen_fr.is_wall && !sen_fl.is_wall && len_mouse > len_target)
			{
				break;
			}
			if (sen_fr.value > TH_RIGHT_90 && sen_fl.value > TH_LEFT_90 && len_mouse > len_target - 5)
			{
				break;
			}
			// 一定速度まで減速したら最低駆動トルクで走行
			if (tar_speed <= end_speed)
			{ // 目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				// tar_speed = end_speed;
			}
		}
	}
	// 加速度を0にする
	accel = 0;
	// 現在距離を0にリセット
	len_mouse = 0;
}

void straight_debug(float len, float acc, float max_sp, float end_sp)
{
	char r_wall_check = 0, l_wall_check = 0, hosei_f = 0;
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	// 走行モードを直線にする
	run_mode = STRAIGHT_MODE;
	// 壁制御を有効にする
	con_wall.enable = true;
	// 目標距離をグローバル変数に代入する
	len_target = len;
	// 目標速度を設定
	end_speed = end_sp;
	// 加速度を設定
	accel = acc;
	// 最高速度を設定
	max_speed = max_sp;

	// モータ出力をON
	driver::motor::enable();

	if (end_speed != 0 && len == SECTION)
	{
		r_wall_check = sen_r.is_wall;
		l_wall_check = sen_l.is_wall;
	}

	printf("len_target, len_mouse\n\r");
	if (end_speed == 0)
	{ // 最終的に停止する場合
		// 減速処理を始めるべき位置まで加速、定速区間を続行
		while (((len_target - 10) - len_mouse) > 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
		{
			printf("%f, %f\n\r", len_target, len_mouse);
		}
		// 減速処理開始
		accel = -acc; // 減速するために加速度を負の値にする
		while (len_mouse < len_target - 1)
		{
			printf("%f, %f\n\r", len_target, len_mouse);
			// 停止したい距離の少し手前まで継続
			// 一定速度まで減速したら最低駆動トルクで走行
			if (tar_speed <= MIN_SPEED)
			{ // 目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				tar_speed = MIN_SPEED;
			}
		}
		accel = 0;
		tar_speed = 0;
		// 速度が0以下になるまで逆転する
		while (speed >= 0.0)
		{
			printf("%f, %f\n\r", len_target, len_mouse);
		}
	}
	else
	{
		// 減速処理を始めるべき位置まで加速、定速区間を続行
		while (((len_target - 10) - len_mouse) > 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
		{
			if (len == SECTION)
			{
				if (!sen_r.is_wall && r_wall_check && !hosei_f)
				{
					len_mouse = (len_mouse + RIGHT_WALL_NOWALL) / 2;
					hosei_f = 1;
				}
				if (!sen_l.is_wall && l_wall_check && !hosei_f)
				{
					len_mouse = (len_mouse + LEFT_WALL_NOWALL) / 2;
					hosei_f = 1;
				}
			}
		}

		// 減速処理開始
		accel = -acc; // 減速するために加速度を負の値にする
		while (len_mouse < len_target + 5)
		{ // 停止したい距離の少し手前まで継続
			if (!sen_fr.is_wall && !sen_fl.is_wall && len_mouse > len_target)
			{
				break;
			}
			if (sen_fr.value > TH_RIGHT_90 && sen_fl.value > TH_LEFT_90 && len_mouse > len_target - 5)
			{
				break;
			}
			// 一定速度まで減速したら最低駆動トルクで走行
			if (tar_speed <= end_speed)
			{ // 目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				// tar_speed = end_speed;
			}
		}
	}
	// 加速度を0にする
	accel = 0;
	// 現在距離を0にリセット
	len_mouse = 0;
}

void turn(int deg, float ang_accel, float max_ang_velocity, short dir)
{
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	tar_degree = 0;

	float local_degree = 0;
	accel = 0;
	tar_speed = 0;
	tar_ang_vel = 0;
	// 走行モードをスラロームモードにする
	run_mode = TURN_MODE;

	// 回転方向定義
	TURN_DIR = dir;

	// 車体の現在角度を取得
	local_degree = degree;
	tar_degree = 0;

	// 角加速度、加速度、最高角速度設定
	driver::motor::enable();
	if (dir == LEFT)
	{
		ang_acc = ang_accel; // 角加速度を設定
		max_ang_vel = max_ang_velocity;
		max_degree = deg;
		while ((max_degree - (degree - local_degree)) * PI / 180.0 > (tar_ang_vel * tar_ang_vel / (2.0 * ang_acc)))
			;
	}
	else if (dir == RIGHT)
	{
		ang_acc = -ang_accel; // 角加速度を設定
		max_ang_vel = -max_ang_velocity;
		max_degree = -deg;
		while (-(float)(max_degree - (degree - local_degree)) * PI / 180.0 > (float)(tar_ang_vel * tar_ang_vel / (float)(2.0 * -ang_acc)))
			;
	}

	// 角減速区間に入るため、角加速度設定
	if (dir == LEFT)
	{
		ang_acc = -ang_accel; // 角加速度を設定
		// 減速区間走行
		while ((degree - local_degree) < max_degree)
		{
			if (tar_ang_vel < TURN_MIN_SPEED)
			{
				ang_acc = 0;
				tar_ang_vel = TURN_MIN_SPEED;
			}
		}

		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
	}
	else if (dir == RIGHT)
	{
		ang_acc = +ang_accel; // 角加速度を設定
		// 減速区間走行
		while ((degree - local_degree) > max_degree)
		{
			if (-tar_ang_vel < TURN_MIN_SPEED)
			{
				ang_acc = 0;
				tar_ang_vel = -TURN_MIN_SPEED;
			}
		}
		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
	}

	while (ang_vel >= 0.05 || ang_vel <= -0.05)
		;

	tar_ang_vel = 0;
	ang_acc = 0;
	// 現在距離を0にリセット
	len_mouse = 0;
}

void turn_debug(int deg, float ang_accel, float max_ang_velocity, short dir)
{
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	tar_degree = 0;

	float local_degree = 0;
	accel = 0;
	tar_speed = 0;
	tar_ang_vel = 0;
	// 走行モードをスラロームモードにする
	run_mode = TURN_MODE;

	// 回転方向定義
	TURN_DIR = dir;

	// 車体の現在角度を取得
	local_degree = degree;
	tar_degree = 0;

	// 角加速度、加速度、最高角速度設定
	driver::motor::enable();
	if (dir == LEFT)
	{
		ang_acc = ang_accel; // 角加速度を設定
		max_ang_vel = max_ang_velocity;
		max_degree = deg;
		while ((max_degree - (degree - local_degree)) * PI / 180.0 > (tar_ang_vel * tar_ang_vel / (2.0 * ang_acc)))
		{
			printf("%f\n\r",(degree - local_degree));
		}
	}
	else if (dir == RIGHT)
	{
		ang_acc = -ang_accel; // 角加速度を設定
		max_ang_vel = -max_ang_velocity;
		max_degree = -deg;
		while (-(float)(max_degree - (degree - local_degree)) * PI / 180.0 > (float)(tar_ang_vel * tar_ang_vel / (float)(2.0 * -ang_acc)))
		{
			printf("%f\n\r", (degree - local_degree));
		}
	}

	// 角減速区間に入るため、角加速度設定
	if (dir == LEFT)
	{
		ang_acc = -ang_accel; // 角加速度を設定
		// 減速区間走行
		while ((degree - local_degree) < max_degree)
		{
			printf("%f\n\r", (degree - local_degree));
			if (tar_ang_vel < TURN_MIN_SPEED)
			{
				ang_acc = 0;
				tar_ang_vel = TURN_MIN_SPEED;
			}
		}

		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
	}
	else if (dir == RIGHT)
	{
		ang_acc = +ang_accel; // 角加速度を設定
		// 減速区間走行
		while ((degree - local_degree) > max_degree)
		{
			printf("%f\n\r", (degree - local_degree));
			if (-tar_ang_vel < TURN_MIN_SPEED)
			{
				ang_acc = 0;
				tar_ang_vel = -TURN_MIN_SPEED;
			}
		}
		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
	}

	while (ang_vel >= 0.05 || ang_vel <= -0.05)
	{
		printf("%f\n\r", (degree - local_degree));
	}

	tar_ang_vel = 0;
	ang_acc = 0;
	// 現在距離を0にリセット
	len_mouse = 0;
}

void slalom_turn(int deg, float ang_accel, float max_ang_velocity, short dir, float end_sp)
{
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	tar_degree = 0;

	float local_degree = 0;
	accel = 0;
	tar_speed = end_sp;
	tar_ang_vel = 0;
	// 走行モードをスラロームモードにする
	run_mode = TURN_MODE;

	// 回転方向定義
	TURN_DIR = dir;

	// 車体の現在角度を取得
	local_degree = degree;
	tar_degree = 0;

	// 角加速度、加速度、最高角速度設定
	driver::motor::enable();
	if (dir == LEFT)
	{
		ang_acc = ang_accel; // 角加速度を設定
		max_ang_vel = max_ang_velocity;
		max_degree = deg;
		while ((max_degree - (degree - local_degree)) * PI / 180.0 > (tar_ang_vel * tar_ang_vel / (2.0 * ang_acc)))
			;
	}
	else if (dir == RIGHT)
	{
		ang_acc = -ang_accel; // 角加速度を設定
		max_ang_vel = -max_ang_velocity;
		max_degree = -deg;
		while (-(float)(max_degree - (degree - local_degree)) * PI / 180.0 > (float)(tar_ang_vel * tar_ang_vel / (float)(2.0 * -ang_acc)))
			;
	}

	// BEEP();
	// 角減速区間に入るため、角加速度設定
	driver::motor::enable();
	if (dir == LEFT)
	{
		ang_acc = -ang_accel; // 角加速度を設定
		// 減速区間走行
		while ((degree - local_degree) < max_degree)
		{
			if (tar_ang_vel < TURN_MIN_SPEED)
			{
				ang_acc = 0;
				tar_ang_vel = TURN_MIN_SPEED;
			}
		}

		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
	}
	else if (dir == RIGHT)
	{
		ang_acc = +ang_accel; // 角加速度を設定
		// 減速区間走行
		while ((degree - local_degree) > max_degree)
		{
			if (-tar_ang_vel < TURN_MIN_SPEED)
			{
				ang_acc = 0;
				tar_ang_vel = -TURN_MIN_SPEED;
			}
		}
		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
	}

	while (ang_vel >= 0.05 || ang_vel <= -0.05)
		;

	tar_ang_vel = 0;
	ang_acc = 0;
	// 現在距離を0にリセット
	len_mouse = 0;
}

/**
 * 位置補正関数
 *
 */
// 前壁&距離センサを用いて位置補正
void adjust_fwall(void)
{
	// 割り込みで角度制御を行う
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	// 走行モードを前壁制御モードにする
	run_mode = F_WALL_MODE;
	// 前壁制御を有効にする
	con_fwall.enable = true;
	// 目標速度を設定
	end_speed = 0;
	// 加速度を設定
	accel = 0;
	// 最高速度を設定
	max_speed = SEARCH_SPEED;

	driver::motor::enable();

	while (true)
	{
		short now_distance = sen_fr.value + sen_fl.value;
		short target_distance = sen_fr.ref + sen_fl.value;
		if ((now_distance - target_distance) < -10)
		{
			accel = SEARCH_ACCEL;
		}
		else if ((now_distance - target_distance) > 10)
		{
			accel = -SEARCH_ACCEL;
		}
		else
		{
			accel = 0;
			tar_speed = 0;
			// 角度合わせを待つ
			if (-10 < con_fwall.error && con_fwall.error < 5)
				break;
		}
	}
	max_speed = 0;
	tar_ang_vel = 0;
	ang_acc = 0;
	// 現在距離を0にリセット
	len_mouse = 0;
}

// 物理位置補正
void back(float len, float acc, float max_sp, float end_sp)
{
	unsigned int start_timer; // タイヤがロックした時の対策
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	len_mouse = 0; // 距離の初期化
	run_mode = STRAIGHT_MODE;
	con_wall.enable = false; // 壁制御を無効にする
	len_target = len;				 // lenの値はマイナスが入っている
	end_speed = end_sp;
	accel = -acc; // 速度をマイナスにするとバックする
	max_speed = max_sp;
	driver::motor::enable();
	while (((len_target - 10) - len_mouse) > 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
		;
	accel = 1.0; // マイナスの速度を0にするためプラスにしている
	start_timer = timer;
	while (len_mouse > (len_target + 1))
	{
		if (tar_speed >= -1 * MIN_SPEED)
		{ // MIN_SPEEDはプラスの値なので、-1を掛けてマイナスにする
			accel = 0;
			tar_speed = -1 * MIN_SPEED;
		}
		if ((timer - start_timer) > 500)
		{ // 0.5秒経ってもwhileを抜け出せない時の処置
			break;
		}
	}
	accel = 0;
	tar_speed = 0;
	len_mouse = 0;
}

#define turn_norm(deg, dir) turn(deg, TURN_ACCEL, TURN_SPEED, dir);
void adjust_turn()
{
	// 生成した地図をもとに壁を検出
	switch (mypos.dir)
	{
	case north:
		if (wall[mypos.x][mypos.y].north == WALL)
		{
			adjust_fwall();
		}

		if (wall[mypos.x][mypos.y].east == WALL)
		{
			turn_norm(90, RIGHT);
			adjust_fwall();
			turn_norm(90, RIGHT);
		}
		else if (wall[mypos.x][mypos.y].west == WALL)
		{
			turn_norm(90, LEFT);
			adjust_fwall();
			turn_norm(90, LEFT);
		}
		else
		{
			turn_norm(180, RIGHT);
		}
		break;
	case east:
		if (wall[mypos.x][mypos.y].east == WALL)
		{
			adjust_fwall();
		}

		if (wall[mypos.x][mypos.y].south == WALL)
		{
			turn_norm(90, RIGHT);
			adjust_fwall();
			turn_norm(90, RIGHT);
		}
		else if (wall[mypos.x][mypos.y].north == WALL)
		{
			turn_norm(90, LEFT);
			adjust_fwall();
			turn_norm(90, LEFT);
		}
		else
		{
			turn_norm(180, RIGHT);
		}
		break;
	case west:
		if (wall[mypos.x][mypos.y].west == WALL)
		{
			adjust_fwall();
		}

		if (wall[mypos.x][mypos.y].north == WALL)
		{
			turn_norm(90, RIGHT);
			adjust_fwall();
			turn_norm(90, RIGHT);
		}
		else if (wall[mypos.x][mypos.y].south == WALL)
		{
			turn_norm(90, LEFT);
			adjust_fwall();
			turn_norm(90, LEFT);
		}
		else
		{
			turn_norm(180, RIGHT);
		}
		break;
	case south:
		if (wall[mypos.x][mypos.y].south == WALL)
		{
			adjust_fwall();
		}

		if (wall[mypos.x][mypos.y].west == WALL)
		{
			turn_norm(90, RIGHT);
			adjust_fwall();
			turn_norm(90, RIGHT);
		}
		else if (wall[mypos.x][mypos.y].east == WALL)
		{
			turn_norm(90, LEFT);
			adjust_fwall();
			turn_norm(90, LEFT);
		}
		else
		{
			turn_norm(180, RIGHT);
		}
		break;
	}
}

void start_position(void)
{
	straight(20, SEARCH_ACCEL, SEARCH_SPEED, 0);
	turn_norm(90, RIGHT);
	adjust_fwall();
	turn_norm(90, RIGHT);
	adjust_fwall();
	turn_norm(90, RIGHT);
	adjust_fwall();
	turn_norm(90, RIGHT);
}
