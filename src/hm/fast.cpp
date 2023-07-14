#include "parameters.h"
#include "mytypedef.h"
#include "glob_var.h"
#include "run.h"
#include "interface.h"
#include "fast.h"


extern int get_nextdir(int x, int y, int mask, t_direction *dir);

void fast_run(int x, int y)
{
//引数の座標x,yに向かって最短走行する

	t_direction glob_nextdir;
	int straight_count=0;

	//現在の向きから、次に行くべき方向へ向く
	switch(get_nextdir(x,y,MASK_SECOND,&glob_nextdir))	//次に行く方向を戻り値とする関数を呼ぶ
	{
		case front:
			straight_count++;			//前向きだった場合は直線を走る距離を伸ばす
			break;

		case right:					//右に向く
			turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);				//右に曲がって
			straight_count = 1;
			break;

		case left:					//左に向く
			turn(90,TURN_ACCEL,TURN_SPEED,LEFT);				//左に曲がって
			straight_count = 1;
			break;

		case rear:					//後ろに向く
			turn(180,TURN_ACCEL,TURN_SPEED,LEFT);				//左に曲がって
			straight_count = 1;
			break;
	}

	mypos.dir = glob_nextdir;	//自分の向きを更新


	//向いた方向によって自分の座標を更新する
	switch(mypos.dir)
	{
		case north:
			mypos.y++;	//北を向いた時はY座標を増やす
			break;

		case east:
			mypos.x++;	//東を向いた時はX座標を増やす
			break;

		case south:
			mypos.y--;	//南を向いた時はY座標を減らす
			break;

		case west:
			mypos.x--;	//西を向いたときはX座標を減らす
			break;

	}



	while((mypos.x != x) || (mypos.y != y)){			//ゴールするまで繰り返す


		switch(get_nextdir(x,y,MASK_SECOND,&glob_nextdir))	//次に行く方向を戻り値とする関数を呼ぶ
		{
			case front:					//直線をまとめて走るようにする
				straight_count++;
				break;

			case right:
				straight(SECTION*straight_count,FAST_ACCEL,FAST_SPEED,0.0);
				turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);				//右に曲がって
				straight_count = 1;			//走る直線の距離をリセット
				break;

			case left:
				straight(SECTION*straight_count,FAST_ACCEL,FAST_SPEED,0.0);
				turn(90,TURN_ACCEL,TURN_SPEED,LEFT);				//左に曲がって
				straight_count = 1;			//走る直線の距離をリセット
				break;

			case rear:
				straight(SECTION*straight_count,FAST_ACCEL,FAST_SPEED,0.0);
				adjust_turn();				//左に曲がって
				straight_count = 1;			//走る直線の距離をリセット
				break;
		}

		mypos.dir = glob_nextdir;	//自分の向きを修正

		//向いた方向によって自分の座標を更新する
		switch(mypos.dir)
		{
			case north:
				mypos.y++;	//北を向いた時はY座標を増やす
				break;

			case east:
				mypos.x++;	//東を向いた時はX座標を増やす
				break;

			case south:
				mypos.y--;	//南を向いた時はY座標を減らす
				break;

			case west:
				mypos.x--;	//西を向いたときはX座標を減らす
				break;

		}
	}
	straight(SECTION*straight_count,FAST_ACCEL,FAST_SPEED,0.0);
}

void fast_run_sla(int x, int y)
{
//引数の座標x,yに向かって最短走行する

	t_direction glob_nextdir;

	float straight_length = 0;

	//現在の向きから、次に行くべき方向へ向く
	switch(get_nextdir(x,y,MASK_SECOND,&glob_nextdir))	//次に行く方向を戻り値とする関数を呼ぶ
	{
		case front:
			straight_length += HALF_SECTION;		//半区画進む
			break;

		case right:					//右に向く
			turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);				//右に曲がって
			break;

		case left:					//左に向く
			turn(90,TURN_ACCEL,TURN_SPEED,LEFT);				//左に曲がって
			break;

		case rear:					//後ろに向く
			adjust_turn();				//左に曲がって
			break;

	}

	mypos.dir = glob_nextdir;	//自分の向きを更新


	//向いた方向によって自分の座標を更新する
	switch(mypos.dir)
	{
		case north:
			mypos.y++;	//北を向いた時はY座標を増やす
			break;

		case east:
			mypos.x++;	//東を向いた時はX座標を増やす
			break;

		case south:
			mypos.y--;	//南を向いた時はY座標を減らす
			break;

		case west:
			mypos.x--;	//西を向いたときはX座標を減らす
			break;

	}

	while((mypos.x != x) || (mypos.y != y)){			//ゴールするまで繰り返す


		switch(get_nextdir(x,y,MASK_SECOND,&glob_nextdir))	//次に行く方向を戻り値とする関数を呼ぶ
		{
			case front:
				straight_length += SECTION;
				break;

			case right:
				straight(straight_length+SLA_SECTION,FAST_ACCEL,FAST_SPEED,SEARCH_SPEED);
				straight_length = 0;
				slalom_turn(90,SLA_ACCEL,SLA_SPEED,RIGHT,SEARCH_SPEED);				//右に曲がって
				straight(SLA_SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
				break;

			case left:
				straight(straight_length+SLA_SECTION,FAST_ACCEL,FAST_SPEED,SEARCH_SPEED);
				straight_length = 0;
				slalom_turn(90,SLA_ACCEL,SLA_SPEED,LEFT,SEARCH_SPEED);				//左に曲がって
				straight(SLA_SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
				break;

			case rear:
				straight(straight_length,FAST_ACCEL,FAST_SPEED,FAST_SPEED);		//半区画進む
				straight_length = 0;
				adjust_turn();					//180ターン
				straight_length += HALF_SECTION;
				break;
		}

		mypos.dir = glob_nextdir;	//自分の向きを修正

		//向いた方向によって自分の座標を更新する
		switch(mypos.dir)
		{
			case north:
				mypos.y++;	//北を向いた時はY座標を増やす
				break;

			case east:
				mypos.x++;	//東を向いた時はX座標を増やす
				break;

			case south:
				mypos.y--;	//南を向いた時はY座標を減らす
				break;

			case west:
				mypos.x--;	//西を向いたときはX座標を減らす
				break;

		}
	}
	straight_length += HALF_SECTION;
	straight(straight_length,FAST_ACCEL,FAST_SPEED,0.0);
}
