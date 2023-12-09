#pragma once

#define PI (3.141592653589793f)			//円周率

//方位
#define RIGHT	(0)
#define LEFT	(1)
#define FRONT	(2)
#define REAR	(3)

//最短走行終了フラグ
#define END_FAST	(4)

#define HALF_SECTION	(45)			//半区画の距離
#define SECTION			(90)		//一区画の距離

#define MAZESIZE_X		(32)		//迷路の大きさ(MAZESIZE_X * MAZESIZE_Y)迷路
#define MAZESIZE_Y		(32)		//迷路の大きさ(MAZESIZE_X * MAZESIZE_Y)迷路

#define UNKNOWN	2				//壁があるかないか判らない状態の場合の値
#define NOWALL	0				//壁がないばあいの値
#define WALL	1				//壁がある場合の値
#define VWALL	3				//仮想壁の値(未使用)

#define STRAIGHT_MODE	0			//直進時のモード
#define TURN_MODE	1			//超信地旋回時のモード
#define SLA_MODE	2			//スラロームモード
#define NON_CON_MODE	3			//非制御モード
#define TEST_MODE	4			//テストモード(割り込み用モータ制御を切るモード)
#define F_WALL_MODE	5

#define MASK_SEARCH		0x01		//探索走行用マスク値.壁情報とこの値のAND値が０（NOWALL）なら壁なしor未探索区間
#define MASK_SECOND		0x03		//最短走行用マスク値.壁情報とこの値のAND値が０（NOWALL）なら壁なし
