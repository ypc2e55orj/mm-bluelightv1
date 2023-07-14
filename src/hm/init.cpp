/***********************************************************************/
/*                                                                     */
/*  FILE        :init.c                                                */
/*  DATE        :2017/6/27	                                       */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :RX631 48P                                             */
/*                                                                     */
/*  NOTE:Initiarise micro controler.                                   */
/*                                                                     */
/***********************************************************************/
#include "sci.h"
#include "glob_var.h"
#include "DataFlash.h"
#include "misc.h"
#include "interface.h"

#include "../driver/imu.h"

/*****************************************************************************************
光センサー系のパラメータ初期化
	リファレンスとか壁の閾値とか
*****************************************************************************************/
void init_parameters(void)
{
	sen_r.ref = REF_SEN_R;				//右センサのリファレンス値を初期化
	sen_l.ref = REF_SEN_L;				//左センサのリファレンス値を初期化

	sen_r.th_wall = TH_SEN_R;			//右センサの壁有無判断の閾値を初期化
	sen_l.th_wall = TH_SEN_L;			//左センサの壁有無判断の閾値を初期化

	sen_fr.th_wall = TH_SEN_FR;			//右前センサの壁有無判断の閾値を初期化
	sen_fl.th_wall = TH_SEN_FL;			//左前センサの壁有無判断の閾値を初期化

	sen_r.th_control = CONTH_SEN_R;			//右センサの壁制御かけるか否かの閾値を初期化
	sen_l.th_control = CONTH_SEN_L;			//左センサの壁制御かけるか否かの閾値を初期化

	con_wall.kp = CON_WALL_KP/10000.0;			//壁比例制御の比例定数を初期化
}

/*****************************************************************************************
迷路情報の初期化

*****************************************************************************************/
void init_maze(void)	//迷路情報の初期化
{
	int i,j;

	for(i = 0; i < MAZESIZE_X; i++)
	{
		for(j = 0; j < MAZESIZE_Y; j++)
		{
			wall[i][j].north = wall[i][j].east = wall[i][j].south = wall[i][j].west = UNKNOWN;	//迷路の全体がわからない事を設定する
		}
	}

	for(i = 0; i < MAZESIZE_X; i++)
	{
		wall[i][0].south = WALL;		//四方の壁を追加する(南)
		wall[i][MAZESIZE_Y-1].north = WALL;	//四方の壁を追加する(北)
	}

	for(j = 0; j < MAZESIZE_Y; j++)
	{
		wall[0][j].west = WALL;			//四方の壁を追加する(西)
		wall[MAZESIZE_X-1][j].east = WALL;	//四方の壁を追加する(東)
	}

	wall[0][0].east = wall[1][0].west = WALL;	//スタート地点の右の壁を追加する

}


/*****************************************************************************************
ジャイロのリファレンス取得

*****************************************************************************************/
void gyro_get_ref(void){
	long i = 0;
	float gyro_ref_temp = 0;
	gyro_ref = 0;
	//ジャイロのリファレンス取得
	for(i = 0; i < 2500; i++){
		gyro_ref_temp += (float)gyro_x_new;
		wait_ms(1);
	}
	gyro_ref = (gyro_ref_temp/2500.0);
	degree = 0;
	wait_ms(100);
}

/*****************************************************************************************
全ての機能のイニシャライズ

*****************************************************************************************/
void init_all(void){
	LED(0);

	init_parameters();
	init_maze();

  timer = 0;

}
