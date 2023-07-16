#pragma once

//物理的なパラメータ
#define TIRE_DIAMETER	(12.80)				//タイヤの直径	[mm]
#define TIRE_RADIUS	(TIRE_DIAMETER/2.0)		//タイヤの半径	[mm]
#define MMPP 		(TIRE_DIAMETER*PI)/(ENC_RES_MAX)	//エンコーダ1パルスあたりに進む距離[mm](TIRE_DIAMETER*PI)/(ENC_MAX)
#define ENC_RES_MAX	(1024)
#define ENC_RES_HALF	(512)

#define V_ref		3.8				//モータ制御の基準電圧

//ログ用のパラメータ
#define LOG_CNT		1000				//ログをとる個数。 1mmsで取得しているので、取得時間[s]はこの数の1000分の1

//センサ関連パラメータ
#define WAITLOOP_SLED	180				//LEDを光らせてからAD変換を開始するまでの時間稼ぎ用定数

#define REF_SEN_R	1422				//マウスを迷路中央に置いた時のセンサの値
#define REF_SEN_L	1155				//マウスを迷路中央に置いた時のセンサの値
#define REF_SEN_FR	3048				//マウスを迷路中央に置いた時のセンサの値
#define REF_SEN_FL	3040				//マウスを迷路中央に置いた時のセンサの値

#define TH_SEN_R	796				//壁があるか否かの閾値	車体を区画の左へ寄せた時のセンサ値(壁あり)
#define TH_SEN_L	555				//壁があるか否かの閾値	車体を区画の右へ寄せた時のセンサ値(壁あり)
#define TH_SEN_FR	1100				//壁があるか否かの閾値
#define TH_SEN_FL	920				//壁があるか否かの閾値

#define TH_RIGHT_90		1080
#define TH_LEFT_90		900

#define RIGHT_WALL_NOWALL		59.435032			//壁切れ補正用距離
#define LEFT_WALL_NOWALL		56.097076		//壁切れ補正用距離

#define CONTH_SEN_R	TH_SEN_R			//制御をかけるか否かの閾値
#define CONTH_SEN_L	TH_SEN_L			//制御をかけるか否かの閾値
#define CON_WALL_KP	(2.0)				//壁センサによる姿勢制御の比例制御の比例定数
#define CON_FWALL_KP	(100.0)

//フィードバックゲインパラメータ
//Pゲイン　最初に調整する	実速度が目標速度を中心として軽く振動する程度に調整
//Iゲイン　最後に調整する	積分値が合うようにする程度。
//Dゲイン　二番目に調整する。	P制御によって発生した振動を抑えられる程度に調整
//車体中心における並進方向速度に関するフィードバックゲイン
#define SPEED_KP	(10.0)				//Pゲイン
#define SPEED_KI	(0.05)				//Iゲイン
#define SPEED_KD	(0.0)				//Dゲイン　
//車体中心における回転方向速度に関するフィードバックゲイン
#define OMEGA_KP	(0.6)				//Pゲイン
#define OMEGA_KI	(0.01)				//Iゲイン
#define OMEGA_KD	(0.0)				//Dゲイン

//走行パラメータ
#define SEARCH_SPEED	(0.3)				//探索走行の速度	[m/s]
#define SEARCH_ACCEL	(1.5)				//探索走行の加速度	[m/s^2]
#define FAST_SPEED	(1.0)				//最短走行の速度	[m/s]
#define FAST_ACCEL	(2.5)				//最短走行の加速度	[m/s^2]
#define MIN_SPEED	(0.1)				//最低速度	[m/s]

#define TURN_ACCEL	(PI*20)				//超信地旋回の加速度	[rad/s^2]
#define	TURN_SPEED	(PI*10)				//超信地旋回の最高速度	[rad/s]
#define SLA_ACCEL	(PI*40)				//スラローム旋回の加速度	[rad/s^2]
#define SLA_SPEED	(PI*10)				//スラローム旋回の最高速度	[rad/s]
#define SLA_FAST_ACCEL	(PI*40)			//スラローム旋回の加速度	[rad/s^2]
#define SLA_FAST_SPEED	(PI*20)			//スラローム旋回の最高速度	[rad/s]
#define SLA_SECTION_PRE	16					//スラローム旋回前に進む距離
#define SLA_SECTION_POST	13					//スラローム旋回前に進む距離
#define TURN_MIN_SPEED	(PI/10.0)			//超信地旋回の最低速度	[rad/s]

#define WAIT_TIME	5				//各動作後の待機時間	[ms]

//迷路関連パラメータ
#define GOAL_X	10		//ゴール座標(x)
#define GOAL_Y	10		//ゴール座標(y)

//決定用センサパラメータ
#define	SEN_DECISION	2000	//メニュー決定用の光センサ閾値
