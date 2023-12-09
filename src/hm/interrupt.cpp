#include "interrupt.h"
#include "glob_var.h"
#include "parameters.h"
#include "run.h"
#include "mytypedef.h"
#include "interface.h"
#include "misc.h"

#include "../driver/motor.h"
#include "../driver/photo.h"
#include "../driver/encoder.h"
#include "../driver/battery.h"
#include "../driver/imu.h"
#include "../driver/indicator.h"
#include <algorithm>

void int_cmt0(void)
{
	/*****************************************************************************************
	目標速度生成
		tar_speedの生成
		など
	*****************************************************************************************/
	// 直線の場合の目標速度生成
	if (run_mode == STRAIGHT_MODE || run_mode == F_WALL_MODE)
	{
		tar_speed += accel / 1000.0f; // 目標速度を設定加速度で更新
		// 最高速度制限
		if (tar_speed > max_speed)
		{
			tar_speed = max_speed; // 目標速度を設定最高速度に設定
		}
		// バック時の最高速度制限
		if (tar_speed < 0 && tar_speed < -1 * max_speed)
		{
			tar_speed = -max_speed; // 目標速度を設定最高速度で丸める
		}
	}
	else if (run_mode == TURN_MODE)
	{

		// 車体中心速度更新
		tar_speed += accel / 1000.0f;
		// 最高速度制限
		if (tar_speed > max_speed)
		{
			tar_speed = max_speed; // 目標速度を設定最高速度に設定
		}

		// 角加速度更新
		tar_ang_vel += ang_acc / 1000.0f; // 目標角速度を設定加速度で更新
		tar_degree += (tar_ang_vel * 180.0 / PI) / 1000.0f;

		// 左回転の場合
		if (TURN_DIR == RIGHT)
		{
			// 最高角速度制限
			if (tar_ang_vel > max_ang_vel)
			{
				tar_ang_vel = max_ang_vel; // 目標速度を設定最高速度に設定
			}
			if (tar_degree > max_degree)
			{
				tar_degree = max_degree;
			}
		}
		else if (TURN_DIR == LEFT)
		{
			// 右回転の場合
			// 最高角速度制限
			if (tar_ang_vel < max_ang_vel)
			{
				tar_ang_vel = max_ang_vel; // 目標速度を設定最高速度に設定
			}
			if (tar_degree < max_degree)
			{
				tar_degree = max_degree;
			}
		}
	}
	else if (run_mode == NON_CON_MODE)
	{
		// 何もしない
	}
	else
	{
		// 何もしない
	}

	/*****************************************************************************************
	壁制御
		横壁センサによる目標角度生成
	*****************************************************************************************/
	if (run_mode == STRAIGHT_MODE)
	{
		if (con_wall.enable == true && sen_fr.value + sen_fl.value <= (TH_SEN_FR + TH_SEN_FL) * 5) // 壁制御が許可されているかチェック
		{

			con_wall.p_error = con_wall.error; // 過去の偏差を保存

			// 左右のセンサが、それぞれ使える状態であるかどうかチェックして、姿勢制御の偏差を計算
			if ((sen_r.is_control == true) && (sen_l.is_control == true))
			{ // 両方とも有効だった場合の偏差を計算
				con_wall.error = sen_r.error - sen_l.error;
			}
			else // 片方もしくは両方のセンサが無効だった場合の偏差を計算
			{
				con_wall.error = 2.0f * (sen_r.error - sen_l.error); // 片方しか使用しないので2倍する
			}

			// DI制御計算
			con_wall.diff = con_wall.error - con_wall.p_error; // 偏差の微分値を計算
			con_wall.sum += con_wall.error;										 // 偏差の積分値を計算

			if (con_wall.sum > con_wall.sum_max) // 偏差の積分値の最大値を制限
			{
				con_wall.sum = con_wall.sum_max;
			}
			else if (con_wall.sum < (-con_wall.sum_max)) // 偏差の積分値の最低値を制限
			{
				con_wall.sum = -con_wall.sum_max;
			}

			con_wall.p_omega = con_wall.omega;
			con_wall.omega = con_wall.kp * con_wall.error * 0.5f + con_wall.p_omega * 0.5f; // 現在の目標角速度[rad/s]を計算
			tar_ang_vel = -con_wall.omega;
		}
		else
		{
			tar_ang_vel = 0;
		}
	}
	else if (run_mode == F_WALL_MODE)
	{
		/*****************************************************************************************
		壁制御
			前壁センサによる目標角度生成
		*****************************************************************************************/
		if (con_fwall.enable) // 壁制御が許可されているかチェック
		{

			con_fwall.p_error = con_fwall.error; // 過去の偏差を保存

			// 左右のセンサが、それぞれ使える状態であるかどうかチェックして、姿勢制御の偏差を計算
			if ((sen_fr.is_control == true) && (sen_fl.is_control == true))
			{ // 両方とも有効だった場合の偏差を計算
				con_fwall.error = sen_fl.error - sen_fr.error;
			}
			else // 片方もしくは両方のセンサが無効だった場合の偏差を計算
			{
				con_fwall.error = 2.0f * (sen_fl.error - sen_fr.error); // 片方しか使用しないので2倍する
			}

			// DI制御計算
			con_fwall.diff = con_fwall.error - con_fwall.p_error; // 偏差の微分値を計算
			con_fwall.sum += con_fwall.error;											// 偏差の積分値を計算

			if (con_fwall.sum > con_fwall.sum_max) // 偏差の積分値の最大値を制限
			{
				con_fwall.sum = con_fwall.sum_max;
			}
			else if (con_fwall.sum < (-con_fwall.sum_max)) // 偏差の積分値の最低値を制限
			{
				con_fwall.sum = -con_fwall.sum_max;
			}

			con_fwall.p_omega = con_fwall.omega;
			con_fwall.omega = con_fwall.kp * con_fwall.error * 0.5f + con_fwall.p_omega * 0.5f; // 現在の目標角速度[rad/s]を計算
			tar_ang_vel = -con_fwall.omega;
		}
		else
		{
			tar_ang_vel = 0;
		}
	}
	else if (run_mode == NON_CON_MODE)
	{
		// 何もしない
	}
	else
	{
	}

	/*****************************************************************************************
	壁制御かスラロームの理想値によって生成された
	目標速度と目標角角度の積分
	*****************************************************************************************/
	I_tar_speed += tar_speed;
	if (I_tar_speed > 30.0f * 10000000000.0f)
	{
		I_tar_speed = 30.0f * 10000000000.0f;
	}
	else if (I_tar_speed < -1.0f * 10000000000.0f)
	{
		I_tar_speed = 1.0f * 10000000000.0f;
	}

	I_tar_ang_vel += tar_ang_vel;
	if (I_tar_ang_vel > 30.0f * 10000000000.0f)
	{
		I_tar_ang_vel = 30.0f * 10000000000.0f;
	}
	else if (I_tar_ang_vel < -1.0f * 10000000000.0f)
	{
		I_tar_ang_vel = 1.0f * 10000000000.0f;
	}
	/*****************************************************************************************
	目標速度の偏差から出力電圧にフィードバック

	*****************************************************************************************/
	// フィードバック制御
	V_r = V_l = 0.0;
	if (run_mode == STRAIGHT_MODE || run_mode == F_WALL_MODE || run_mode == TURN_MODE)
	{
		// 直進時のフィードバック制御
		// 左右モータのフィードバック
		// 速度に対するP制御
		V_r += 1.0f * (tar_speed - speed) * SPEED_KP;
		V_l += 1.0f * (tar_speed - speed) * SPEED_KP;
		// 速度に対するI制御
		V_r += 1.0f * (I_tar_speed - I_speed) * SPEED_KI;
		V_l += 1.0f * (I_tar_speed - I_speed) * SPEED_KI;
		// 速度に対するD制御
		V_r -= 1.0f * (p_speed - speed) * SPEED_KD;
		V_l -= 1.0f * (p_speed - speed) * SPEED_KD;

		// 角速度に対するP制御
		V_r += 1.0f * (tar_ang_vel - ang_vel) * OMEGA_KP;
		V_l -= 1.0f * (tar_ang_vel - ang_vel) * OMEGA_KP;
		// 角速度に対するI制御

		V_r += 1.0f * (I_tar_ang_vel - I_ang_vel) * OMEGA_KI; //(0.4-0.3)*0.1 -> 0.01
		V_l -= 1.0f * (I_tar_ang_vel - I_ang_vel) * OMEGA_KI;
		// 角速度に対するD制御

		V_r -= 1.0f * (p_ang_vel - ang_vel) * OMEGA_KD; //(0.4-0.3)*0.1 -> 0.01
		V_l += 1.0f * (p_ang_vel - ang_vel) * OMEGA_KD;
	}
	else if (run_mode == NON_CON_MODE)
	{
		// 何もしない
	}
	else
	{
		// 何もしない
	}
	/*****************************************************************************************
	出力電圧の制限
		モータへの出力電圧の上限を3Vに制限
	*****************************************************************************************/
	V_r = std::min(3.0f, std::max(-3.0f, V_r));
	V_l = std::min(3.0f, std::max(-3.0f, V_l));

	/*****************************************************************************************
	モータへ出力

	*****************************************************************************************/

	// バッテリー電圧からデューティを計算
	Duty_r = V_r / V_bat;
	Duty_l = V_l / V_bat;

	// モータにPWMを出力
	if (run_mode != TEST_MODE)
	{
		driver::motor::duty({Duty_l, Duty_r});
	}

	timer++;
}

void int_cmt1(void) // センサ読み込み用り込み
{
	/*****************************************************************************************
	A/D変換
		センサとバッテリー電圧取得
	*****************************************************************************************/
	int photo[4] = {};

	driver::photo::get(photo);

	sen_r.value = (photo[driver::photo::RIGHT_45] - sen_r.d_value); // 値を保存

	if (sen_r.value > sen_r.th_wall) // 壁の有無を判断
	{
		sen_r.is_wall = true; // 右壁あり
		sen_r.error = sen_r.value - sen_r.ref;
		sen_r.is_control = true;
	}
	else
	{
		sen_r.is_wall = false; // 右壁なし
		sen_r.error = 0;
		sen_r.is_control = false;
	}

	if (sen_r.value > sen_r.th_control) // 制御をかけるか否かを判断
	{
		sen_r.error = sen_r.value - sen_r.ref; // 制御をかける場合は偏差を計算
		sen_r.is_control = true;							 // 右センサを制御に使う
	}
	else
	{
		sen_r.error = 0;					// 制御に使わない場合は偏差を0にしておく
		sen_r.is_control = false; // 右センサを制御に使わない
	}

	sen_fl.value = (photo[driver::photo::LEFT_90] - sen_fl.d_value); // 値を保存

	if (sen_fl.value > sen_fl.th_wall) // 壁の有無を判断
	{
		sen_fl.is_wall = true; // 左前壁あり
		sen_fl.error = sen_fl.value - sen_fl.ref;
		sen_fl.is_control = true;
	}
	else
	{
		sen_fl.is_wall = false; // 左前壁なし
		sen_fl.error = false;
		sen_fl.is_control = false;
	}

	sen_fr.value = (photo[driver::photo::RIGHT_90] - sen_fr.d_value); // 値を保存

	if (sen_fr.value > sen_fr.th_wall) // 壁の有無を判断
	{
		sen_fr.is_wall = true; // 右前壁あり
		sen_fr.error = sen_fr.value - sen_fr.ref;
		sen_fr.is_control = true;
	}
	else
	{
		sen_fr.is_wall = false; // 右前壁なし
		sen_fr.error = 0;
		sen_fr.is_control = false;
	}

	sen_l.value = (photo[driver::photo::LEFT_45] - sen_l.d_value); // 値を保存

	if (sen_l.value > sen_l.th_wall) // 壁の有無を判断
	{
		sen_l.is_wall = true; // 左壁あり
	}
	else
	{
		sen_l.is_wall = false; // 左壁なし
	}

	if (sen_l.value > sen_l.th_control) // 制御をかけるか否かを判断
	{
		sen_l.error = sen_l.value - sen_l.ref; // 制御をかける場合は偏差を計算する
		sen_l.is_control = true;							 // 左センサを制御に使う
	}
	else
	{
		sen_l.error = 0;					// 制御に使わない場合は偏差を0にしておく
		sen_l.is_control = false; // 左センサを制御に使わない
	}

	V_bat = static_cast<float>(driver::battery::get()) / 1000.0f;
	if (V_bat < 3.2)
	{
		// モータ止める
		Duty_r = 0;
		Duty_l = 0;
		driver::motor::brake();

                driver::indicator::set(0, 0xFF0000);
          driver::indicator::set(1, 0xFF0000);
          driver::indicator::set(2, 0xFF0000);
          driver::indicator::set(3, 0xFF0000);
		// ブザー鳴らし続ける
		while (1)
		{
                        wait_ms(1000);
		}
	}
	/*****************************************************************************************
	1kHzごとにログを取得

	*****************************************************************************************/

	if (log_flag == 1 && log_timer < (LOG_CNT))
	{

		log_buffer[0][log_timer] = (int)(len_mouse);
		log_buffer[1][log_timer] = (int)(1000 * tar_speed);
		log_buffer[2][log_timer] = (int)(1000 * speed);
		log_buffer[3][log_timer] = (int)(100 * Duty_r);
		log_buffer[4][log_timer] = (int)(100 * Duty_l);
		log_buffer[5][log_timer] = (int)(1000 * V_bat);
		log_buffer[6][log_timer] = (int)(tar_degree * 10);
		log_buffer[7][log_timer] = (int)(degree * 10);
		log_buffer[8][log_timer] = (int)(tar_ang_vel * 1000);
		log_buffer[9][log_timer] = (int)(ang_vel * 1000);
		log_buffer[10][log_timer] = (int)(I_tar_ang_vel);
		log_buffer[11][log_timer] = (int)(ang_acc * 1000);
	}

	log_timer++;
}

void int_cmt2(void)
{
	static unsigned int enc_data_r; // エンコーダの生データ
	static unsigned int enc_data_l; // エンコーダの生データ
	/*****************************************************************************************
	エンコーダ関連
		値の取得　速度更新　距離積分など
	*****************************************************************************************/
	auto [left, right] = driver::encoder::get();
	enc_data_r = right;
	enc_data_l = left;

	// 左右エンコーダから角度取得
	// 4096で一回転(360deg = 0deg)
	locate_r = enc_data_r;
	locate_l = enc_data_l;

	// 右エンコーダの現在の位置と,1msec前の位置との差分を計算
	// 単位時間（1msec）あたりの変位量を計算
	diff_pulse_r = (-locate_r + before_locate_r);
	// 変化点を1023から0//へ移動したときの補正
	if ((diff_pulse_r > ENC_RES_HALF || diff_pulse_r < -ENC_RES_HALF) && before_locate_r > ENC_RES_HALF)
	{
		diff_pulse_r = (((ENC_RES_MAX - 1) - before_locate_r) + locate_r);
	}
	// 変化点を0から1023へ移動したときの補正
	else if ((diff_pulse_r > ENC_RES_HALF || diff_pulse_r < -ENC_RES_HALF) && before_locate_r <= ENC_RES_HALF)
	{
		diff_pulse_r = 1 * (before_locate_r + ((ENC_RES_MAX - 1) - locate_r));
	}

	// 左エンコーダの現在の位置と,1msec前の位置との差分を計算
	// 単位時間（1msec）あたりの変位量を計算
	diff_pulse_l = (locate_l - before_locate_l);
	// 変化点を1023から0//へ移動したときの補正
	if ((diff_pulse_l > ENC_RES_HALF || diff_pulse_l < -ENC_RES_HALF) && before_locate_l > ENC_RES_HALF)
	{
		diff_pulse_l = 1 * (((ENC_RES_MAX - 1) - before_locate_l) + locate_l);
	}
	// 変化点を0から1023へ移動したときの補正
	else if ((diff_pulse_l > ENC_RES_HALF || diff_pulse_l < -ENC_RES_HALF) && before_locate_l <= ENC_RES_HALF)
	{
		diff_pulse_l = (before_locate_l + ((ENC_RES_MAX - 1) - locate_l));
	}

	// 現在速度を算出
	speed_new_r = (float)((float)diff_pulse_r * (float)MMPP);
	speed_new_l = (float)((float)diff_pulse_l * (float)MMPP);

	// 過去の値を保存
	speed_old_r = speed_r;
	speed_old_l = speed_l;

	// 速度のローパスフィルタ
	speed_r = speed_new_r * 0.1f + speed_old_r * 0.9f;
	speed_l = speed_new_l * 0.1f + speed_old_l * 0.9f;

	p_speed = speed;
	// 車体速度を計算
	speed = ((speed_r + speed_l) / 2.0f);

	// I成分のオーバーフローとアンダーフロー対策
	I_speed += speed;
	if (I_speed > 30.0f * 10000000000.0f)
	{
		I_speed = 30.0f * 10000000000.0f;
	}
	else if (I_speed < -1.0f * 10000000000.0f)
	{
		I_speed = -1.0f * 10000000000.0f;
	}

	// 距離の計算
	len_mouse += (speed_new_r + speed_new_l) / 2.0f;

	// 過去の値を保存
	before_locate_r = locate_r;
	before_locate_l = locate_l;

	/*****************************************************************************************
	ジャイロ関連(ヨー軸)
		値の取得　角度の積分　
	*****************************************************************************************/
	auto [gyro_unused_x, gyro_unused_y, gyro_z] = driver::imu::gyro_raw();

	gyro_x_new = (float)gyro_z;
	gyro_x = (gyro_ref - gyro_x_new) * 70.0f;

	// 角速度の更新
	p_ang_vel = ang_vel;
	ang_vel = gyro_x * PI / 180.0f / 1000.0f;
	// 積分値の更新
	I_ang_vel += ang_vel;
	if (I_ang_vel > 30.0f * 10000000000.0f)
	{
		I_ang_vel = 30.0f * 10000000000.0f;
	}
	else if (I_ang_vel < -1.0f * 10000000000.0f)
	{
		I_ang_vel = -1.0f * 10000000000.0f;
	}

	// ジャイロの値を角度に変換
	degree += gyro_x / 1'000'000.0f;
}
