#ifndef STAIRCTRL
#define STAIRCTRL
#include "mbed.h"

class StairCtrl_S{
public:
	StairCtrl_S();
	//初期化
	void PwmPins(PwmOut *P0, PwmOut *P1, PwmOut *P2, PwmOut *P3, int period_ms);

	//移動実行
	void Move(unsigned char JoyX, unsigned char JoyY);
	void Orient(double angle);
	void Direct(int freq);
	#define FRONT 1500
	#define BACK  1500
	#define RIGHT 2033
	#define LEFT  967

	//各種設定
	void InvldArea(double value); //スティック傾き無効範囲
	void InvldArea();
	void Delay(double value); //スティック傾き無効範囲
	void Delay();
	//void ForceAngle(double value); //四方位移動強制範囲
	//void ForceAngle();

	//void Offset(int n, double value); //移動オフセット
	//void Offset();

	//ユーティリティ
	void Debug(); //回路デバッグ用
	void Emergency(); //緊急停止&マシン停止時
	void Neutral(); //サーボニュートラル位置移動

private:
	//宣言
	PwmOut *_Pwm[4]; //サーボモーター用PWM(ポインタ)
	//Timeout stop;

	//各種変数
	double Distance, Angle;
	int FreQuency;
	double InvldRange, F_Angle, Time;

	#define INVALID_RANGE 0.1  //スティック傾き無効範囲
	//#define FORCE_ANGLE 2.5    //四方位移動強制範囲
	#define STOP_DELAY 0.5     //停止割り込み時間

	#define NEUTRAL_0 1767 //各サーボ停止位置
	#define NEUTRAL_1 1233
	#define NEUTRAL_2 1233
	#define NEUTRAL_3 1767

	//ユーティリティ変数
	int i; //forループ用変数

	//ライブラリ内利用関数
	double Revise(char value); //ジョイスティック補正 char[0~255] -> double[-128~127]
	int FreqCalc(double angle);     //角度->周波数変換
	int FreqCalc_inv(double angle); //角度->周波数変換(角度反転)

	//KONDO KRS-2552RHV
	#define SERVO_NEUTRAL   1500 //サーボ初期位置パルス幅
	#define SERVO_MAX_ANGLE 135  //サーボ最大角度
	#define SERVO_MAX_FREQ  800  //サーボ最大角度周波数
};

#endif
/*ステアリング制御
 *
 * モーター配置
 * ・サーボ
 *   [2]───────[0]
 *    │         │
 *    │    ↑    │
 *    │         │
 *   [3]───────[1]
 *  [KONDO KRS-2552RHV]仕様
 *
*/

/*
 * タスク
 * ・サーボ側ライブラリ：完成-8/20
 * ・デバッグ待ち
 *
 */
