#ifndef STAIRCTRL
#define STAIRCTRL
#include "mbed.h"

class StairCtrl_M{
public:
	StairCtrl_M();
	//初期化
	void DigiPins(DigitalOut *D0, DigitalOut *D1, DigitalOut *D2, DigitalOut *D3);
	void PwmPins(PwmOut *P0, PwmOut *P1, PwmOut *P2, PwmOut *P3, int period_ms);

	//移動実行
	void Move(unsigned char JoyX, unsigned char JoyY, unsigned char L2, unsigned char R2);
	void Speed(double value);
	void Speed();

	//各種設定
	void InvldArea(double value); //スティック傾き無効範囲
	void InvldArea();
	void ForceAngle(double value); //四方位移動強制範囲
	void ForceAngle();

	//void MoveOffset(int n, double value); //移動オフセット
	//void MoveOffset();

	//ユーティリティ
	void Debug(); //回路デバッグ用
	void Emergency(); //緊急停止

private:
	//ピン宣言(ポインタ)
	DigitalOut *_Digi[4]; //モーター用デジタル&PWM
	PwmOut *_Pwm[4];

	//各種変数
	double PwmValue[4][2], PwmSUM[4], TurnValue[4]; //PWM実行変数, PWM目標値, 旋回制御
	bool Digital[4]; //DigitalOut制御
	double Distance, Angle;
	double Prop, Restrict;
	unsigned char L2_old, R2_old; //旋回方向検出用

	double InvldRange, F_Angle;
	#define INVALID_RANGE 0.1  //スティック傾き無効範囲
	#define FORCE_ANGLE 2.5    //四方位移動強制範囲

	//ユーティリティ変数
	int i; //forループ用変数

	//ライブラリ内利用関数
	double Revise(char value); //ジョイスティック補正 char[0~255] -> double[-128~127]
	void PwmCtrl(int N); //モーター回転制御関数
	int TurnAct(unsigned char L2, unsigned char R2); //旋回条件判定
	void TurnCtrl(int N, unsigned char value); //超信地旋回処理

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
 *
 *
*/

/*
 * タスク
 * ・サーボ側ライブラリ：完成-8/20
 *
 */
