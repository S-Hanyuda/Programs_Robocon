#ifndef MWCTRLV1-2
#define MWCTRLV1-2
#include "mbed.h"

class MWCtrl{
public:
	MWCtrl();
	//初期化
	void PwmPins(PwmOut *P0, PwmOut *P1, PwmOut *P2, PwmOut *P3, int period_us);
	void DigiPins(DigitalOut *D0, DigitalOut *D1, DigitalOut *D2, DigitalOut *D3);

	//移動実行
	void Move(unsigned char JoyX, unsigned char JoyY, unsigned char L2, unsigned char R2);
	void Speed(double value);
	void Speed();

	//各種設定
	void Trape_Prop(double value); //台形制御増減割合
	void Trape_Prop();
	void InvldArea(double value); //スティック傾き無効範囲
	void InvldArea();
	void ForceAngle(double value); //四方位移動強制範囲
	void ForceAngle();
	void MoveOffset(int n, double value); //移動オフセット
	void MoveOffset();

	//ユーティリティ
	void Debug();     //回路デバッグ用
	void Debug_inv(); //回路デバッグ用 (反転)
	void Emergency(); //緊急停止

private:
	//ピン宣言(ポインタ)
	DigitalOut *_Digi[4]; //モーター:デジタル
	PwmOut *_Pwm[4]; //モーター:PWM

	//各種変数
	double PwmValue[4][2], PwmSUM[4], TurnValue[4]; //PWM実行変数, PWM最大値, 旋回制御
	bool Digital[4]; //DigitalOut制御
	double Distance, Angle; //ジョイスティック傾き距離, ジョイスティック角度
	double Prop, AngleProp; // 台形制御加算割合, 斜め移動スティック角度割合
	double InvldRange, FrcAngle, Restrict; //傾き無効距離, 四方位移動強制角度, 速度制限
	double Offset[4]; //移動オフセット
	unsigned char L2_old, R2_old; //旋回方向検出用

	//ユーティリティ変数
	int i; //forループ用変数

	//ライブラリ内利用関数
	double Revise(char value); //ジョイスティック補正 char[0~255] -> double[-128~127]
	void PwmCtrl(int N); //モーター回転制御関数
	double AngleCalc(double _Angle); //AngleProp算出処理
	int TurnAct(unsigned char L2, unsigned char R2); //旋回条件判定
	void TurnCalc(unsigned char L2, unsigned char R2, int mode); //旋回処理修正
	void TurnCtrl(int N, unsigned char value); //超信地旋回処理

};

#endif
/*メカナム全方位制御の理屈
 *
 * モーター配置
 * ・メカナム or オムニ
 *   [1]───────[0]
 *    │         │
 *    │    ↑    │
 *    │         │
 *   [2]───────[3]
 *
 *   面倒なので理屈は未記述
*/

/*
 * タスク
 * ・移動アルゴリズム完成 - 12/25完了 - デバッグ待ち
 * ・旋回部分修正 - 2/27仮実装完了 - デバッグ待ち
 * ・移動オフセット組み込み - 完了
 * ・旋回処理変更：2018/10/6
 */
