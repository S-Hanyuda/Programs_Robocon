#ifndef MOVECTRL
#define MOVECTRL
#include "mbed.h"

class MoveCtrl{
public:
	MoveCtrl(PinName P1, PinName P2, PinName P3, PinName P4, PinName D1, PinName D2, PinName D3, PinName D4);
	void Mechanum(unsigned char JoyX, unsigned char JoyY); //メカナム・オムニ(X字)制御
	void OmniK(unsigned char JoyX, unsigned char JoyY); //オムニ(十字)制御
	void Omni3(unsigned char JoyX, unsigned char JoyY); //オムニ(3軸)制御 未実装
	void Turn(unsigned char JoyX); //旋回制御
	void Turn_R(unsigned char JoyX); //旋回制御(左右反転)
	void SlideTurn(unsigned char JoyX, unsigned char JoyY); //横移動旋回制御 メカナム・オムニ(X字)用
	void SlideTurn_R(unsigned char JoyX, unsigned char JoyY); //横移動旋回制御(前後反転) メカナム・オムニ(X字)用

	//ユーティリティ関数
	void Slow(float value); //速度制限用
	void Slow();
	void Limiter(float value); //反応下限設定用
	void Limiter();
	void Per_SUM(float value); //台形制御増減割合設定用
	void Per_SUM();

	void Debug(); //回路デバッグ用
	void Emergency(); //緊急停止

private:
	PwmOut _P1, _P2, _P3, _P4;
	DigitalOut _D1, _D2, _D3, _D4;
	float PwmX, PwmY, PwmM, PwmC, Restrict, LowLimit, SUM_per; //各種変数
	float PwmSUM1, PwmSUM2, per1, per2;
	//float Trape1[2], Trape2[2];

	float Trape1[2], Trape2[2], Trape3[2], Trape4[2], PwmSUM[4];
	int Digital[4], Digital_old[4];

	int i; //forループ用変数
	int reseter; //配列初期化用変数

	//ジョイスティック数値補正関数
	float sub(unsigned char JoyN);
	float sub2(unsigned char JoyN);
	unsigned char sub3(unsigned char JoyN);
};

#endif
/*メカナム全方位制御の理屈
 * 作り変えて面倒なのでまだ書いてない
 * 動かし方の都合で全方位・旋回・横旋回移動がセットになってるのは確か

 * モーター配置
   ・メカナム or オムニX字配置
	[2]───────[1]
	 │         │
	 │    ↑    │
	 │         │
	[3]───────[4]

   ・オムニ十字配置
	 ┌───[1]───┐
	 │         │
	[2]   ↑   [4]
     │         │
     └───[3]───┘

 */
