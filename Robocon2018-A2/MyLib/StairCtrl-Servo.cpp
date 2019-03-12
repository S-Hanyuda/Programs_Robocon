#include <MyLib/StairCtrl-Servo.h>
#include "mbed.h"

StairCtrl_S::StairCtrl_S() {
	for(i=0; i<4; ++i) {

	}
	Distance = Angle = 0.0;
	FreQuency = 0;
	InvldRange = F_Angle = Time = 0.0;
}

//初期化
void StairCtrl_S::PwmPins(PwmOut *P0, PwmOut *P1, PwmOut *P2, PwmOut *P3, int period_ms) {
	_Pwm[0] = P0; //RightFront No.0
	_Pwm[1] = P1; //RightBack  No.1
	_Pwm[2] = P2; //LeftFront  No.2
	_Pwm[3] = P3; //LeftBack   No.3

	for(i=0; i<4; ++i) {
		_Pwm[i]->period_ms(period_ms);
		_Pwm[i]->pulsewidth_us(0);
	}
}

////////////////////
//移動実行
void StairCtrl_S::Move(unsigned char JoyX, unsigned char JoyY) {
	Distance = hypot(Revise(JoyX), Revise(JoyY)) / 128.0; //ジョイスティック傾き算出
	Angle = atan2(Revise(JoyY), Revise(JoyX)) * 180.0 / M_PI + 180; //ジョイスティック角度算出

	if(Distance > InvldRange) { //移動時処理
		//stop.detach();
		FreQuency = FreqCalc(fmod(Angle, 180));
		for(i=0; i<4; ++i) _Pwm[i]->pulsewidth_us(FreQuency);
	} else { //停止時処理
		_Pwm[0]->pulsewidth_us(NEUTRAL_0);
		_Pwm[1]->pulsewidth_us(NEUTRAL_1);
		_Pwm[2]->pulsewidth_us(NEUTRAL_2);
		_Pwm[3]->pulsewidth_us(NEUTRAL_3);
		//stop.attach((void*())StairCtrl_S::Emergency(), Time);
		/* FIXME:モーター側が止まるまで向きが変わらないようにタイマー割り込みをしようとしたところ、
		 * ポインタ周りでエラーを吐いたので上手く動かなかった
		 */
	}
}

void StairCtrl_S::Orient(double angle) {
	FreQuency = FreqCalc_inv(fmod(angle, 180));
	for(i=0; i<4; ++i) _Pwm[i]->pulsewidth_us(FreQuency);
}

void StairCtrl_S::Direct(int freq_us) {
	for(i=0; i<4; ++i) _Pwm[i]->pulsewidth_us(freq_us);
}

////////////////////
//各種設定

void StairCtrl_S::InvldArea(double value) { //スティック傾き無効範囲
	if(value >= 1.0) {
		InvldRange = INVALID_RANGE;
	} else if((value < 1.0) && (value > 0.0)) {
		InvldRange = value;
	} else {
		InvldRange = INVALID_RANGE;
	}
}
void StairCtrl_S::InvldArea() {
	InvldRange = INVALID_RANGE;
}


void StairCtrl_S::Delay(double value) { //スティック傾き無効範囲
	if(value >= 1.0) {
		Time = STOP_DELAY;
	} else if((value < 1.0) && (value >= 0.0)) {
		Time = value;
	} else {
		Time = STOP_DELAY;
	}

}
void StairCtrl_S::Delay() {
	Time = STOP_DELAY;
}

/*
void StairCtrl_S::ForceAngle(double value) { //四方位移動強制範囲
	if(value > 45.0) {
		F_Angle = FORCE_ANGLE;
	} else if((value <= 45.0) && ((value >= 0.0))) {
		F_Angle = value;
	} else {
		F_Angle = FORCE_ANGLE;
	}
}
void StairCtrl_S::ForceAngle() {
	F_Angle = FORCE_ANGLE;
}*/

/*
void StairCtrl_S::Offset(int n, double value) { //移動オフセット
	Offset[n] = value;
}
void StairCtrl_S::Offset() {
	for(i=0;i<4;++i) {
		Offset[i] = 1.0;
	}
}*/

////////////////////
//ユーティリティ
void StairCtrl_S::Debug() {
	for(i=0; i<4; ++i) _Pwm[i]->pulsewidth_us(SERVO_NEUTRAL - 500);
	wait(1);
	for(i=0; i<4; ++i) _Pwm[i]->pulsewidth_us(SERVO_NEUTRAL);
	wait(1);
	for(i=0; i<4; ++i) _Pwm[i]->pulsewidth_us(SERVO_NEUTRAL + 500);
	wait(1);
}


//FIXME:上記のFIXME参照
void StairCtrl_S::Emergency() {
	_Pwm[0]->pulsewidth_us(NEUTRAL_0);
	_Pwm[1]->pulsewidth_us(NEUTRAL_1);
	_Pwm[2]->pulsewidth_us(NEUTRAL_2);
	_Pwm[3]->pulsewidth_us(NEUTRAL_3);
}

void StairCtrl_S::Neutral() {
	for(i=0; i<4; ++i) _Pwm[i]->pulsewidth_us(SERVO_NEUTRAL);
}

////////////////////
//Private
double StairCtrl_S::Revise(char value) { //ジョイスティック補正 char[0~255] -> double[-128.0~127.0]
	return (double)value - 128.0;
}

int StairCtrl_S::FreqCalc(double angle) {
	return SERVO_NEUTRAL + (int)(angle / SERVO_MAX_ANGLE * SERVO_MAX_FREQ);
}
int StairCtrl_S::FreqCalc_inv(double angle) {
	return SERVO_NEUTRAL - (int)(angle / SERVO_MAX_ANGLE * SERVO_MAX_FREQ);
}
