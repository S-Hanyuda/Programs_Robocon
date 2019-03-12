#include <MyLib/StairCtrl-Motor.h>
#include "mbed.h"

StairCtrl_M::StairCtrl_M() {
	for(i=0; i<4; ++i) {

	}
	Distance = Angle = 0;
	InvldRange = F_Angle = 0;
	Prop = 0.1;
}

//初期化
void StairCtrl_M::DigiPins(DigitalOut *D0, DigitalOut *D1, DigitalOut *D2, DigitalOut *D3) {
	_Digi[0] = D0;
	_Digi[1] = D1;
	_Digi[2] = D2;
	_Digi[3] = D3;
	for(i=0; i<4; ++i) *_Digi[i] = 0;
}

void StairCtrl_M::PwmPins(PwmOut *P0, PwmOut *P1, PwmOut *P2, PwmOut *P3, int period_us) {
	_Pwm[0] = P0; //RightFront No.0
	_Pwm[1] = P1; //RightBack  No.1
	_Pwm[2] = P2; //LeftFront  No.2
	_Pwm[3] = P3; //LeftBack   No.3

	for(i=0; i<4; ++i) {
		_Pwm[i]->period_us(period_us);
		_Pwm[i]->write(0.0);
	}
}

////////////////////
//移動実行
void StairCtrl_M::Move(unsigned char JoyX, unsigned char JoyY, unsigned char L2, unsigned char R2) {
	Distance = hypot(Revise(JoyX), Revise(JoyY)) / 128.0; //ジョイスティック傾き算出
	Angle = atan2(Revise(JoyY), Revise(JoyX)) * 180.0 / M_PI + 180; //ジョイスティック角度算出

	if(Distance > InvldRange) { //移動時処理
		//移動実行
		if( (Angle >= 0) && (Angle <= 180) ) { //前進
			Digital[0] = Digital[2] = 1;
			Digital[1] = Digital[3] = 0;
			if( TurnAct(L2, R2) == 1) {
				TurnValue[0] = TurnValue[3] = 0.0;
				TurnValue[1] = TurnValue[2] = (double)L2 / 255;
			} else if( TurnAct(L2, R2) == 2) {
				TurnValue[1] = TurnValue[2] = 0.0;
				TurnValue[0] = TurnValue[3] = (double)R2 / 255;
			} else {
				for(i=0; i<4; ++i) TurnValue[i] = 0.0;
			}
			for(i=0; i<4; ++i) PwmCtrl(i);
		} else if( (Angle > 180) && (Angle <= 360) ) { //後進
			Digital[0] = Digital[2] = 0;
			Digital[1] = Digital[3] = 1;
			if( TurnAct(L2, R2) == 1) {
				TurnValue[0] = TurnValue[3] = 0.0;
				TurnValue[1] = TurnValue[2] = (double)L2 / 255;
			} else if( TurnAct(L2, R2) == 2) {
				TurnValue[1] = TurnValue[2] = 0.0;
				TurnValue[0] = TurnValue[3] = (double)R2 / 255;
			} else {
				for(i=0; i<4; ++i) TurnValue[i] = 0.0;
			}
			for(i=0; i<4; ++i) PwmCtrl(i);
		}
	} else { //停止時処理
		if(L2 || R2) {
			if(L2 > R2) { //旋回方向判定
				for(i=0; i<4; ++i) {
					Digital[i] = 1;
					TurnCtrl(i, L2);
				}
			} else if (L2 < R2) {
				for(i=0; i<4; ++i) {
					Digital[i] = 0;
					TurnCtrl(i, R2);
				}
			} else if( (L2 > L2_old) && (R2 <= R2_old) ) {
				for(i=0; i<4; ++i) {
					Digital[i] = 1;
					TurnCtrl(i, L2);
				}
			} else if( (R2 > R2_old) && (L2 <= L2_old) ) {
				for(i=0; i<4; ++i) {
					Digital[i] = 0;
					TurnCtrl(i, R2);
			}}
		} else {
			for(i=0; i<4; ++i) {
				if(PwmValue[i][0] > 0.0) {
					PwmValue[i][0] -= Prop;
					PwmValue[i][1] = 1 - PwmValue[i][0];
					_Pwm[i]->write( PwmValue[i][ *_Digi[i] ] );
		}}}}
	L2_old = L2;
	R2_old = R2;
}

void StairCtrl_M::Speed(double value) {
	if(value >= 1.0) {
		Restrict = 1.0;
	} else if( (value < 1.0) && (value > 0.0) ) {
		Restrict = value;
	} else {
		Restrict = 1.0;
	}
}
void StairCtrl_M::Speed() {
	Restrict = 1.0;
}

////////////////////
//各種設定

void StairCtrl_M::InvldArea(double value) { //スティック傾き無効範囲
	if(value >= 1.0) {
		InvldRange = INVALID_RANGE;
	} else if((value < 1.0) && ((value > 0.0))) {
		InvldRange = value;
	} else {
		InvldRange = INVALID_RANGE;
	}
}
void StairCtrl_M::InvldArea() {
	InvldRange = INVALID_RANGE;
}

void StairCtrl_M::ForceAngle(double value) { //四方位移動強制範囲
	if(value > 45.0) {
		F_Angle = FORCE_ANGLE;
	} else if((value <= 45.0) && ((value >= 0.0))) {
		F_Angle = value;
	} else {
		F_Angle = FORCE_ANGLE;
	}
}
void StairCtrl_M::ForceAngle() {
	F_Angle = FORCE_ANGLE;
}

/*
void StairCtrl::MoveOffset(int n, double value) { //移動オフセット
	Offset[n] = value;
}
void StairCtrl::MoveOffset() {
	for(i=0;i<4;++i) {
		Offset[i] = 1.0;
	}
}*/

////////////////////
//ユーティリティ
void StairCtrl_M::Debug() {
	for(i=0; i<4; ++i) *_Digi[i] = 0;
	_Pwm[0]->write(0.7);
	_Pwm[1]->write(0.7);
	_Pwm[2]->write(0.7);
	_Pwm[3]->write(0.7);
}

void StairCtrl_M::Emergency() {
	for(i=0; i<4; ++i) *_Digi[i] = 0;
	_Pwm[0]->write(0.0);
	_Pwm[1]->write(0.0);
	_Pwm[2]->write(0.0);
	_Pwm[3]->write(0.0);
	for(i=0; i<4; ++i) {
		PwmValue[i][0] = 0.0;
		PwmValue[i][1] = 1.0;
	}
}

////////////////////
//Private
double StairCtrl_M::Revise(char value) { //ジョイスティック補正 char[0~255] -> double[-128.0~127.0]
	return (double)value - 128.0;
}

void StairCtrl_M::PwmCtrl(int N) { //モーター回転制御関数
	if( (*_Digi[N] != Digital[N]) && (PwmValue[N][0] > 0.0) ) { //まだ逆に回ってる場合
		PwmValue[N][0] -= Prop;
		PwmValue[N][1] = 1 - PwmValue[N][0];
		_Pwm[N]->write( PwmValue[N][ *_Digi[N] ] );
	} else { //止まってるor同じ方向に回ってる場合
		*_Digi[N] = Digital[N]; //モーター回転方向設定

		PwmSUM[N] = Distance * (1.0 - TurnValue[N]); //PwmSUM算出処理

		//台形制御
		if( (PwmValue[N][0] > PwmSUM[N]) || (PwmValue[N][0] > Restrict * (1.0 - TurnValue[N]) ) ){
			PwmValue[N][0] -= Prop;
			PwmValue[N][1] = 1.0 - PwmValue[N][0];
		} else if( (PwmValue[N][0] < PwmSUM[N]) || (PwmValue[N][0] < Restrict * TurnValue[N]) ) {
			PwmValue[N][0] += Prop;
			PwmValue[N][1] = 1.0 - PwmValue[N][0];
		}
		_Pwm[N]->write( PwmValue[N][ Digital[N] ] );
	}
}

int StairCtrl_M::TurnAct(unsigned char L2, unsigned char R2) {
	if(L2 || R2) {
		if(L2 > R2) { //旋回方向判定
			return 1;
		} else if (L2 < R2) {
			return 2;
		} else if( (L2 > L2_old) && (R2 <= R2_old) ) {
			return 1;
		} else if( (R2 > R2_old) && (L2 <= L2_old) ) {
			return 2;
		}
	} else return 0;
	return 0;
}

void StairCtrl_M::TurnCtrl(int N, unsigned char value) {
	if( (*_Digi[N] != Digital[N]) && (PwmValue[N][0] > 0.0) ) { //まだ逆に回ってる場合
		PwmValue[N][0] -= Prop;
		PwmValue[N][1] = 1 - PwmValue[N][0];
		_Pwm[N]->write( PwmValue[N][ *_Digi[N] ] );
	} else { //止まってるor同じ方向に回ってる場合
		*_Digi[N] = Digital[N];
		PwmSUM[N] = (double)value / 255;
		if( (PwmValue[N][0] > PwmSUM[N]) || (PwmValue[N][0] > Restrict) ) {
			PwmValue[N][0] -= Prop;
			PwmValue[N][1] = 1.0 - PwmValue[N][0];
		} else if( (PwmValue[N][0] < PwmSUM[N]) || (PwmValue[N][0] < Restrict) ) {
			PwmValue[N][0] += Prop;
			PwmValue[N][1] = 1.0 - PwmValue[N][0];
		}
		_Pwm[N]->write( PwmValue[N][ Digital[N] ] );
	}
}
