#include <MyLib/MWCtrl-v1.h>
#include "mbed.h"

MWCtrl::MWCtrl() {
	for(i=0; i<4; ++i) {
		PwmValue[i][0] = PwmValue[i][1] = 0.0;
		PwmSUM[i] = Offset[i] = 0.0;
		Digital[i] = 0;
		TurnValue[i] = 1.0;
	}
	Distance = Angle = InvldRange = FrcAngle = 0.0;
	Prop = Restrict = AngleProp = 1.0;
	L2_old = R2_old = 0;
}

//初期化
void MWCtrl::PwmPins(PwmOut *P0, PwmOut *P1, PwmOut *P2, PwmOut *P3, int period_us) {
	_Pwm[0] = P0; //RightFront No.0
	_Pwm[1] = P1; //LeftFront  No.1
	_Pwm[2] = P2; //LeftBack   No.2
	_Pwm[3] = P3; //RightBack  No.3

	for(i=0; i<4; ++i) {
		_Pwm[i]->period_us(period_us);
		_Pwm[i]->write(0.0);
	}
}

void MWCtrl::DigiPins(DigitalOut *D0, DigitalOut *D1, DigitalOut *D2, DigitalOut *D3) {
	_Digi[0] = D0;
	_Digi[1] = D1;
	_Digi[2] = D2;
	_Digi[3] = D3;

	for(i=0; i<4; ++i) *_Digi[i] = 0;
}

////////////////////
//移動実行
void MWCtrl::Move(unsigned char JoyX, unsigned char JoyY, unsigned char L2, unsigned char R2) {
	Distance = hypot(Revise(JoyX), Revise(JoyY)) / 128.0; //ジョイスティック傾き算出
	Angle = atan2(Revise(JoyY), Revise(JoyX)) * 180.0 / M_PI + 180; //ジョイスティック角度算出 M_PI:math.h内の円周率定数(double型)

	if(Distance > InvldRange) { //移動時処理
		//移動実行
		if( (Angle >= 45) && (Angle <= 135) ) { //前進
			Digital[0] = Digital[3] = 1;
			Digital[1] = Digital[2] = 0;
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
		} else if( (Angle >= 225) && (Angle <= 315) ) { //後進
			Digital[0] = Digital[3] = 0;
			Digital[1] = Digital[2] = 1;
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
		} else if( (Angle > 135) && (Angle < 225) ) { //右進
			Digital[0] = Digital[1] = 0;
			Digital[2] = Digital[3] = 1;
			if( TurnAct(L2, R2) == 1 ) {
				TurnValue[2] = TurnValue[3] = 0.0;
				TurnValue[0] = TurnValue[1] = (double)L2 / 255;
			} else if( TurnAct(L2, R2) == 2 ) {
				TurnValue[0] = TurnValue[1] = 0.0;
				TurnValue[2] = TurnValue[3] = (double)R2 / 255;
			} else {
				for(i=0; i<4; ++i) TurnValue[i] = 0.0;
			}
			for(i=0; i<4; ++i) PwmCtrl(i);
		} else if( (Angle < 45) || (Angle > 315) ) { //左進
			Digital[0] = Digital[1] = 1;
			Digital[2] = Digital[3] = 0;
			if( TurnAct(L2, R2) == 1 ) {
				TurnValue[0] = TurnValue[1] = 0.0;
				TurnValue[2] = TurnValue[3] = (double)L2 / 255;
			} else if( TurnAct(L2, R2) == 2 ) {
				TurnValue[2] = TurnValue[3] = 0.0;
				TurnValue[0] = TurnValue[1] = (double)R2 / 255;
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
				}
			}
		} else {
			for(i=0; i<4; ++i) {
				if(PwmValue[i][0] > 0.0) {
					PwmValue[i][0] -= Prop;
					PwmValue[i][1] = 1 - PwmValue[i][0];
					_Pwm[i]->write( PwmValue[i][ *_Digi[i] ] );
				}
			}
		}
	}
	L2_old = L2;
	R2_old = R2;
}

void MWCtrl::Speed(double value) {
	if(value >= 1.0) {
		Restrict = 1.0;
	} else if( (value < 1.0) && (value > 0.0) ) {
		Restrict = value;
	} else {
		Restrict = 1.0;
	}
}
void MWCtrl::Speed() {
	Restrict = 1.0;
}

////////////////////
//各種設定
void MWCtrl::Trape_Prop(double value) { //台形制御増減割合
	if(value >= 1.0) {
		Prop = 1.0;
	} else if((value < 1.0) && ((value > 0.0))) {
		Prop = value;
	} else {
		Prop = 1.0;
	}
}
void MWCtrl::Trape_Prop() {
	Prop = 0.05;
}

void MWCtrl::InvldArea(double value) { //スティック傾き無効範囲
	if(value >= 1.0) {
		InvldRange = 0.1;
	} else if((value < 1.0) && ((value > 0.0))) {
		InvldRange = value;
	} else {
		InvldRange = 0.1;
	}
}
void MWCtrl::InvldArea() {
	InvldRange = 0.1;
}

void MWCtrl::ForceAngle(double value) { //四方位移動強制範囲
	if(value > 45.0) {
		FrcAngle = 2.5;
	} else if((value <= 45.0) && ((value >= 0.0))) {
		FrcAngle = value;
	} else {
		FrcAngle = 2.5;
	}
}
void MWCtrl::ForceAngle() {
	FrcAngle = 2.5;
}

void MWCtrl::MoveOffset(int n, double value) { //移動オフセット
	Offset[n] = value;
}
void MWCtrl::MoveOffset() {
	for(i=0;i<4;++i) {
		Offset[i] = 1.0;
	}
}

////////////////////
//ユーティリティ
void MWCtrl::Debug() {
	for(i=0; i<4; ++i) *_Digi[i] = 0;
	_Pwm[0]->write(0.7);
	_Pwm[1]->write(0.7);
	_Pwm[2]->write(0.7);
	_Pwm[3]->write(0.7);
}

void MWCtrl::Emergency() {
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
double MWCtrl::Revise(char value) { //ジョイスティック補正 char[0~255] -> double[-128.0~127.0]
	return (double)value - 128.0;
}

void MWCtrl::PwmCtrl(int N) { //モーター回転制御関数
	if( (*_Digi[N] != Digital[N]) && (PwmValue[N][0] > 0.0) ) { //まだ逆に回ってる場合
		PwmValue[N][0] -= Prop;
		PwmValue[N][1] = 1 - PwmValue[N][0];
		_Pwm[N]->write( PwmValue[N][ *_Digi[N] ] );
	} else { //止まってるor同じ方向に回ってる場合
		*_Digi[N] = Digital[N]; //モーター回転方向設定

		//AngleProp演算
		if( (fmod(Angle, 90) <= FrcAngle) || (fmod(Angle, 90) >= 90 - FrcAngle) ) { //4方位強制の場合
			AngleProp = 1.0;
		} else if( ((int)Angle / 90) % 2 == (N % 2) ) { //スティックの傾いていない方向の場合
			AngleProp = 1.0;
		} else if( ((int)Angle / 45) % 2 == 0) { //値反転が必要な場合
			AngleProp = 1.0 - AngleCalc(Angle); // - FrcAngle);
		} else { //値反転が不要な場合
			AngleProp = AngleCalc(Angle);// - FrcAngle);
		}
		PwmSUM[N] = Distance * AngleProp * (1.0 - TurnValue[N]); //PwmSUM算出処理

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

double MWCtrl::AngleCalc(double _Angle) { //AngleProp算出処理
	return ( _Angle - 45 * ((int)_Angle / 45) ) / (45 - FrcAngle);
}

int MWCtrl::TurnAct(unsigned char L2, unsigned char R2) {
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
}

void MWCtrl::TurnCtrl(int N, unsigned char value) {
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

