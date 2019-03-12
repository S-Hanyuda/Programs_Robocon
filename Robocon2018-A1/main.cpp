//Robocon2018-A1.cpp
//TODO:A手動マシン

#include "mbed.h"
#include "Mylib/SerialCtrl-v1b.h"
#include <MyLib/MWCtrl-v1.h>

//デバッグ用LED(プリント基板)
DigitalOut LEDG(p6), LEDY(p7), LEDR(p8);

//ライブラリ宣言
//シリアル周り
Serial FEP(p13, p14);
SerialCtrl Wire(&FEP); //シリアル通信 ライブラリ宣言
void LEDON(); void LEDOFF(); //シリアル通信 成功・失敗時 関数プロトタイプ宣言
int mistake; //通信失敗カウンタ
//足周り
MWCtrl Mechanum;
DigitalOut Digi0(P2_8), Digi1(P2_11), Digi2(P2_7), Digi3(P2_6);
PwmOut Pwm0(p24), Pwm1(p23), Pwm2(p25), Pwm3(p26);

//エアシリ
Timeout shrink;
DigitalOut Air(p18);
#define SHRINK_TIME 1.0
void Air_off();

void setup() { //TODO: セットアップ処理
	LEDG = LEDY = LEDR = 0;

	FEP.baud(38400);
	Wire.Func( LEDON, LEDOFF ); //シリアル通信成功・失敗時関数

	Air = 0;

	Mechanum.DigiPins(&Digi0, &Digi1, &Digi2, &Digi3);
	Mechanum.PwmPins(&Pwm0, &Pwm1, &Pwm2, &Pwm3, 83);
	Mechanum.Trape_Prop(0.1);
	Mechanum.InvldArea();
	Mechanum.ForceAngle();
}

void move() {
	//通常移動
	if(Wire.data[BA1] & CROSS) {
		Mechanum.Speed(0.3);
	} else {
		Mechanum.Speed();
	}
	Mechanum.Move(Wire.data[LX], Wire.data[LY], Wire.data[L2], Wire.data[R2]);

	/*
	//十字キー移動
	if(msg.data[BA1] & UP) {
		Mechanum.Move(0, 127, 0, 0);
	} else if(msg.data[BA1] & DOWN) {
		Mechanum.Move(255, 127, 0, 0);
	} else if(msg.data[BA1] & RIGHT) {
		Mechanum.Move(127, 255, 0, 0);
	} else if(msg.data[BA1] & LEFT) {
		Mechanum.Move(127, 0, 0, 0);
	} else {
		Mechanum.Move(127, 127, msg.data[L2], msg.data[R2]);
	}*/
}

void actuate() { //動作
	if(Wire.data[BA1] & CIRCLE) {
		Air = 1;
		shrink.attach(Air_off, SHRINK_TIME);
	}
}

void stopper() { //通信エラー時の動作停止処理
	Mechanum.Emergency();
}

void loop() { //TODO: ループ処理

	if( Wire.get() ) {
		move();
		actuate();
		LEDR = 0;
		mistake = 0;
	} else {
		++mistake;
		if (mistake > 2) {
			stopper();
			LEDR = 1;
			//Wire.probe();
		}
	}
}

	int main(void) {
	setup();
	for(;;) {
		loop();
	}
	return 0;
}

void LEDON() { //シリアル通信LED点灯処理
	LEDG = 1;
}

void LEDOFF() { //シリアル通信LED消灯処理
	LEDG = 0;
}

void Air_off() {
	Air = 0;
}
