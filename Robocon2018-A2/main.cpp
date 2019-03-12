/*LabProject.cpp
 * 各種ライブラリ等検証用プログラム
 * 本番では使用不可
 */

#include "mbed.h"
#include "Mylib/SerialCtrl-v1b.h"
#include "Mylib/StairCtrl-Servo.h"

//デバッグ用LED(プリント基板)
DigitalOut LEDG(p6), LEDY(p7), LEDR(p8);

//シリアル 宣言
Serial FEP(p13, p14);
SerialCtrl Wire(&FEP); //シリアル通信 ライブラリ宣言
void LEDON(); void LEDOFF(); //シリアル通信 成功・失敗時 関数プロトタイプ宣言
int mistake; //通信失敗カウンタ

//CAN通信 宣言
bool Fail = false;
int i, Error;
CAN Slave(p30, p29);
CANMessage msg;
#define EMGC (0x01 << 7)

//ステア(サーボ)
StairCtrl_S Servo;
PwmOut Pwm0(p23), Pwm1(p24), Pwm2(p25), Pwm3(p26);

//エンコーダー
#define ENCODER_N 3
int pulse[ENCODER_N] = {}; //パルス数
double range[ENCODER_N] = {}; //移動距離
InterruptIn diskA0(P0_3), diskA1(P0_28), diskA2(P0_22); //右前・左後・振り子
DigitalIn diskB0(P0_21), diskB1(P2_13), diskB2(P0_27);
void din0() { if(diskB0) --pulse[0]; else if(!diskB0) ++pulse[0]; } //右前
void din1() { if(diskB1) ++pulse[1]; else if(!diskB1) --pulse[1]; } //左後
void din2() { if(diskB2) --pulse[2]; else if(!diskB2) ++pulse[2]; } //振り子

//座標・ステージ管理
void Stage();
int count=0;

void setup() { //TODO: セットアップ処理
	LEDG = LEDY = LEDR = 0;

	FEP.baud(38400);
	Wire.Func( LEDON, LEDOFF ); //シリアル通信成功・失敗時関数
	Slave.frequency(500*1000);

	Servo.PwmPins(&Pwm0, &Pwm1, &Pwm2, &Pwm3, 15);
	Servo.InvldArea();
}

bool can() { //CAN通信
	Error = 0;
	for(i = LX; i <= BA2; ++i) msg.data[i] = Wire.data[i];
	if(Fail) msg.data[BA2] |= EMGC;
	while(1) {
		if(Slave.write(msg)) {
			LEDY = 1;
			Error = 0;
			return true;
		} else {
			Error ++;
			if(Error > 5) {
				LEDY = 0;
				return false;
			}
		}
	}
}

void move() {
	//Servo.Move(msg.data[LX], msg.data[LY]);
	if(Wire.data[BA1] & LEFT) {
		//Servo.Orient(0.0);
		Servo.Orient(-90.0);
		//Servo.Direct(2033);
	} else if(Wire.data[BA1] & UP) {
		//Servo.Orient(90.0);
		Servo.Orient(0.0);
		//Servo.Direct(1500);
	} else if(Wire.data[BA1] & RIGHT) {
		//Servo.Orient(180);
		Servo.Orient(90);
		//Servo.Direct(967);
	} else if(Wire.data[BA1] & DOWN) {
		//Servo.Orient(270);
		Servo.Orient(180);
		//Servo.Direct(1500);
	} else {
		Servo.Emergency();
	}
}

void actuate() { //動作

}

void stopper() { //通信エラー時の動作停止処理

}

void loop() { //TODO: ループ処理
	if( can() && Wire.get() ) {
		move();
		actuate();
		LEDR = 0;
		mistake = 0;
		Fail = false;
	} else {
		++mistake;
		if (mistake > 2) {
			stopper();
			LEDR = 1;
			Fail = true;
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
