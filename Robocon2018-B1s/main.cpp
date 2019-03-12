//Robocon2018-B1s.cpp
#include <MyLib/MWCtrl-v1-3f.h>
//TODO:B自動マシン

#include "mbed.h"
#define AIR //エアシリ関連

//ライブラリ 宣言
DigitalOut Digi0(P2_6), Digi1(P2_7), Digi2(P2_8), Digi3(P2_11);
PwmOut Pwm0(p26), Pwm1(p25), Pwm2(p24), Pwm3(p23);
MWCtrl Mechanum; //足回り

//デバッグ用LED(プリント基板)
DigitalOut LEDG(p6), LEDY(p7), LEDR(p8);

//CAN通信 宣言
int mistake, Error;
enum{ LX, LY, RX, RY, L2, R2, BA1, BA2, SUM, END }; //データ位置
CAN Mother(p30, p29);
CANMessage msg;
Timer TimeOut;
#define TRIANGLE (0x01 << 0) //ボタンデータ位置マクロ
#define CIRCLE   (0x01 << 1)
#define CROSS    (0x01 << 2)
#define SQUARE   (0x01 << 3)
#define UP       (0x01 << 4)
#define RIGHT    (0x01 << 5)
#define DOWN     (0x01 << 6)
#define LEFT     (0x01 << 7)

#define L1     (0x01 << 0)
#define L3     (0x01 << 1)
#define R1     (0x01 << 2)
#define R3     (0x01 << 3)
#define SELECT (0x01 << 4)
#define START  (0x01 << 5)
#define PS     (0x01 << 6)
#define EMGC   (0x01 << 7)

//エアシリ関連
#ifdef AIR
#define DROP_INTERBAL 0.5 //ボトル投下までの時間(秒)
DigitalOut Pet1(P0_3), Pet2(P0_21); //ペットボトル用エアシリ
DigitalOut Hold(p20), Jack(P0_22), Lock(p18); //合体用エアシリ ホールド・持ち上げ・ロック
Timeout Syrinder;
bool Flg_Lock=false, Flg_pet1=false, Flg_pet2=false;

void Assem(); //合体
void Assem2(); //ロック
void Bottle1(); //ボトル1
void Bottle2(); //ボトル2
#endif

void setup() { //TODO: セットアップ処理
	int i;
	LEDG = LEDY = LEDR = 0;

	//CAN通信
	Mother.frequency(500*1000);
	TimeOut.start();

	Mechanum.DigiPins(&Digi0, &Digi1, &Digi2, &Digi3);
	Mechanum.PwmPins(&Pwm0, &Pwm1, &Pwm2, &Pwm3, 83);
	Mechanum.Trape_Prop(0.0001); //0.0002
	Mechanum.InvldArea(0.075);
	Mechanum.ForceAngle(0);
	for(i=0; i<4; ++i) Mechanum.MoveOffset(i, 1.0);
	/*Mechanum.MoveOffset(0, 1.0);
	Mechanum.MoveOffset(1, 0.98);
	Mechanum.MoveOffset(2, 0.98);
	Mechanum.MoveOffset(3, 1.0);*/
	Mechanum.Emergency();

	#ifdef AIR
	Hold = 0;
	Jack = 1;
	Lock = 0;
	Pet1 = Pet2 = 0;
	#endif
}

void move() {
	//通常移動
	int i;

	Mechanum.Speed();
	/*if(msg.data[BA1] & CROSS) Mechanum.Speed();
	else Mechanum.Speed(0.3);*/

	//通常移動
	//if(msg.data[BA2] &  PS) {
	Mechanum.Move(msg.data[LX], msg.data[LY], msg.data[L2], msg.data[R2]);
}

void actuate() {
	if(msg.data[BA2] & (L1 | R1) ) { //FIXME:デバッグ
		LEDG = 1;
	} else {
		LEDG = 0;
	}

	#ifdef AIR
	if(msg.data[BA2] & L1 && !Hold) { //ホールド・合体
		Hold = 1;
		Syrinder.attach(Assem, 0.5);
	}
	if(msg.data[BA2] & R1 && !Flg_Lock) { //Lock
		Syrinder.attach(Assem2, 1);
		Flg_Lock = true;
	}

	if(msg.data[BA2] & L3 && !Flg_pet1) { //ボトル1
		Syrinder.attach(Bottle1, DROP_INTERBAL);
		Flg_pet1 = true;
	} else if(!(msg.data[BA2] & L3) && Flg_pet1) {
		Syrinder.detach();
		Flg_pet1 = false;
	}
	if(msg.data[BA2] & R3 && !Flg_pet2) { //ボトル2
		Syrinder.attach(Bottle2, DROP_INTERBAL);
		Flg_pet2 = true;
	} else if(!(msg.data[BA2] & R3) && Flg_pet2) {
		Syrinder.detach();
		Flg_pet2 = false;
	}
	/*if(msg.data[BA2] & L3) { //ボトル1
		Pet1 = 1;
	}
	if(msg.data[BA2] & R3) { //ボトル1
		Pet2 = 1;
	}*/

	/*if( msg.data[BA2] & SELECT ) { //下ろす
		Lock = 0;
		Syrinder.attach(Deassem, 0.5);
	}
	if(msg.data[BA2] & START) { //離す
		Hold = 0;
	}*/
	#endif
}

void stopper() { //通信エラー時の動作停止処理
	//Mechanum.Emergency();
	Mechanum.Move(127, 127, 0, 0);
	LEDG = 0;
}

void loop() { //TODO: ループ処理
	/*if( Mother.read(msg) && !(msg.data[BA2] & EMGC) ) {
		move();
		actuate();
		LEDY = 1;
	} else {
		stopper();
		LEDY = 0;
	}*/

	if( Mother.read(msg) ) TimeOut.reset();
	if( TimeOut.read_ms() > 250) { //タイムアウトした場合
		LEDY = 0;
		++mistake;
		if(mistake > 5) {
			stopper();
			LEDR = 1;
		}
	} else if(msg.data[BA2] & EMGC) { //シリアルが切れた場合
		stopper();
		LEDY = 1;
		LEDR = 1;
	} else { //通信成功
		move();
		actuate();
		mistake = 0;
		LEDY = 1;
		LEDR = 0;
	}
}

int main() {
	setup();
	for(;;) {
		loop();
	}
	return 0;
}

#ifdef AIR
void Assem() { //合体
	Jack = 0;
}
void Assem2() { //ロック
	Lock = 1;
}
void Bottle1() { //ボトル1
	Pet1 = 1;
}
void Bottle2() { //ボトル2
	Pet2 = 1;
}
#endif
