//Robocon2018-B1.cpp
#include <MyLib/SerialCtrl-v1c2.h>
#include "mbed.h"

#define AUTO   //自動スタート
#define STEP   //自動移動
//#define RETURN //自動帰還

bool Flg_Test=false;

//デバッグ用LED(プリント基板)
DigitalOut LEDG(p6), LEDY(p7), LEDR(p8);
PwmOut TLEDR(p24), TLEDG(p25), TLEDB(p26); //テープLED

//ライブラリ宣言
Serial FEP(p28, p27); //シリアル 宣言
SerialCtrl Wire(&FEP); //シリアル通信 ライブラリ宣言
void LEDON(); void LEDOFF(); //シリアル通信 成功・失敗時 関数プロトタイプ宣言
int mistake; //通信失敗カウンタ
//CAN通信 宣言
CAN Slave(p30, p29);
CANMessage msg;
#define EMGC (0x01 << 7)
//通信系
bool Fail = false;

/////////////////////////////////////////////////////////////////////////////////////
//自動移動フラグ・座標管理
void calculate(); //スティック数値演算
void count_inc(); //カウントインクリメント
void count_return(); //帰還座標指定
void Stage();
void isNext(); //カウント進め判断
bool Flg_AM=false, Flg_Pos=false;    //自動移動・位置判定フラグ
bool Flg_PosX=false, Flg_PosY=false; //座標軸別判定フラグ
bool Flg_isTrigger=false; //センシング チャタリング防止フラグ
bool Flg_Inc=false;       //countインクリメント過多防止
int count=0; //ステージ管理
DigitalIn BorR(P1_18), VorN(P1_21); //モード切り替えスイッチ
Timeout Wait; //待ち時間割り込み
#define DROP_TIME 10  //ボトル投下インターバル
#define MOVE_TIME 1.5 //移動開始インターバル

DigitalIn Hold(P0_27), Jacked(p5), Sensing(P0_28); //マシン接触・持ち上げ検知・センシング
unsigned char Signal; //スレーブ指示送信変数

/////////////////////////////////////////////////////////////////////////////////////
//エンコーダー
#define WHEEL_DIA 38    //オムニ直径
#define PULSE_CYCLE 200 //エンコーダー分解能
#define CONST_M1 128    //移動速度 最低値 ニュートラル
#define CONST_M2 16     //移動速度 最低値 定数1
#define CONST_M3 24     //移動速度 最低値 定数2
#define THRESHOLD 750   //移動速度低下領域

#define TOLERANCE_C 10.0 //座標誤差許容範囲 //5より厳しすぎると微動しまくる
#define TOLERANCE_R 8.9 //回転誤差許容範囲 //8.9でわりと安定するがエンコーダーの機嫌次第 //ジャイロでは不要
#define TURN_SPEED_M 32 //旋回速度 移動中/角度大
#define TURN_SPEED_S 20 //旋回速度 停止中/角度小

InterruptIn diskA0(p11), diskA1(p13); //, diskA2(P0_21); //前・右・後
DigitalIn diskB0(p12), diskB1(p14); //, diskB2(P0_22);

#define ENCODER_N 2 //使用エンコーダー数 0->前 1->右 2->後
int pulse[ENCODER_N] = {}; //パルス数
double range[ENCODER_N] = {}, target[2] = {}; //距離・目標座標
double sub[2]={}; //位置確定用差分

unsigned char AutoX, AutoY; //自動スティック
unsigned char AutoL, AutoR; //回転

void din0_b() { if(diskB0) ++pulse[0]; else if(!diskB0) --pulse[0]; } //前 青
void din0_r() { if(diskB0) --pulse[0]; else if(!diskB0) ++pulse[0]; } //前 赤
void din1() { if(diskB1) ++pulse[1]; else if(!diskB1) --pulse[1]; } //右
//void din2_r() { if(diskB2) ++pulse[2]; else if(!diskB2) --pulse[2]; } //後 赤
//void din2_b() { if(diskB2) --pulse[2]; else if(!diskB2) ++pulse[2]; } //後 青

unsigned char StickCalcX_b(); //通常移動 X軸 青
unsigned char StickCalcX_r(); //通常移動 X軸 赤
unsigned char StickCalcY();   //通常移動 Y軸
unsigned char LineCalcX_b();  //トレース X軸 青
unsigned char LineCalcX_r();  //トレース X軸 青
unsigned char LineCalcY();    //トレース Y軸
unsigned char WallCalcX_b();  //壁検知 青
unsigned char WallCalcX_r();  //壁検知 赤
unsigned char LineDitect();   //白線検知
void TurnCalc(); //旋回演算

/////////////////////////////////////////////////////////////////////////////////////
//補正
DigitalIn sw_l(p23), sw_r(p22), sw_fl(P2_6), sw_fr(p21), sw_bl(P2_7), sw_br(P2_8); //リミットスイッチ
DigitalIn Rotate_0(P1_20), Rotate_1(P1_23), Rotate_2(P1_19); //角度補正bit

#define ANLG_THRE 8.9 //アナログ数値の閾値
//#define LINE_THRE 250 //過去の遺物
AnalogIn Ana_lf(p19), Ana_lb(p20), Ana_rf(p16), Ana_rb(p15); //ライントレース
DigitalIn Table_0(P0_3), Table_1(P0_2); //距離検知
//bool line=false; //過去の遺物
//int lcount=0;    //過去の遺物
#define IS_ANA_LF (Ana_lf.read() > ANLG_THRE) //AnalogIn定義
#define IS_ANA_LB (Ana_lb.read() > ANLG_THRE)
#define IS_ANA_RF (Ana_rf.read() > ANLG_THRE)
#define IS_ANA_RB (Ana_rb.read() > ANLG_THRE)

/////////////////////////////////////////////////////////////////////////////////////
void setup() { //セットアップ処理
	LEDG = LEDY = LEDR = 0; //LED初期化
	TLEDR.period_us(10); TLEDG.period_us(10); TLEDB.period_us(10);
	TLEDR.write(0.0);    TLEDG.write(0.0);    TLEDB.write(0.0);

	//シリアル関連
	FEP.baud(38400);
	Wire.Func( LEDON, LEDOFF ); //シリアル通信成功・失敗時関数
	Wire.isParityDisable(true);
	Slave.frequency(500*1000);

	if(BorR) {
		diskA0.rise(din0_b); //前
		//diskA2.rise(din2_r); //左
	} else {
		diskA0.rise(din0_r); //前
		//diskA2.rise(din2_r); //左
	}
	diskA1.rise(din1); //右
}
/////////////////////////////////////////////////////////////////////////////////////
bool can() { //CAN通信
	int i=0, Error=0;

	Stage();
	calculate();

	for(i=0; i<DATA_N-1; ++i) msg.data[i] = 0; //Lスティックより後ろ
	if(count == 0) {
		msg.data[LX] = 127;
		msg.data[LY] = 127;
		msg.data[L2] = 0;
		msg.data[R2] = 0;
	} else if(Flg_AM) { //Lスティック・L2R2
		msg.data[LX] = AutoX;
		msg.data[LY] = AutoY;
		msg.data[L2] = AutoL;
		msg.data[R2] = AutoR;
	} else {
		msg.data[LX] = Wire.data[LX];
		msg.data[LY] = Wire.data[LY];
		msg.data[L2] = Wire.data[L2];
		msg.data[R2] = Wire.data[R2];
	}

	msg.data[BA2] = Signal;

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
/////////////////////////////////////////////////////////////////////////////////////
void calculate() { //センサ情報処理 赤
	int i;
	if(BorR) {
		diskA0.rise(din0_b); //前
		//diskA2.rise(din2_r); //左
	} else {
		diskA0.rise(din0_r); //前
		//diskA2.rise(din2_r); //左
	}

	//座標算出
	for(i=0; i<ENCODER_N; ++i) range[i] = (double)pulse[i] / PULSE_CYCLE * WHEEL_DIA * M_PI;
	sub[0] = range[0] - target[0];
	sub[1] = range[1] - target[1];

	//マシン位置確定判断
	if(count != 0) {
		if(fabs(sub[0])<TOLERANCE_C || sw_l || sw_r ) Flg_PosX = true;
		else Flg_PosX = false;

		if(fabs(sub[1])<TOLERANCE_C || sw_fl || sw_fr || sw_bl || sw_br ) Flg_PosY = true;
		else Flg_PosY = false;

		if( !Rotate_0 && !Rotate_2 && Flg_PosX && Flg_PosY ) {
			/*range[0] = target[0]; //座標リセット
			range[1] = target[1];
			for(i=0; i<ENCODER_N; ++i) pulse[i] = (int)(range[i] * PULSE_CYCLE / (WHEEL_DIA * M_PI));*/
			Flg_Pos = true;
		} else {
			Flg_Pos = false;
		}
	}

	//TODO:スティック数値算出
	if(BorR) { //青
		if(VorN) { //点稼ぎ
			AutoX = StickCalcX_b();
			AutoY = StickCalcY();
		} else { //Vゴール
			switch(count) {
			case 8: case 11: case 14: //ライン検知
				AutoX = WallCalcX_b();
				AutoY = LineDitect();
				break;
			case 9: case 12: case 15: //ライントレース
				AutoX = LineCalcX_b();
				AutoY = LineCalcY();
				break;
			default: //通常座標移動
				AutoX = StickCalcX_b();
				AutoY = StickCalcY();
			}
		}
	} else { //赤
		if(VorN) { //点稼ぎ
			AutoX = StickCalcX_r();
			AutoY = StickCalcY();
		} else { //Vゴール
			switch(count) {
			case 8: case 11: case 14: //ライン検知
				AutoX = WallCalcX_r();
				AutoY = LineDitect();
				break;
			case 9: case 12: case 15: //ライントレース
				AutoX = LineCalcX_r();
				AutoY = LineCalcY();
				break;
			default: //通常座標移動
				AutoX = StickCalcX_r();
				AutoY = StickCalcY();
			}
		}
	}
	TurnCalc(); //回転補正
	if(Wire.data[BA1] == 255) {
		AutoX = 127;
		AutoY = 127;
		AutoL = 0;
		AutoR = 0;
	}

//各種シグナル
	Signal = 0;
	if(Hold) { //マシン接触検知
		Signal |= L1;
		if(!Jacked) { //マシン持ち上げ完了検知
			Signal |= R1;
		} else {
			Signal &= ~R1;
		}
	} else {
		Signal &= ~L1;
	}
	if(Flg_Pos && count == 1) { //ボトル投下1 //マシン位置が確定した場合にシグナルを送る
		Signal |= L3;
	} else {
		Signal &= ~L3;
	}
	if(VorN) {
		if(Flg_Pos && count == 11) { //ボトル投下2 点稼ぎ 追加コンテンツ前10
			Signal |= R3;
		} else {
			Signal &= ~R3;
		}
	} else {
		if(Flg_Pos && count == 7) { //ボトル投下2 Vゴール
			Signal |= R3;
		} else {
			Signal &= ~R3;
		}
	}
}
/////////////////////////////////////////////////////////////////////////////////////
void actuate() {
	if(count == 3) { //デバッグ Hold || !Jacked || //Sensing
		TLEDR.write(1.0);
		TLEDG.write(1.0);
		TLEDB.write(1.0);
	} else
	if(Flg_Pos) {
		TLEDR.write(0.0);
		TLEDG.write(1.0);
		TLEDB.write(0.0);
	} else if(Flg_AM) {
		TLEDR.write(1.0);
		TLEDG.write(1.0);
		TLEDB.write(0.0);
	} else {
		TLEDR.write(0.0);
		TLEDG.write(0.0);
		TLEDB.write(1.0);
	}
}

void stopper() { //通信エラー時の動作停止処理
	TLEDR.write(1.0);
	TLEDG.write(0.0);
	TLEDB.write(0.0);
}

void loop() {
	/*Flg1 = Wire.get();
	Stage();
	calculate();
	Flg2 = can();*/
	if( Wire.get() && can() ) {
		actuate();
		LEDR = 0;
		mistake = 0;
		Fail = false;
	} else {
		++mistake;
		if (mistake > 6) {
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

void LEDON() { LEDG = 1; } //シリアル通信LED点灯処理
void LEDOFF() { LEDG = 0; } //シリアル通信LED消灯処理

/////////////////////////////////////////////////////////////////////////////////////
void isNext() { //カウント判断
	if(VorN) { //TODO:カウント 点稼ぎ
		switch(count) {
#ifdef AUTO
		case 0:
			if((Hold && !Jacked && !Flg_Inc) || Sensing) {
				Wait.attach(count_inc, MOVE_TIME);
				Flg_Inc = true;
			}
			//else if(!Jacked) Wait.detach();
			break;
#endif
#ifdef STEP
		case 1: case 11:
			if(Flg_Pos && !Flg_Inc) { //(count == 1 || count == 11) &&
				//FIXME:Timeout割り込みのタイミングの都合で、判定が2回発生している可能性が高い
				//それならば動きの説明がつくため、インクリメント2回を防ぐためにカウントの判定も追加した
				Wait.attach(count_inc, MOVE_TIME);
				Flg_Test = true;
				Flg_Inc = true;
			}/* else if(!Flg_Pos) {
				Wait.detach();
				Flg_Inc = false;
			}*/
			break;

		case 2: case 4: case 6: case 7: case 9: case 12: case 13: case 14:
			if(Flg_Test) {
				count = 2;
				Flg_Test = false;
			} else
			if(Flg_Pos && !Flg_Inc) {
				//++count;
				Wait.attach(count_inc, MOVE_TIME);
				Flg_Inc = true;
			}
			break;
#endif
		default:
#ifdef RETURN
			if(Flg_Pos && !Flg_Inc && count != 0) { //帰還
				Wait.attach(count_return, DROP_TIME);
				Flg_Inc = true;
			}
#endif
			if( Sensing && !Flg_isTrigger ) {
				++count;
				Wait.detach();
				Flg_isTrigger = true;
			} else if( !Sensing ) {
				Flg_isTrigger = false;
			}
		}
//////////////
	} else { ////TODO:カウント Vゴール
		switch(count) {
#ifdef AUTO
		case 0:
			if(Hold && !Jacked && !Flg_Inc) {
				Wait.attach(count_inc, 1);
				Flg_Inc = true;
			}
			//else if(!Jacked) Wait.detach();
			break;
#endif
#ifdef STEP
		case 1: case 7:
			if((count == 1 || count == 7) && Flg_Pos && !Flg_Inc) {
				Wait.attach(count_inc, MOVE_TIME);
				Flg_Inc = true;
			}
			break;

		case 2: case 4: case 6: case 8: case 9: case 11: case 12: case 14: case 15: case 17: case 18: case 19: case 20:
		case 101: case 102: case 103: case 201: case 202: case 203: case 204:
			if(Flg_Pos && !Flg_Inc) {
				++count;
				//Wait.attach(count_inc, MOVE_TIME);
				//Flg_Inc = true;
			}
			break;
#endif
		default:
#ifdef RETURN
			if(Flg_Pos && !Flg_Inc && count != 0) { //帰還
				Wait.attach(count_return, DROP_TIME);
				Flg_Inc = true;
			}
#endif
			if( Sensing && !Flg_isTrigger ) {
				++count;
				Wait.detach();
				Flg_isTrigger = true;
			} else if( !Sensing ) {
				Flg_isTrigger = false;
			}
		}
	}
}

void Stage() { //座標登録
	isNext();
	if(VorN) { //TODO:座標 点稼ぎ
		switch(count) {
			case 0:  target[0] = 0;    target[1] = 0;    Flg_AM = false; break; //配置       x    y
			case 1:  target[0] = 545;  target[1] = 15;   Flg_AM = true;  break; //テーブル1   550  0
			case 2:  target[0] = 545;  target[1] = 1185; Flg_AM = true;  break; //テーブル2-1 550  1160 座標を下手に弄ると異常挙動をする
			case 3:  target[0] = 1545; target[1] = 1160; Flg_AM = true;  break; //テーブル2-2 1550 1160
			case 4:  target[0] = 1325; target[1] = 1700; Flg_AM = true;  break; //テーブル3-1 1400
			case 5:  target[0] = 1385; target[1] = 2915; Flg_AM = true;  break; //テーブル3-2 1400 2915
			case 6:  target[0] = 1350; target[1] = 1900; Flg_AM = true;  break; //テーブル3-3 1400 2020
			case 7:  target[0] = 2245; target[1] = 1900; Flg_AM = true;  break; //追加コンテンツ
			case 8:  target[0] = 2245; target[1] = 2010; Flg_AM = true;  break; //テーブル3-4 2245 2020
			case 9:  target[0] = 3250; target[1] = 1950; Flg_AM = true;  break; //テーブル3-5 3190 2020
			case 10: target[0] = 3190; target[1] = 2915; Flg_AM = true;  break; //テーブル3-6 3190 2915
			case 11: target[0] = 3750; target[1] = 1165; Flg_AM = true;  break; //テーブル4   3800 1160
			case 12: target[0] = 3700; target[1] = 1850; Flg_AM = true;  break; //帰還
			case 13: target[0] = -100; target[1] = 1800; Flg_AM = true;  break;
			case 14: target[0] = -100; target[1] = 500;  Flg_AM = true;  break;

			case 101:                   target[1] = 1850; Flg_AM = true;  break; //帰還 イレギュラー
			case 102: target[0] = -100; target[1] = 1850; Flg_AM = true;  break;
			case 103: target[0] = -100; target[1] = 500;  Flg_AM = true;  break;

			default: Flg_AM = false; //count = 0;
		}
	} else { //TODO:座標 Vゴール用
		switch(count) {
			case 0:  target[0] = 0;    target[1] = 0;    Flg_AM = false; break; //配置       x    y
			case 1:  target[0] = 545;  target[1] = 15;   Flg_AM = true;  break; //テーブル1   550  0
			case 2:  target[0] = 545;  target[1] = 1185; Flg_AM = true;  break; //テーブル2-1 550  1160 座標を下手に弄ると異常挙動をする
			case 3:  target[0] = 1545; target[1] = 1160; Flg_AM = true;  break; //テーブル2-2 1550 1160
			case 4:  target[0] = 1540; target[1] = 1700; Flg_AM = true;  break; //テーブル3-1 1550 2020 or1700
			case 5:  target[0] = 2295; target[1] = 2010; Flg_AM = true;  break; //テーブル3-2 2295 2020 斜め可
			case 6:  target[0] = 3250; target[1] = 2000; Flg_AM = true;  break; //テーブル4-1 3800 2020
			case 7:  target[0] = 3750; target[1] = 1165; Flg_AM = true;  break; //テーブル4-2 3800 1160
			case 8:  target[0] = 3800; target[1] = 1180; Flg_AM = true;  break; //追加コンテンツ
			case 9:  target[0] = 3800; target[1] = 4815; Flg_AM = true;  break; //テーブル5-1 3800 4915 トレース
			case 10: target[0] = 1290; target[1] = 4815; Flg_AM = true;  break; //テーブル5-2           トレース
			case 11: target[0] = 3800; target[1] = 4815; Flg_AM = true;  break; //テーブル5-3 3800 4915
			case 12: target[0] = 3800; target[1] = 5915; Flg_AM = true;  break; //テーブル6-1 3800 5915
			case 13: target[0] = 1290; target[1] = 5915; Flg_AM = true;  break; //テーブル6-2           トレース
			case 14: target[0] = 3800; target[1] = 5915; Flg_AM = true;  break; //テーブル6-3 3800 5915
			case 15: target[0] = 3800; target[1] = 6915; Flg_AM = true;  break; //テーブル7-1 3800 6915
			case 16: target[0] = 1290; target[1] = 6915; Flg_AM = true;  break; //テーブル7-2           トレース
			case 17: target[0] = 3800; target[1] = 6915; Flg_AM = true;  break; //テーブル7-3 3800 6915
			case 18: target[0] = 3800; target[1] = 1850; Flg_AM = true;  break; //帰還
			case 19: target[0] = -100; target[1] = 1850; Flg_AM = true;  break;
			case 20: target[0] = -100; target[1] = 500;  Flg_AM = true;  break;

			case 101:                   target[1] = 1850; Flg_AM = true;  break; //帰還 イレギュラー1
			case 102: target[0] = -100; target[1] = 1850; Flg_AM = true;  break;
			case 103: target[0] = -100; target[1] = 500;  Flg_AM = true;  break;

			case 201: target[0] = 3800;                   Flg_AM = true;  break; //帰還 イレギュラー2
			case 202: target[0] = 3800; target[1] = 4100; Flg_AM = true;  break;
			case 203: target[0] = -100; target[1] = 4100; Flg_AM = true;  break;
			case 204: target[0] = -100; target[1] = 500;  Flg_AM = true;  break;

			default: Flg_AM = false; //count = 0;
		}
	}
}

//通常移動
unsigned char StickCalcX_b() { //X軸 青ゾーン
	unsigned char Stick;
	if( (sub[0] > THRESHOLD) ) {
		Stick = 255;
		if(sw_r) Stick = 127;
	} else if( (sub[0] < -THRESHOLD) ) {
		Stick = 0;
		if(sw_l) Stick = 127;
	} else if( (sub[0] > TOLERANCE_C) ) {
		Stick = 127 + (char)(fabs(sub[0]) / THRESHOLD * 127);
		if(sw_r) Stick = 127;
		else if(Stick < CONST_M1+CONST_M2) Stick = CONST_M1+CONST_M2;
	} else if( (sub[0] < -TOLERANCE_C) ) {
		Stick = 127 - (char)(fabs(sub[0]) / THRESHOLD * 127);
		if(sw_l) Stick = 127;
		else if(Stick > CONST_M1-CONST_M2) Stick = CONST_M1-CONST_M2;
	} else {
		Stick = 127;
	}
	return Stick;
}
unsigned char StickCalcX_r() { //X軸 赤ゾーン
	unsigned char Stick;
	if( (sub[0] > THRESHOLD) ) { //&& !sw_l ) {
		Stick = 0;
		if(sw_l) Stick = 127;
	} else if( (sub[0] < -THRESHOLD) ) { //&& !sw_r ) {
		Stick = 255;
		if(sw_r) Stick = 127;
	} else if( (sub[0] > TOLERANCE_C) ) { //&& !sw_l ) {
		Stick = 127 - (char)(fabs(sub[0]) / THRESHOLD * 127);
		if(sw_l) Stick = 127;
		else if(Stick > CONST_M1-CONST_M2) Stick = CONST_M1-CONST_M2;
	} else if( (sub[0] < -TOLERANCE_C) ) { //&& !sw_r ) {
		Stick = 127 + (char)(fabs(sub[0]) / THRESHOLD * 127);
		if(sw_r) Stick = 127;
		else if(Stick < CONST_M1+CONST_M2) Stick = CONST_M1+CONST_M2;
	} else {
		Stick = 127;
	}
	return Stick;
}
unsigned char StickCalcY() { //Y軸
	unsigned char Stick;
	if(sub[1] > THRESHOLD) {
		Stick = 255;
		if(sw_bl || sw_br) Stick = 127;
	} else if(sub[1] < -THRESHOLD) {
		Stick = 0;
		if(sw_fl || sw_fr) Stick = 127;
	} else if(sub[1] > TOLERANCE_C) {
		Stick = 127 + (char)(fabs(sub[1]) / THRESHOLD * 127);
		if(sw_bl || sw_br) Stick = 127;
		else if(Stick < CONST_M1+CONST_M2) Stick = CONST_M1+CONST_M2;
	} else if(sub[1] < -TOLERANCE_C) {
		Stick = 127 - (char)(fabs(sub[1]) / THRESHOLD * 127);
		if(sw_fl || sw_fr) Stick = 127;
		else if(Stick > CONST_M1-CONST_M2) Stick = CONST_M1-CONST_M2;
	} else {
		Stick = 127;
	}
	return Stick;
}

//ライントレース用
unsigned char LineCalcX_b() { //X軸 青ゾーン
	if(!Table_0 && !Table_1 && !sw_r) {
		return CONST_M1+CONST_M2;
	} else if(!Table_0 && Table_1) {
		return 143; //191-48
	} else if(Table_0 && !Table_1) {
		return 191; //255-64
	} else if(Table_0 && Table_1) {
		return 255;
	} else {
		return 127;
	}
}
unsigned char LineCalcX_r() { //X軸 赤ゾーン
	if(!Table_0 && !Table_1 && !sw_l) {
		return CONST_M1-CONST_M2;
	} else if(!Table_0 && Table_1) {
		return 112; //64+48
	} else if(Table_0 && !Table_1) {
		return 64; //0+64
	} else if(Table_0 && Table_1) {
		return 0;
	} else {
		return 127;
	}
}
unsigned char LineCalcY() { //Y軸
	if( (IS_ANA_LF && IS_ANA_LB) || (IS_ANA_RF && IS_ANA_RB) ) {
		return 127;
	} else if(IS_ANA_LF || IS_ANA_RF) { //前移動
		return CONST_M1-CONST_M2;
	} else if(IS_ANA_LB || IS_ANA_RB) { //後ろ移動
		return CONST_M1+CONST_M2;
	} else {
		return 127;
	}
}
unsigned char LineDitect() { //ライン検知
	unsigned char Stick;
	if(range[1] > 4000 && (IS_ANA_LF || IS_ANA_RF)) { //ライン検知
		Stick = 127;
	} else if(sub[1] > THRESHOLD && !Flg_PosY) { //以下通常移動と同じ
		Stick = 255;
	} else if(sub[1] < -THRESHOLD && !Flg_PosY) {
		Stick = 0;
	} else if(sub[1] > TOLERANCE_C && !Flg_PosY) {
		Stick = 127 + (char)(fabs(sub[1]) / THRESHOLD * 127);
		if(AutoY < CONST_M1+CONST_M2) Stick = CONST_M1+CONST_M2;
	} else if(sub[1] < -TOLERANCE_C && !Flg_PosY) {
		Stick = 127 - (char)(fabs(sub[1]) / THRESHOLD * 127);
		if(AutoY > CONST_M1-CONST_M2) Stick = CONST_M1-CONST_M2;
	} else {
		Stick = 127;
	}
	return Stick;
}

unsigned char WallCalcX_b() { //壁 青
	if(!sw_l && (sub[0]>TOLERANCE_C)) {
		return CONST_M1-CONST_M3;
	} else {
		return 127;
	}
}
unsigned char WallCalcX_r() { //壁 赤
	if(!sw_r && (sub[0]>TOLERANCE_C)) {
		return CONST_M1+CONST_M3;
	} else {
		return 127;
	}
}

void TurnCalc() {
	if(Rotate_0 && Rotate_1) {
		AutoL = 0;
		AutoR = TURN_SPEED_M;
	} else if(Rotate_0) {
		AutoL = 0;
		AutoR = TURN_SPEED_S;
	} else if(Rotate_1 && Rotate_2) {
		AutoL = TURN_SPEED_M;
		AutoR = 0;
	} else if(Rotate_2) {
		AutoL = TURN_SPEED_S;
		AutoR = 0;
	} else {
		AutoL = 0; AutoR = 0;
	}
}

void count_inc() {
	++count;
	Flg_Inc = false;
}
void count_return() {
	if(VorN) {
		count = 101;
	} else {
		if(count < 8) {
			count = 101;
		} else {
			count = 201;
		}
	}
	Flg_Inc = false;
}
