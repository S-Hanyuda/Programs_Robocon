//高専ロボコン2017マシンプログラム(ハリセンボン)

#include "mbed.h"
#include "MyLib/DataCtrl.h"
#include "MyLib/MoveCtrl.h"

/*ButtonAssign
Serial
|-------|---------------|----------------------
| Byte  |     Data      | Information
|-------|---------------|----------------------
| 0Byte |     0xAF      | Start Germany:Anfang
| 1Byte | unsigned char | LStick X
| 2Byte | unsigned char | LStick Y
| 3Byte | unsigned char | RStick X
| 4Byte | unsigned char | RStick Y
| 5Byte | unsigned char | L2
| 6Byte | unsigned char | R2
| 7Byte | unsigned char | ButtonAssign1
| 8Byte | unsigned char | ButtonAssign2
| 9Byte | unsigned char | CheckSUM
|10Byte |     0xED      | End Germany:Ende
|-------|---------------|----------------------

7Byte ButtonAssign
|-------|--------|-------------
|  Bit  |  Data  | Information
|-------|--------|-------------
| 0Bit  |  i/o   |  TRIANGLE
| 1Bit  |  i/o   |  CIRCLE
| 2Bit  |  i/o   |  CROSS
| 3Bit  |  i/o   |  SQUARE
| 4Bit  |  i/o   |  UP
| 5Bit  |  i/o   |  RIGHT
| 6Bit  |  i/o   |  DOWN
| 7Bit  |  i/o   |  LEFT
|-------|--------|-------------

8Byte ButtonAssign
|-------|--------|-------------
|  Bit  |  Data  | Information
|-------|--------|-------------
| 0Bit  |  i/o   |  L1
| 1Bit  |  i/o   |  L3
| 2Bit  |  i/o   |  R1
| 3Bit  |  i/o   |  R3
| 4Bit  |  i/o   |  SELECT
| 5Bit  |  i/o   |  START
| 6Bit  |  i/o   |  PS
| 7Bit  |  i/o   |  -
|-------|--------|-------------

Method of Operation
|--------|---------------------------
| Action | Button
|--------|---------------------------
|  Move  |  LStick
|  Turn  |  LStick & Circle
|  Slow  |  While press Square
|        |
|        |
|--------|---------------------------
*/

//デバッグ用LED(プリント基板)
DigitalOut LEDG(p6), LEDY(p7), LEDR(p8);

//ライブラリ 宣言
DataCtrl SerialI(p13, p14);
MoveCtrl Move(p24, p23, p25, p26, P2_8, P2_11, P2_7, P2_6);
//MoveCtrl Move(p23, p24, p26, p25, P2_11, P2_8, P2_6, P2_7); //旧版仕様

//足回り速度制御変数
int MoveLevel;
bool MoveFlag, LEDToggleFlag, LEDFlag;

//シリアル通信 宣言
int mistake = 0, Error = 0, count = 0;
unsigned char InputData[10] = {};
enum {LX, LY, RX, RY, L2, R2, BA1, BA2, SUM, END};/*
#define TRIANGLE (0x01 << 0)
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
#define PS     (0x01 << 6)*/

unsigned char HSUM(unsigned char *Data) {
	return Data[LX] + Data[LY] + Data[RX] + Data[RY] + Data[L2] + Data[R2] + Data[BA1] + Data[BA2];
}

//CAN通信 宣言
CAN Slave(p30, p29);
CANMessage msg;
unsigned char McData;

void setup() {
	LEDY = LEDR = 0;
	LEDG = 1;
	Slave.frequency(100*1000);
	Move.Slow(0.5);
	Move.Limiter(0.2);
	Move.Per_SUM(0.025);

	McData = 0;

	MoveLevel = 0;
	MoveFlag = LEDToggleFlag = true;
	LEDFlag = true;
}

bool serial() { //シリアル通信
	Error = 0;
	while (1) {
		if(SerialI.Input() == 0xAF) {
			for(count = LX; count <= END; ++count){
				InputData[count] = SerialI.Input();
			}
			if((InputData[SUM] == HSUM(InputData)) && (InputData[END] == 0xED)) {
				LEDR = 1;
				return true;
				break;
			} else {
				LEDR = 0;
				return false;
				break;
			}
		} else {
			Error ++;
			if(Error > 10) {
				return false;
				break;
			}
		}
	}
}

bool can() { //CAN通信
	Error = 0;
	for(count = LX; count <= BA2; ++count){
		msg.data[count] = InputData[count];
	}
	Error = 0;
	while(1) {
		if(Slave.write(msg)) {
			LEDY = 1;
			return true;
			break;
		} else {
			Error ++;
			if(Error > 10) {
				LEDY = 0;
				return false;
				break;
			}
		}
	}
}

void movement() { //移動関連
	//減速(仮)
	if((InputData[BA2] & R1) || (InputData[BA2] & START)) {
		Move.Slow(0.17);
	} else if(InputData[BA1] & SQUARE) {
		Move.Slow(0.85);
	} else {
		Move.Slow(0.35);
	}

	//移動処理
	if(InputData[BA1] & CIRCLE) { //超信地旋回
		Move.Turn_R(InputData[LX]);
	} else if(InputData[BA1] & TRIANGLE) { //横移動旋回
		Move.SlideTurn_R(InputData[LX], InputData[LY]);
	} else { //全方位制御
		Move.Mechanum(InputData[LX], InputData[LY]);
	}
}

void actuate() {
	if((msg.data[BA1] & CROSS) && LEDToggleFlag) { //速度判別用LED点灯切り替え
		if(LEDFlag) {
			LEDFlag = false;
		} else {
			LEDFlag = true;
		}
		LEDToggleFlag = false;
	} else if(!(msg.data[BA1] & CROSS)) {
		LEDToggleFlag = true;
	}
	if(LEDFlag) {
		LEDG = 1;
	} else {
		LEDG = 0;
	}
}

void safety() { //通信エラー時の動作停止処理
	Move.Emergency();
	LEDG = LEDY = LEDR = 0;
}

void loop() { //ループ処理
	if(serial() && can()) {
		movement();
		actuate();
		mistake = 0;
	} else {
		++mistake;
		if(mistake > 4) {
			safety();
		}
	}
}

int main() {
	setup();
	for(;;) {
		loop();
	}
	return 0;
}

