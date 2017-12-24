//ロボコン2017マシンプログラム(ハリセンボン・CAN通信受信側)

#include "mbed.h"

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

//ライブラリ 宣言
	//CANのみのため無し

//デバッグ用LED(プリント基板)
DigitalOut LEDG(p6), LEDY(p7), LEDR(p8);
//フルカラーテープLED
PwmOut TLEDR(p23), TLEDG(p22), TLEDB(p21);
//足回り速度切り替え判別用変数
float LEDLevel;
bool LEDTypeFlag, LEDToggleFlag, LEDFlag;
int LEDType;

//CAN通信 宣言
int mistake, Error;
enum{LX, LY, RX, RY, L2, R2, BA1, BA2};
CAN Mother(p30, p29);
CANMessage msg;
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
#define PS     (0x01 << 6)

//竿制御 宣言
DigitalOut Rod(P2_11), Rock(P2_8);
Timeout Timing;
bool RodFlag = true;

//腕制御 宣言
DigitalOut ArmMD(P2_6);
PwmOut ArmMP(p26);
AnalogIn ArmMA(p18);
float ArmMAv;
bool ArmFlag;


void setup() {
	LEDG = LEDY = LEDR = 0;
	TLEDR.period_ms(20);
	TLEDG.period_ms(20);
	TLEDB.period_ms(20);
	TLEDR.write(0.0);
	TLEDG.write(0.0);
	TLEDB.write(0.0);
	Rod = Rock = 0;
	RodFlag = true;
	Mother.frequency(100*1000);

	ArmMD = 0;
	ArmMP.period_ms(1);
	ArmMP.write(0.0);
	ArmMAv = 0.0;
	ArmFlag = true;

	LEDLevel = 0.6;
	LEDToggleFlag = true;
	LEDFlag = true;
	LEDType = 0;
}

bool can() { //CAN通信
	Error = 0;
	while(1) {
		if(Mother.read(msg)) {
			LEDY = 1;
			return true;
			break;
		} else {
			Error ++;
			if(Error > 20) {
				LEDY = 0;
				return false;
				break;
			}
		}
	}
}

void burst() { //竿展開割り込み関数
	Rod = 1;
	TLEDB.write(1.0);
}

void actuate() {
	//速度判別用LED点灯強度切り替え
	if((msg.data[BA2] & R1) || (msg.data[BA2] & START)) {
		//条件分岐の優先度をマザーと合わせるための分岐
		LEDLevel = 0.0;
	} else if(msg.data[BA1] & SQUARE) {
		LEDG = 1;
		LEDLevel = 1.0;
	} else {
		LEDG = 0;
		LEDLevel = 0.1;
	}

	if((msg.data[BA1] & CROSS) && LEDToggleFlag) { //速度判別用LED点灯切り替え
		if(LEDFlag) {
			LEDFlag = false;
		} else {
			LEDFlag = true;
		}
		LEDToggleFlag = false;
	} else if(!(msg.data[BA1] & CROSS)){
		LEDToggleFlag = true;
	}
	if(LEDFlag) {
		LEDG = 1;
		TLEDG.write(LEDLevel);
	} else {
		LEDG = 0;
		TLEDG.write(0.0);
	}

	if((msg.data[BA2] & R1) || (msg.data[BA2] & START)) { //竿用電磁弁セーフティ
		TLEDR.write(1.0);
	} else {
		TLEDR.write(0.0);
	}
	if((msg.data[BA2] & R1) && (msg.data[BA2] & START)) { //竿用電磁弁解放
		Rock = 1;
		if(RodFlag) {
			Timing.attach(burst, 0.5);
			RodFlag = false;
		}
	} else if(msg.data[BA2] & SELECT) { //電磁弁閉鎖
		Rock = 0;
		Rod = 0;
		TLEDB.write(0.0);
		RodFlag = true;
	}

	//剣振り
	ArmMAv = ArmMA.read();
	if(msg.data[BA2] & L1) { //ニュートラルに補正
		if(ArmMAv > 0.53) { //右側のとき
			ArmMD = 0;
			ArmMP.write(0.3);
		} else if(ArmMAv < 0.47) { //左側のとき
			ArmMD = 1;
			ArmMP.write(0.7);
		} else {
			ArmMD = 0;
			ArmMP.write(0.0);
		}
	} else {
		if((msg.data[R2] >= 30) && (msg.data[L2] >= 30)) { //交互振り
			if(ArmFlag) {
				ArmMD = 0;
				ArmMP.write(0.55);
				if(ArmMAv < 0.38) {
					ArmFlag = false;
				}
			} else {
				ArmMD = 1;
				ArmMP.write(0.45);
				if(ArmMAv > 0.63) {
					ArmFlag = true;
				}
			}
		} else if((msg.data[R2] >= 30) && (ArmMAv > 0.38)) { //右振り
			ArmMD = 0;
			ArmMP.write(0.55);
		} else if((msg.data[L2] >= 30) && (ArmMAv < 0.63)) { //左振り
			ArmMD = 1;
			ArmMP.write(0.45);
		} else {
			ArmMP.write(0.0);
			ArmMD = 0;
		}
	}
}

void safety() { //通信エラー時の動作停止処理
	Rod = Rock = 0;
}

void loop() { //ループ処理
	mistake = 0;
	if(can()) {
		actuate();
	} else {
		mistake ++;
		if(mistake > 2) {
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

