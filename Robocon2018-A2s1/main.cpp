/*LabProject-CAN.cpp
 * 各種ライブラリ等検証用プログラム
 * 本番では使用不可
 */

#include "mbed.h"
#include "MyLib/StairCtrl-Motor.h"

//ライブラリ 宣言
//ステア(モーター)
StairCtrl_M Motor;
DigitalOut Digi0(P2_8), Digi1(P2_11), Digi2(P2_7), Digi3(P2_6);
PwmOut Pwm0(p24), Pwm1(p23), Pwm2(p25), Pwm3(p26);

//デバッグ用LED(プリント基板)
DigitalOut LEDG(p6), LEDY(p7), LEDR(p8);

//CAN通信 宣言
int mistake, Error;
enum{ LX, LY, RX, RY, L2, R2, BA1, BA2, SUM, END }; //データ位置
CAN Mother(p30, p29);
CANMessage msg;
Timer TimeOut;
//ボタンデータ位置マクロ
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
#define EMGC   (0x01 << 7)

//BMX055
//#define GYRO1
#ifdef GYRO1
I2C Gyro(p9, p10);
char g_data[2];
double zGyro;
char reg[2]={0x06, 0x07};
#define GYRO_ADDR 0x69 //ジャイロアドレス
#define GYRO_REG1 0x0F //レジスタ
#define GYRO_REG2 0x10
#define GYRO_REG3 0x11
#define GYRO_DAT1 0x03 //データ //標準0x04
#define GYRO_DAT2 0x06 //標準0x07
#define GYRO_DAT3 0x00
#endif

void setup() { //TODO: セットアップ処理
	LEDG = LEDY = LEDR = 0;

	Mother.frequency(500*1000);
	TimeOut.start();

	Motor.DigiPins(&Digi0, &Digi1, &Digi2, &Digi3);
	Motor.PwmPins(&Pwm0, &Pwm1, &Pwm2, &Pwm3, 83);
	//Motor.Trape_Prop(0.0005);
	Motor.InvldArea();
	Motor.ForceAngle();
}

void move() {
	//通常移動
	if(msg.data[BA1] & CROSS) {
		Motor.Speed(0.25);
	} else {
		Motor.Speed();
	}

	//通常移動
	Motor.Move(msg.data[LX], msg.data[LY], msg.data[L2], msg.data[R2]);
}

void actuate() {

}

void stopper() {

}

void loop() { //TODO: ループ処理

	if( Mother.read(msg) ) TimeOut.reset();
	if( TimeOut.read_ms() > 200) { //タイムアウトした場合
		LEDY = 0;
		++mistake;
		if(mistake > 10) {
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

#ifdef GYRO1
void gyro_setup() {
	char gyro_data[2]={};
	Gyro.frequency(100*1000);

	gyro_data[0] = GYRO_REG1;
	gyro_data[1] = GYRO_DAT1;
	printf("reg1:%d\n", Gyro.write(GYRO_ADDR, gyro_data, 2, false));
	wait_ms(100);
	gyro_data[0] = GYRO_REG2;
	gyro_data[1] = GYRO_DAT2;
	printf("reg2:%d\n", Gyro.write(GYRO_ADDR, gyro_data, 2, false));
	wait_ms(100);
	gyro_data[0] = GYRO_REG3;
	gyro_data[1] = GYRO_DAT3;
	printf("reg3:%d\n", Gyro.write(GYRO_ADDR, gyro_data, 2, false));
	wait_ms(100);
}
void gyro_loop() {
	for (int i = 0; i < 2; i++) {
		printf("%d w:%d ", i, Gyro.write(GYRO_ADDR, &reg[i], 1, false));
		printf("r:%d\n", Gyro.read(GYRO_ADDR, &g_data[i], 1, false));
	}
	zGyro = (g_data[1] * 256) + g_data[0];
	if (zGyro > 32767)  zGyro -= 65536;
	zGyro = zGyro * 0.0038;

	printf("%f\n", zGyro);
	wait(1);
}
#endif

int main() {
#ifndef GYRO1
	setup();
	for(;;) {
		loop();
	}
#else
	gyro_setup();
	for(;;) gyro_loop();
#endif
	return 0;
}

