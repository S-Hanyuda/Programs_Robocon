#ifndef SERIALCTRL
#define SERIALCTRL
#include "mbed.h"

#define DATA_N 9 //データ数
enum{ LX, LY, RX, RY, L2, R2, BA1, BA2, SUM }; //データ位置
#define TRIANGLE (0x01 << 0)//ボタンデータ位置マクロ
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

class SerialCtrl {
public:
	SerialCtrl(Serial *ptr);
	bool get();

	//設定
	void Func( void (*func1)(void), void (*func2)(void) );
	void isParityDisable(bool);

	//ユーティリティ
	void probe(); //ボーレート探査関数
	void reset(); //ボーレート探査リセット関数

	unsigned char data[DATA_N]; //データ代入変数

private:
	//各種変数など
	Serial *_Serial;   //Serialポインタ
	Timer Counter;     //タイムアウト検知タイマー
	int Timeout, parity;       //タイムアウト回数, パリティチェック
	unsigned char sum; //sumチェック代入
	int i, j;             //forループ用
	#define WAIT 8 //通信待ち時間

	#define BAUDS 14
	const int baudrate[BAUDS] = {110, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 115200, 128000, 256000};
	bool signal; //ボーレート探査用
	int num;

	bool dable_parity;

	//設定
	void (*_Func)(void);     //通信成功時実行関数ポインタ
	void (*_FailFunc)(void); //通信失敗時実行関数ポインタ

	//マクロ
	#define NUL 0x00 //ヌル文字　　 Input()で受信できなかった場合に返す値
	#define STX 0xAF //テキスト開始 先頭パケット
	#define ETX 0xED //テキスト終了 終端パケット
	#define WAIT_COUNT 20

	//ライブラリ内利用関数
	unsigned char Input();    //受信&タイムアウト処理関数
	bool SUMCheck(); //SUMチェック加算関数
	bool ParityCheck();

	//デバッグ変数
	bool debug1, debug2;
};

#endif

/* シリアル通信ライブラリ
 * 簡易BASIC手順方式
 * BaudRateの設定はmain.cpp側で行うため、それを忘れると通信しない
 */

/*
 * ○タスク
 * ・ライブラリ完成 - 2017/12/25完了 - 2018/2/28微修正 - デバッグ待ち - 2018/3/14デバッグ完了
 * ・ボーレート自動検知機能搭載 - デバッグ待ち
 */
