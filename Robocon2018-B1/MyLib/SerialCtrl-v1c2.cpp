#include <MyLib/SerialCtrl-v1c2.h>
#include "mbed.h"

SerialCtrl::SerialCtrl(Serial *ptr) {
	_Serial = ptr;
	Timeout = 0;
	sum = 0;
	i = 0;
	signal = false;
	num = 0;
	dable_parity = false;
}

////////////////////
//実行
bool SerialCtrl::get() {
	Timeout = 0;
	while (1) {
		if(Input() == STX) {
			break;
		} else {
			++Timeout;
			if(Timeout > WAIT_COUNT) {
				_FailFunc();
				return false;
			}
		}
	}
	for(i=0; i<9; ++i) data[i] = Input();
	if( (debug1=SUMCheck()) && (debug2=ParityCheck()) && (Input() == ETX) ) { //ParityCheck() && (Input() == ETX) ) {
		_Func();
		signal = true;
		return true;
	} else {
		_FailFunc();
		return false;
	}
}

////////////////////
//設定
void SerialCtrl::Func( void (*func1)(void), void (*func2)(void) ) {
	_Func = func1;
	_FailFunc = func2;
}

void SerialCtrl::isParityDisable(bool flg) {
	dable_parity = flg;
}

////////////////////
//ユーティリティ
void SerialCtrl::probe() {
	if(!signal) {
		_Serial->baud( baudrate[num] );
		if(num >= BAUDS) num = 0;
		else ++num;
	}
}

void SerialCtrl::reset() {
	signal = false;
}

////////////////////
//Private
unsigned char SerialCtrl::Input() {
	Counter.start();
	Counter.reset();
	while (Counter.read_ms() < WAIT) {
		if (_Serial->readable()) {
			return _Serial->getc();
		}
	}
	return NUL;
}

bool SerialCtrl::SUMCheck() {
	sum = 0;
	for(i=0; i<DATA_N-1; ++i) sum += data[i];
	if(data[SUM] == sum) return true;
	return false;
}

bool SerialCtrl::ParityCheck() {
	if(dable_parity) return true;
	for(i=0; i<DATA_N-1; ++i) {
		for(j=0; j<8; ++j) {
			if(data[i] & (0x01 << j)) ++parity;
		}
	}

	if(!(parity%2)) return true;
	return false;
}
