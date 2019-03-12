#include <MyLib/SerialCtrl-v1b.h>
#include "mbed.h"

SerialCtrl::SerialCtrl(Serial *ptr) {
	_Serial = ptr;
	Timeout = 0;
	sum = 0;
	i = 0;
	signal = false;
	num = 0;
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
			if(Timeout > 25) {
				_FailFunc();
				return false;
			}
		}
	}
	for(i=0; i<9; ++i) data[i] = Input();
	if( (data[SUM] == SUMCheck()) && (Input() == ETX) ) {
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

unsigned char SerialCtrl::SUMCheck() {
	sum = 0;
	for(i=0; i<=DATA_N-2; ++i) sum += data[i];
	return sum;
}
