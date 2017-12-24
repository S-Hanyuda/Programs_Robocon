#include "mbed.h"
#include "MyLib/DataCtrl.h"

Timer Counter;

DataCtrl::DataCtrl(PinName tx, PinName rx) :_SerialI(tx, rx) {
	_SerialI.baud(38400);
	sum = count = 0;
}

unsigned char DataCtrl::Input() {
	Counter.start();
	Counter.reset();
	while (Counter.read_ms() < 3) {
		if (_SerialI.readable()) {
			return _SerialI.getc();
		}
	}
}

void DataCtrl::Input_test() {
}

unsigned char DataCtrl::SUM_Check(unsigned char *DATA, int n) { //ガバがあるので利用停止
	for(count = 0; count <= n; ++count) {
		sum += DATA[count];
	}
	return sum;
}
