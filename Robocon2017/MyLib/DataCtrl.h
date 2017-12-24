#ifndef DATACTRL
#define DATACTRL
#include "mbed.h"

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

class DataCtrl {
public:
	DataCtrl(PinName tx, PinName rx);
	unsigned char Input();
	void Input_test();
	unsigned char SUM(unsigned char *DATA, int n);

	unsigned char InData[10] = {};
private:
	Serial _SerialI;
	unsigned char sum;
	int count, Error;
};

#endif
