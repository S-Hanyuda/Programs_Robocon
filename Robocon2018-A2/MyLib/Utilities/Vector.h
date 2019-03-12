#ifndef VECTOR
#define VECTOR
#include "mbed.h"

class Vector {
public:
	Vector(); //ベクトル関連の定義をまとめておきたい

	struct vector { //ベクトル
		double angle;
		double range;
	};
};

#endif
