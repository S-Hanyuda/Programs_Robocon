#include "mbed.h"
#include "MyLib/MoveCtrl.h"

MoveCtrl::MoveCtrl(PinName P1, PinName P2, PinName P3, PinName P4, PinName D1, PinName D2, PinName D3, PinName D4) :_P1(P1), _P2(P2), _P3(P3), _P4(P4), _D1(D1), _D2(D2), _D3(D3), _D4(D4) {
	_P1.period_us(83); //Right or RightFront
	_P2.period_us(83); //Front or LeftFront
	_P3.period_us(83); //Left or LeftBack
	_P4.period_us(83); //Back or RightBack
	_P1.write(0);
	_P2.write(0);
	_P3.write(0);
	_P4.write(0);
	_D1 = _D2 = _D3 = _D4 = 0;
	PwmX = PwmY = PwmM = PwmC = PwmSUM1 = PwmSUM2 = per1 = per2 = 0.0;
	Restrict = 1.0;
	LowLimit = 0.1;
	SUM_per = 0.05;
	for(reseter = 0; reseter < 2; ++reseter) { //float[2]
		Trape1[reseter] = Trape2[reseter] = Trape3[reseter] = Trape4[reseter] = 0.0;
	}
	for(reseter = 0; reseter < 4; ++reseter) { //float[4]
		PwmSUM[reseter] = 0.0;
	}
	for(reseter = 0; reseter < 4; ++reseter) { //int[4]
		Digital[reseter] = Digital_old[reseter] = 0;
	}
}

void MoveCtrl::Mechanum(unsigned char JoyX, unsigned char JoyY) { //メカナム・オムニ(X字)制御 台形・横旋回移動対応
	PwmM = hypotf(sub(JoyX), sub(JoyY)) / 128.0; //PwmMaster算出

	if(PwmM >= LowLimit) { //傾きにより動かすかを判別
		//モーター1制御
		if(JoyX <= sub2(JoyY)) { //スティックが左上側の場合
			Digital[0] = 1;
			if((Digital[0] != Digital_old[0]) && (Trape1[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape1[0] -= SUM_per;
				Trape1[1] = 1 - Trape1[0];
				_P1.write(Trape1[ Digital_old[0] ]);
			} else {
				//動いてない場合
				_D1 = Digital_old[0] = Digital[0];
				if(JoyX > 127) { //per1算出
					per1 = fabs(sub3(JoyY) / sub(JoyX)) - 1.0;
				} else if(JoyY > 127) {
					per1 = 1.0 - fabs(sub(JoyY) / sub(JoyX));
				} else {
					per1 = 1.0;
				}
				PwmSUM[0] = PwmM * per1;
				if((Trape1[0] > PwmSUM[0]) || (Trape1[0] > Restrict)) {
					Trape1[0] -= SUM_per;
					Trape1[1] = 1 - Trape1[0];
				} else {
					Trape1[0] += SUM_per;
					Trape1[1] = 1 - Trape1[0];
				}
				_P1.write(Trape1[1]);
			}
		} else if(JoyX > sub2(JoyY)) { //スティックが右下側の場合
			Digital[0] = 0;
			if((Digital[0] != Digital_old[0]) && (Trape1[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape1[0] -= SUM_per;
				Trape1[1] = 1 - Trape1[0];
				_P1.write(Trape1[ Digital_old[0] ]);
			} else {
				//動いてない場合
				_D1 = Digital_old[0] = Digital[0];
				if(JoyY <= 127) { //per算出
					per1 = 1.0 - fabs(sub3(JoyY) / sub(JoyX));
				} else if(JoyX <= 127) {
					per1 = fabs(sub(JoyY) / sub(JoyX)) - 1.0;
				} else {
					per1 = 1.0;
				}
				PwmSUM[0] = PwmM * per1;
				if((Trape1[0] > PwmSUM[0]) || (Trape1[0] > Restrict)) {
					Trape1[0] -= SUM_per;
				} else {
					Trape1[0] += SUM_per;
				}
				_P1.write(Trape1[0]);
			}
		}

		//モーター3制御
		if(JoyX <= sub2(JoyY)) { //スティックが左上側の場合
			Digital[2] = 0;
			if((Digital[2] != Digital_old[2]) && (Trape3[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape3[0] -= SUM_per;
				Trape3[1] = 1 - Trape1[0];
				_P3.write(Trape3[ Digital_old[2] ]);
			} else {
				//動いてない場合
				_D3 = Digital_old[2] = Digital[2];
				if(JoyX > 127) { //per1算出
					per1 = fabs(sub3(JoyY) / sub(JoyX)) - 1.0;
				} else if(JoyY > 127) {
					per1 = 1.0 - fabs(sub(JoyY) / sub(JoyX));
				} else {
					per1 = 1.0;
				}
				PwmSUM[2] = PwmM * per1;
				if((Trape3[0] > PwmSUM[2]) || (Trape3[0] > Restrict)) {
					Trape3[0] -= SUM_per;
				} else {
					Trape3[0] += SUM_per;
				}
				_P3.write(Trape3[0]);
			}
		} else if(JoyX > sub2(JoyY)) { //スティックが右下側の場合
			Digital[2] = 1;
			if((Digital[2] != Digital_old[2]) && (Trape3[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape3[0] -= SUM_per;
				Trape3[1] = 1 - Trape3[0];
				_P3.write(Trape3[ Digital_old[2] ]);
			} else {
				//動いてない場合
				_D3 = Digital_old[2] = Digital[2];
				if(JoyY <= 127) { //per算出
					per1 = 1.0 - fabs(sub3(JoyY) / sub(JoyX));
				} else if(JoyX <= 127) {
					per1 = fabs(sub(JoyY) / sub(JoyX)) - 1.0;
				} else {
					per1 = 1.0;
				}
				PwmSUM[2] = PwmM * per1;
				if((Trape3[0] > PwmSUM[2]) || (Trape3[0] > Restrict)) {
					Trape3[0] -= SUM_per;
					Trape3[1] = 1 - Trape3[0];
				} else {
					Trape3[0] += SUM_per;
					Trape3[1] = 1 - Trape3[0];
				}
				_P3.write(Trape3[1]);
			}
		}

		//モーター2制御
		if(JoyX >= JoyY) { //スティックが右上側の場合
			Digital[1] = 0;
			if((Digital[1] != Digital_old[1]) && (Trape2[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape2[0] -= SUM_per;
				Trape2[1] = 1 - Trape2[0];
				_P2.write(Trape2[ Digital_old[1] ]);
			} else {
				//動いてない場合
				_D2 = Digital_old[1] = Digital[1];
				if(JoyX <= 127) { //per2算出
					per2 = fabs(sub3(JoyY) / sub(JoyX)) - 1.0;
				} else if(JoyY > 127) {
					per2 = 1.0 - fabs(sub(JoyY) / sub(JoyX));
				} else {
					per2 = 1.0;
				}
				PwmSUM[1] = PwmM * per2;
				if((Trape2[0] > PwmSUM[1]) || (Trape2[0] > Restrict)) {
					Trape2[0] -= SUM_per;
				} else {
					Trape2[0] += SUM_per;
				}
				_P2.write(Trape2[0]);
			}
		} else if(JoyX < JoyY) { //スティックが左下側の場合
			Digital[1] = 1;
			if((Digital[1] != Digital_old[1]) && (Trape2[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape2[0] -= SUM_per;
				Trape2[1] = 1 - Trape2[0];
				_P2.write(Trape2[ Digital_old[1] ]);
			} else {
				//動いてない場合
				_D2 = Digital_old[1] = Digital[1];
				if(JoyY <= 127) { //per2算出
					per2 = 1.0 - fabs(sub3(JoyY) / sub(JoyX));
				} else if(JoyX > 127) {
					per2 = fabs(sub(JoyY) / sub(JoyX)) - 1.0;
				} else {
					per2 = 1.0;
				}
				PwmSUM[1] = PwmM * per2;
				if((Trape2[0] > PwmSUM[1]) || (Trape2[0] > Restrict)) {
					Trape2[0] -= SUM_per;
					Trape2[1] = 1 - Trape2[0];
				} else {
					Trape2[0] += SUM_per;
					Trape2[1] = 1 - Trape2[0];
				}
				_P2.write(Trape2[1]);
			}
		}

		//モーター4制御
		if(JoyX >= JoyY) { //スティックが右上の場合
			Digital[3] = 1;
			if((Digital[3] != Digital_old[3]) && (Trape4[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape4[0] -= SUM_per;
				Trape4[1] = 1 - Trape4[0];
				_P4.write(Trape4[ Digital_old[3] ]);
			} else {
				//動いてない場合
				_D4 = Digital_old[3] = Digital[3];
				if(JoyX <= 127) { //per2算出
					per2 = fabs(sub3(JoyY) / sub(JoyX)) - 1.0;
				} else if(JoyY > 127) {
					per2 = 1.0 - fabs(sub(JoyY) / sub(JoyX));
				} else {
					per2 = 1.0;
				}
				PwmSUM[3] = PwmM * per2;
				if((Trape4[0] > PwmSUM[3]) || (Trape4[0] > Restrict)) {
					Trape4[0] -= SUM_per;
					Trape4[1] = 1 - Trape4[0];
				} else {
					Trape4[0] += SUM_per;
					Trape4[1] = 1 - Trape4[0];
				}
				_P4.write(Trape4[1]);
			}
		} else if(JoyX < JoyY) { //スティックが左下の場合
			Digital[3] = 0;
			if((Digital[3] != Digital_old[3]) && (Trape4[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape4[0] -= SUM_per;
				Trape4[1] = 1 - Trape4[0];
				_P4.write(Trape4[ Digital_old[3] ]);
			} else {
				//動いてない場合
				_D4 = Digital_old[3] = Digital[3];
				if(JoyY <= 127) { //per2算出
					per2 = 1.0 - fabs(sub3(JoyY) / sub(JoyX));
				} else if(JoyX > 127) {
					per2 = fabs(sub(JoyY) / sub(JoyX)) - 1.0;
				} else {
					per2 = 1.0;
				}
				PwmSUM[3] = PwmM * per2;
				if((Trape4[0] > PwmSUM[3]) || (Trape4[0] > Restrict)) {
					Trape4[0] -= SUM_per;
				} else {
					Trape4[0] += SUM_per;
				}
				_P4.write(Trape4[0]);
			}
		}
	} else { //動かさないとき
		_D1 = Digital_old[0];
		_D3 = Digital_old[2];
		_D2 = Digital_old[1];
		_D4 = Digital_old[3];
		if(Trape1[0] > 0.0) {
			Trape1[0] -= SUM_per;
			Trape1[1] = 1 - Trape1[0];
		}
		if(Trape3[0] > 0.0) {
			Trape3[0] -= SUM_per;
			Trape3[1] = 1 - Trape3[0];
		}
		if(Trape2[0] > 0.0) {
			Trape2[0] -= SUM_per;
			Trape2[1] = 1 - Trape2[0];
		}
		if(Trape4[0] > 0.0) {
			Trape4[0] -= SUM_per;
			Trape4[1] = 1 - Trape4[0];
		}
		_P1.write(Trape1[Digital_old[0]]);
		_P3.write(Trape3[Digital_old[2]]);
		_P2.write(Trape2[Digital_old[1]]);
		_P4.write(Trape4[Digital_old[3]]);
	}
}

void MoveCtrl::OmniK(unsigned char JoyX, unsigned char JoyY){ //オムニ(十字)制御
	PwmX = fabs(sub(JoyX) / 128.0) * Restrict;
	PwmY = fabs(sub(JoyY) / 128.0) * Restrict;

	if(JoyX > 140) { //X軸方向制御
		_D2 = 0;
		_D4 = 1;
		_P2.write(PwmX);
		_P4.write(1.0 - PwmX);
	} else if(JoyX < 114) {
		_D2 = 1;
		_D4 = 0;
		_P2.write(1.0 - PwmX);
		_P4.write(PwmX);
	} else {
		_D2 = _D4 = 0;
		_P2.write(0.0);
		_P4.write(0.0);
	}

	if(JoyY > 140) { //Y軸方向制御
		_D1 = 1;
		_D3 = 0;
		_P1.write(1.0 - PwmY);
		_P3.write(PwmY);
	} else if(JoyY < 114) {
		_D1 = 0;
		_D3 = 1;
		_P1.write(PwmY);
		_P3.write(1.0 - PwmY);
	} else {
		_D1 = _D3 = 0;
		_P1.write(0.0);
		_P3.write(0.0);
	}
}

void MoveCtrl::Omni3(unsigned char JoyX, unsigned char JoyY){ //3軸オムニ制御
	//面倒なので未実装
}

void MoveCtrl::Turn(unsigned char JoyX){ //旋回制御(左右反転) 台形・・横旋回移動対応
	//ギアを噛ませている場合、ピン宣言が対角線で入れ替わってしまうのでそれの対応用
	PwmM = fabs(sub(JoyX) / 128.0); //PwmMaster算出
	if(PwmM >= LowLimit) { //傾きから動かすか検出
		if(JoyX > 127) { //右旋回の場合
			for(i = 0; i < 4; ++i){
				Digital[i] = 0;
			}
			//モーター1制御
			if((Digital[0] != Digital_old[0]) && (Trape1[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape1[0] -= SUM_per;
				Trape1[1] = 1 - Trape1[0];
				_P1.write(Trape1[ Digital_old[0] ]);
			} else {
				//動いてない場合
				_D1 = Digital_old[0] = Digital[0];
				if((Trape1[0] > PwmM) || (Trape1[0] > Restrict)) {
					Trape1[0] -= SUM_per;
				} else {
					Trape1[0] += SUM_per;
				}
				_P1.write(Trape1[0]);
			}

			//モーター3制御
			if((Digital[2] != Digital_old[2]) && (Trape3[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape3[0] -= SUM_per;
				Trape3[1] = 1 - Trape3[0];
				_P3.write(Trape3[ Digital_old[2] ]);
			} else {
				//動いてない場合
				_D3 = Digital_old[2] = Digital[2];
				if((Trape3[0] > PwmM) || (Trape3[0] > Restrict)) {
					Trape3[0] -= SUM_per;
				} else {
					Trape3[0] += SUM_per;
				}
				_P3.write(Trape3[0]);
			}

			//モーター2制御
			if((Digital[1] != Digital_old[1]) && (Trape2[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape2[0] -= SUM_per;
				Trape2[1] = 1 - Trape2[0];
				_P2.write(Trape2[ Digital_old[1] ]);
			} else {
				//動いてない場合
				_D2 = Digital_old[1] = Digital[1];
				if((Trape2[0] > PwmM) || (Trape2[0] > Restrict)) {
					Trape2[0] -= SUM_per;
				} else {
					Trape2[0] += SUM_per;
				}
				_P2.write(Trape2[0]);
			}

			//モーター4制御
			if((Digital[3] != Digital_old[3]) && (Trape4[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape4[0] -= SUM_per;
				Trape4[1] = 1 - Trape4[0];
				_P4.write(Trape4[ Digital_old[3] ]);
			} else {
				//動いてない場合
				_D4 = Digital_old[3] = Digital[3];
				if((Trape4[0] > PwmM) || (Trape4[0] > Restrict)) {
					Trape4[0] -= SUM_per;
				} else {
					Trape4[0] += SUM_per;
				}
				_P4.write(Trape4[0]);
			}
		} else if(JoyX <= 127) { //左旋回の場合
			for(i = 0; i < 4; ++i){
				Digital[i] = 1;
			}
			//モーター1制御
			if((Digital[0] != Digital_old[0]) && (Trape1[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape1[0] -= SUM_per;
				Trape1[1] = 1 - Trape1[0];
				_P1.write(Trape1[ Digital_old[0] ]);
			} else {
				//動いてない場合
				_D1 = Digital_old[0] = Digital[0];
				if((Trape1[0] > PwmM) || (Trape1[0] > Restrict)) {
					Trape1[0] -= SUM_per;
					Trape1[1] = 1 - Trape1[0];
				} else {
					Trape1[0] += SUM_per;
					Trape1[1] = 1 - Trape1[0];
				}
				_P1.write(Trape1[1]);
			}

			//モーター3制御
			if((Digital[2] != Digital_old[2]) && (Trape3[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape3[0] -= SUM_per;
				Trape3[1] = 1 - Trape3[0];
				_P3.write(Trape3[ Digital_old[2] ]);
			} else {
				//動いてない場合
				_D3 = Digital_old[2] = Digital[2];
				if((Trape3[0] > PwmM) || (Trape3[0] > Restrict)) {
					Trape3[0] -= SUM_per;
					Trape3[1] = 1 - Trape3[0];
				} else {
					Trape3[0] += SUM_per;
					Trape3[1] = 1 - Trape3[0];
				}
				_P3.write(Trape3[1]);
			}

			//モーター2制御
			if((Digital[1] != Digital_old[1]) && (Trape2[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape2[0] -= SUM_per;
				Trape2[1] = 1 - Trape2[0];
				_P2.write(Trape2[ Digital_old[1] ]);
			} else {
				//動いてない場合
				_D2 = Digital_old[1] = Digital[1];
				if((Trape2[0] > PwmM) || (Trape2[0] > Restrict)) {
					Trape2[0] -= SUM_per;
					Trape2[1] = 1 - Trape2[0];
				} else {
					Trape2[0] += SUM_per;
					Trape2[1] = 1 - Trape2[0];
				}
				_P2.write(Trape2[1]);
			}

			//モーター4制御
			if((Digital[3] != Digital_old[3]) && (Trape4[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape4[0] -= SUM_per;
				Trape4[1] = 1 - Trape4[0];
				_P4.write(Trape4[ Digital_old[3] ]);
			} else {
				//動いてない場合
				_D4 = Digital_old[3] = Digital[3];
				if((Trape4[0] > PwmM) || (Trape4[0] > Restrict)) {
					Trape4[0] -= SUM_per;
					Trape4[1] = 1 - Trape4[0];
				} else {
					Trape4[0] += SUM_per;
					Trape4[1] = 1 - Trape4[0];
				}
				_P4.write(Trape4[1]);
			}
		}
	} else { //動かさないとき
		_D1 = Digital_old[0];
		_D3 = Digital_old[2];
		_D2 = Digital_old[1];
		_D4 = Digital_old[3];
		if(Trape1[0] > 0.0) {
			Trape1[0] -= SUM_per;
			Trape1[1] = 1 - Trape1[0];
		}
		if(Trape3[0] > 0.0) {
			Trape3[0] -= SUM_per;
			Trape3[1] = 1 - Trape3[0];
		}
		if(Trape2[0] > 0.0) {
			Trape2[0] -= SUM_per;
			Trape2[1] = 1 - Trape2[0];
		}
		if(Trape4[0] > 0.0) {
			Trape4[0] -= SUM_per;
			Trape4[1] = 1 - Trape4[0];
		}
		_P1.write(Trape1[ Digital_old[0] ]);
		_P3.write(Trape3[ Digital_old[2] ]);
		_P2.write(Trape2[ Digital_old[1] ]);
		_P4.write(Trape4[ Digital_old[3] ]);
	}
}

void MoveCtrl::Turn_R(unsigned char JoyX){ //旋回制御(左右反転) 台形・・横旋回移動対応
	//ギアを噛ませている場合、ピン宣言が対角線で入れ替わってしまうのでそれの対応用
	PwmM = fabs(sub(JoyX) / 128.0); //PwmMaster算出
	if(PwmM >= LowLimit) { //傾きから動かすか検出
		if(JoyX > 127) { //右旋回の場合
			for(i = 0; i < 4; ++i){
				Digital[i] = 1;
			}
			//モーター1制御
			if((Digital[0] != Digital_old[0]) && (Trape1[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape1[0] -= SUM_per;
				Trape1[1] = 1 - Trape1[0];
				_P1.write(Trape1[ Digital_old[0] ]);
			} else {
				//動いてない場合
				_D1 = Digital_old[0] = Digital[0];
				if((Trape1[0] > PwmM) || (Trape1[0] > Restrict)) {
					Trape1[0] -= SUM_per;
					Trape1[1] = 1 - Trape1[0];
				} else {
					Trape1[0] += SUM_per;
					Trape1[1] = 1 - Trape1[0];
				}
				_P1.write(Trape1[1]);
			}

			//モーター3制御
			if((Digital[2] != Digital_old[2]) && (Trape3[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape3[0] -= SUM_per;
				Trape3[1] = 1 - Trape3[0];
				_P3.write(Trape3[ Digital_old[2] ]);
			} else {
				//動いてない場合
				_D3 = Digital_old[2] = Digital[2];
				if((Trape3[0] > PwmM) || (Trape3[0] > Restrict)) {
					Trape3[0] -= SUM_per;
					Trape3[1] = 1 - Trape3[0];
				} else {
					Trape3[0] += SUM_per;
					Trape3[1] = 1 - Trape3[0];
				}
				_P3.write(Trape3[1]);
			}

			//モーター2制御
			if((Digital[1] != Digital_old[1]) && (Trape2[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape2[0] -= SUM_per;
				Trape2[1] = 1 - Trape2[0];
				_P2.write(Trape2[ Digital_old[1] ]);
			} else {
				//動いてない場合
				_D2 = Digital_old[1] = Digital[1];
				if((Trape2[0] > PwmM) || (Trape2[0] > Restrict)) {
					Trape2[0] -= SUM_per;
					Trape2[1] = 1 - Trape2[0];
				} else {
					Trape2[0] += SUM_per;
					Trape2[1] = 1 - Trape2[0];
				}
				_P2.write(Trape2[1]);
			}

			//モーター4制御
			if((Digital[3] != Digital_old[3]) && (Trape4[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape4[0] -= SUM_per;
				Trape4[1] = 1 - Trape4[0];
				_P4.write(Trape4[ Digital_old[3] ]);
			} else {
				//動いてない場合
				_D4 = Digital_old[3] = Digital[3];
				if((Trape4[0] > PwmM) || (Trape4[0] > Restrict)) {
					Trape4[0] -= SUM_per;
					Trape4[1] = 1 - Trape4[0];
				} else {
					Trape4[0] += SUM_per;
					Trape4[1] = 1 - Trape4[0];
				}
				_P4.write(Trape4[1]);
			}
		} else if(JoyX <= 127) { //左旋回の場合
			for(i = 0; i < 4; ++i){
				Digital[i] = 0;
			}
			//モーター1制御
			if((Digital[0] != Digital_old[0]) && (Trape1[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape1[0] -= SUM_per;
				Trape1[1] = 1 - Trape1[0];
				_P1.write(Trape1[ Digital_old[0] ]);
			} else {
				//動いてない場合
				_D1 = Digital_old[0] = Digital[0];
				if((Trape1[0] > PwmM) || (Trape1[0] > Restrict)) {
					Trape1[0] -= SUM_per;
				} else {
					Trape1[0] += SUM_per;
				}
				_P1.write(Trape1[0]);
			}

			//モーター3制御
			if((Digital[2] != Digital_old[2]) && (Trape3[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape3[0] -= SUM_per;
				Trape3[1] = 1 - Trape3[0];
				_P3.write(Trape3[ Digital_old[2] ]);
			} else {
				//動いてない場合
				_D3 = Digital_old[2] = Digital[2];
				if((Trape3[0] > PwmM) || (Trape3[0] > Restrict)) {
					Trape3[0] -= SUM_per;
				} else {
					Trape3[0] += SUM_per;
				}
				_P3.write(Trape3[0]);
			}

			//モーター2制御
			if((Digital[1] != Digital_old[1]) && (Trape2[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape2[0] -= SUM_per;
				Trape2[1] = 1 - Trape2[0];
				_P2.write(Trape2[ Digital_old[1] ]);
			} else {
				//動いてない場合
				_D2 = Digital_old[1] = Digital[1];
				if((Trape2[0] > PwmM) || (Trape2[0] > Restrict)) {
					Trape2[0] -= SUM_per;
				} else {
					Trape2[0] += SUM_per;
				}
				_P2.write(Trape2[0]);
			}

			//モーター4制御
			if((Digital[3] != Digital_old[3]) && (Trape4[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape4[0] -= SUM_per;
				Trape4[1] = 1 - Trape4[0];
				_P4.write(Trape4[ Digital_old[3] ]);
			} else {
				//動いてない場合
				_D4 = Digital_old[3] = Digital[3];
				if((Trape4[0] > PwmM) || (Trape4[0] > Restrict)) {
					Trape4[0] -= SUM_per;
				} else {
					Trape4[0] += SUM_per;
				}
				_P4.write(Trape4[0]);
			}
		}
	} else { //動かさないとき
		_D1 = Digital_old[0];
		_D3 = Digital_old[2];
		_D2 = Digital_old[1];
		_D4 = Digital_old[3];
		if(Trape1[0] > 0.0) {
			Trape1[0] -= SUM_per;
			Trape1[1] = 1 - Trape1[0];
		}
		if(Trape3[0] > 0.0) {
			Trape3[0] -= SUM_per;
			Trape3[1] = 1 - Trape3[0];
		}
		if(Trape2[0] > 0.0) {
			Trape2[0] -= SUM_per;
			Trape2[1] = 1 - Trape2[0];
		}
		if(Trape4[0] > 0.0) {
			Trape4[0] -= SUM_per;
			Trape4[1] = 1 - Trape4[0];
		}
		_P1.write(Trape1[ Digital_old[0] ]);
		_P3.write(Trape3[ Digital_old[2] ]);
		_P2.write(Trape2[ Digital_old[1] ]);
		_P4.write(Trape4[ Digital_old[3] ]);
	}
}

void MoveCtrl::SlideTurn(unsigned char JoyX, unsigned char JoyY) { //横移動旋回制御(前後反転)
	//ギアによる反転用
	PwmM = fabs(sub(JoyX) / 128.0); //PwmMaster算出
	if(PwmM >= LowLimit) { //傾きから動かすかを検出
		if(JoyX > 127) { //右側
			Digital[0] = 1;
			Digital[2] = 0;
			Digital[1] = 1;
			Digital[3] = 0;
			//モーター1制御
			if((Digital[0] != Digital_old[0]) && (Trape1[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape1[0] -= SUM_per;
				Trape1[1] = 1 - Trape1[0];
				_P1.write(Trape1[ Digital_old[0] ]);
			} else {
				//動いてない場合
				_D1 = Digital_old[0] = Digital[0];
				if(JoyY <= 127) { //右前
					PwmSUM[0] = PwmM;
				} else if(JoyY > 127) { //右後
					PwmSUM[0] = PwmM / 5;
				}
				if((Trape1[0] > PwmSUM[0]) || (Trape1[0] > Restrict)) {
					Trape1[0] -= SUM_per;
					Trape1[1] = 1 - Trape1[0];
				} else {
					Trape1[0] += SUM_per;
					Trape1[1] = 1 - Trape1[0];
				}
				_P1.write(Trape1[1]);
			}

			//モーター3制御
			if((Digital[2] != Digital_old[2]) && (Trape3[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape3[0] -= SUM_per;
				Trape3[1] = 1 - Trape3[0];
				_P3.write(Trape3[ Digital_old[2] ]);
			} else {
				//動いてない場合
				_D3 = Digital_old[2] = Digital[2];
				if(JoyY <= 127) { //左前
					PwmSUM[2] = PwmM / 5;
				} else if(JoyY > 127) { //左後
					PwmSUM[2] = PwmM;
				}
				if((Trape3[0] > PwmSUM[2]) || (Trape3[0] > Restrict)) {
					Trape3[0] -= SUM_per;
				} else {
					Trape3[0] += SUM_per;
				}
				_P3.write(Trape3[0]);
			}

			//モーター2制御
			if((Digital[1] != Digital_old[1]) && (Trape2[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape2[0] -= SUM_per;
				Trape2[1] = 1 - Trape2[0];
				_P2.write(Trape2[ Digital_old[1] ]);
			} else {
				//動いてない場合
				_D2 = Digital_old[1] = Digital[1];
				if(JoyY <= 127) { //右前
					PwmSUM[1] = PwmM;
				} else if(JoyY > 127) { //右後
					PwmSUM[1] = PwmM / 5;
				}
				if((Trape2[0] > PwmSUM[1]) || (Trape2[0] > Restrict)) {
					Trape2[0] -= SUM_per;
					Trape2[1] = 1 - Trape2[0];
				} else {
					Trape2[0] += SUM_per;
					Trape2[1] = 1 - Trape2[0];
				}
				_P2.write(Trape2[1]);
			}

			//モーター4制御
			if((Digital[3] != Digital_old[3]) && (Trape4[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape4[0] -= SUM_per;
				Trape4[1] = 1 - Trape4[0];
				_P4.write(Trape4[ Digital_old[3] ]);
			} else {
				//動いてない場合
				_D4 = Digital_old[3] = Digital[3];
				if(JoyY <= 127) { //右前
					PwmSUM[3] = PwmM / 5;
				} else if(JoyY > 127) { //右後
					PwmSUM[3] = PwmM;
				}
				if((Trape4[0] > PwmSUM[3]) || (Trape4[0] > Restrict)) {
					Trape4[0] -= SUM_per;
				} else {
					Trape4[0] += SUM_per;
				}
				_P4.write(Trape4[0]);
			}
		} else if(JoyX <= 127) { //左側
			Digital[0] = 0;
			Digital[2] = 1;
			Digital[1] = 0;
			Digital[3] = 1;
			//モーター1制御
			if((Digital[0] != Digital_old[0]) && (Trape1[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape1[0] -= SUM_per;
				Trape1[1] = 1 - Trape1[0];
				_P1.write(Trape1[ Digital_old[0] ]);
			} else {
				//動いてない場合
				_D1 = Digital_old[0] = Digital[0];
				if(JoyY <= 127) { //右前
					PwmSUM[0] = PwmM;
				} else if(JoyY > 127) { //右後
					PwmSUM[0] = PwmM / 5;
				}
				if((Trape1[0] > PwmSUM[0]) || (Trape1[0] > Restrict)) {
					Trape1[0] -= SUM_per;
				} else {
					Trape1[0] += SUM_per;
				}
				_P1.write(Trape1[0]);
			}

			//モーター3制御
			if((Digital[2] != Digital_old[2]) && (Trape3[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape3[0] -= SUM_per;
				Trape3[1] = 1 - Trape3[0];
				_P3.write(Trape3[ Digital_old[2] ]);
			} else {
				//動いてない場合
				_D3 = Digital_old[2] = Digital[2];
				if(JoyY <= 127) { //右前
					PwmSUM[2] = PwmM / 5;
				} else if(JoyY > 127) { //右後
					PwmSUM[2] = PwmM;
				}
				if((Trape3[0] > PwmSUM[2]) || (Trape3[0] > Restrict)) {
					Trape3[0] -= SUM_per;
					Trape3[1] = 1 - Trape3[0];
				} else {
					Trape3[0] += SUM_per;
					Trape3[1] = 1 - Trape3[0];
				}
				_P3.write(Trape3[1]);
			}

			//モーター2制御
			if((Digital[1] != Digital_old[1]) && (Trape2[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape2[0] -= SUM_per;
				Trape2[1] = 1 - Trape2[0];
				_P2.write(Trape2[ Digital_old[1] ]);
			} else {
				//動いてない場合
				_D2 = Digital_old[1] = Digital[1];
				if(JoyY <= 127) { //右前
					PwmSUM[1] = PwmM;
				} else if(JoyY > 127) { //右後
					PwmSUM[1] = PwmM / 5;
				}
				if((Trape2[0] > PwmSUM[1]) || (Trape2[0] > Restrict)) {
					Trape2[0] -= SUM_per;
				} else {
					Trape2[0] += SUM_per;
				}
				_P2.write(Trape2[0]);
			}

			//モーター4制御
			if((Digital[3] != Digital_old[3]) && (Trape4[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape4[0] -= SUM_per;
				Trape4[1] = 1 - Trape4[0];
				_P4.write(Trape4[ Digital_old[3] ]);
			} else {
				//動いてない場合
				_D4 = Digital_old[3] = Digital[3];
				if(JoyY <= 127) { //右前
					PwmSUM[3] = PwmM / 5;
				} else if(JoyY > 127) { //右後
					PwmSUM[3] = PwmM;
				}
				if((Trape4[0] > PwmSUM[3]) || (Trape4[0] > Restrict)) {
					Trape4[0] -= SUM_per;
					Trape4[1] = 1 - Trape4[0];
				} else {
					Trape4[0] += SUM_per;
					Trape4[1] = 1 - Trape4[0];
				}
				_P4.write(Trape4[1]);
			}
		}
	} else { //動かさないとき
		_D1 = Digital_old[0];
		_D3 = Digital_old[2];
		_D2 = Digital_old[1];
		_D4 = Digital_old[3];
		if(Trape1[0] > 0.0) {
			Trape1[0] -= SUM_per;
			Trape1[1] = 1 - Trape1[0];
		}
		if(Trape3[0] > 0.0) {
			Trape3[0] -= SUM_per;
			Trape3[1] = 1 - Trape3[0];
		}
		if(Trape2[0] > 0.0) {
			Trape2[0] -= SUM_per;
			Trape2[1] = 1 - Trape2[0];
		}
		if(Trape4[0] > 0.0) {
			Trape4[0] -= SUM_per;
			Trape4[1] = 1 - Trape4[0];
		}
		_P1.write(Trape1[Digital_old[0]]);
		_P3.write(Trape3[Digital_old[2]]);
		_P2.write(Trape2[Digital_old[1]]);
		_P4.write(Trape4[Digital_old[3]]);
	}
}

void MoveCtrl::SlideTurn_R(unsigned char JoyX, unsigned char JoyY) { //横移動旋回制御(前後反転)
	//ギアによる反転用
	PwmM = fabs(sub(JoyX) / 128.0); //PwmMaster算出
	if(PwmM >= LowLimit) { //傾きから動かすかを検出
		if(JoyX > 127) { //右側
			Digital[0] = 0;
			Digital[2] = 1;
			Digital[1] = 0;
			Digital[3] = 1;
			//モーター1制御
			if((Digital[0] != Digital_old[0]) && (Trape1[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape1[0] -= SUM_per;
				Trape1[1] = 1 - Trape1[0];
				_P1.write(Trape1[ Digital_old[0] ]);
			} else {
				//動いてない場合
				_D1 = Digital_old[0] = Digital[0];
				if(JoyY <= 127) { //右前
					PwmSUM[0] = PwmM;
				} else if(JoyY > 127) { //右後
					PwmSUM[0] = PwmM / 5;
				}
				if((Trape1[0] > PwmSUM[0]) || (Trape1[0] > Restrict)) {
					Trape1[0] -= SUM_per;
				} else {
					Trape1[0] += SUM_per;
				}
				_P1.write(Trape1[0]);
			}

			//モーター3制御
			if((Digital[2] != Digital_old[2]) && (Trape3[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape3[0] -= SUM_per;
				Trape3[1] = 1 - Trape3[0];
				_P3.write(Trape3[ Digital_old[2] ]);
			} else {
				//動いてない場合
				_D3 = Digital_old[2] = Digital[2];
				if(JoyY <= 127) { //右前
					PwmSUM[2] = PwmM / 5;
				} else if(JoyY > 127) { //右後
					PwmSUM[2] = PwmM;
				}
				if((Trape3[0] > PwmSUM[2]) || (Trape3[0] > Restrict)) {
					Trape3[0] -= SUM_per;
					Trape3[1] = 1 - Trape3[0];
				} else {
					Trape3[0] += SUM_per;
					Trape3[1] = 1 - Trape3[0];
				}
				_P3.write(Trape3[1]);
			}

			//モーター2制御
			if((Digital[1] != Digital_old[1]) && (Trape2[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape2[0] -= SUM_per;
				Trape2[1] = 1 - Trape2[0];
				_P2.write(Trape2[ Digital_old[1] ]);
			} else {
				//動いてない場合
				_D2 = Digital_old[1] = Digital[1];
				if(JoyY <= 127) { //右前
					PwmSUM[1] = PwmM;
				} else if(JoyY > 127) { //右後
					PwmSUM[1] = PwmM / 5;
				}
				if((Trape2[0] > PwmSUM[1]) || (Trape2[0] > Restrict)) {
					Trape2[0] -= SUM_per;
				} else {
					Trape2[0] += SUM_per;
				}
				_P2.write(Trape2[0]);
			}

			//モーター4制御
			if((Digital[3] != Digital_old[3]) && (Trape4[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape4[0] -= SUM_per;
				Trape4[1] = 1 - Trape4[0];
				_P4.write(Trape4[ Digital_old[3] ]);
			} else {
				//動いてない場合
				_D4 = Digital_old[3] = Digital[3];
				if(JoyY <= 127) { //右前
					PwmSUM[3] = PwmM / 5;
				} else if(JoyY > 127) { //右後
					PwmSUM[3] = PwmM;
				}
				if((Trape4[0] > PwmSUM[3]) || (Trape4[0] > Restrict)) {
					Trape4[0] -= SUM_per;
					Trape4[1] = 1 - Trape4[0];
				} else {
					Trape4[0] += SUM_per;
					Trape4[1] = 1 - Trape4[0];
				}
				_P4.write(Trape4[1]);
			}
		} else if(JoyX <= 127) { //左側
			Digital[0] = 1;
			Digital[2] = 0;
			Digital[1] = 1;
			Digital[3] = 0;
			//モーター1制御
			if((Digital[0] != Digital_old[0]) && (Trape1[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape1[0] -= SUM_per;
				Trape1[1] = 1 - Trape1[0];
				_P1.write(Trape1[ Digital_old[0] ]);
			} else {
				//動いてない場合
				_D1 = Digital_old[0] = Digital[0];
				if(JoyY <= 127) { //右前
					PwmSUM[0] = PwmM;
				} else if(JoyY > 127) { //右後
					PwmSUM[0] = PwmM / 5;
				}
				if((Trape1[0] > PwmSUM[0]) || (Trape1[0] > Restrict)) {
					Trape1[0] -= SUM_per;
					Trape1[1] = 1 - Trape1[0];
				} else {
					Trape1[0] += SUM_per;
					Trape1[1] = 1 - Trape1[0];
				}
				_P1.write(Trape1[1]);
			}

			//モーター3制御
			if((Digital[2] != Digital_old[2]) && (Trape3[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape3[0] -= SUM_per;
				Trape3[1] = 1 - Trape3[0];
				_P3.write(Trape3[ Digital_old[2] ]);
			} else {
				//動いてない場合
				_D3 = Digital_old[2] = Digital[2];
				if(JoyY <= 127) { //左前
					PwmSUM[2] = PwmM / 5;
				} else if(JoyY > 127) { //左後
					PwmSUM[2] = PwmM;
				}
				if((Trape3[0] > PwmSUM[2]) || (Trape3[0] > Restrict)) {
					Trape3[0] -= SUM_per;
				} else {
					Trape3[0] += SUM_per;
				}
				_P3.write(Trape3[0]);
			}

			//モーター2制御
			if((Digital[1] != Digital_old[1]) && (Trape2[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape2[0] -= SUM_per;
				Trape2[1] = 1 - Trape2[0];
				_P2.write(Trape2[ Digital_old[1] ]);
			} else {
				//動いてない場合
				_D2 = Digital_old[1] = Digital[1];
				if(JoyY <= 127) { //右前
					PwmSUM[1] = PwmM;
				} else if(JoyY > 127) { //右後
					PwmSUM[1] = PwmM / 5;
				}
				if((Trape2[0] > PwmSUM[1]) || (Trape2[0] > Restrict)) {
					Trape2[0] -= SUM_per;
					Trape2[1] = 1 - Trape2[0];
				} else {
					Trape2[0] += SUM_per;
					Trape2[1] = 1 - Trape2[0];
				}
				_P2.write(Trape2[1]);
			}

			//モーター4制御
			if((Digital[3] != Digital_old[3]) && (Trape4[0] > 0.0)) {
				//まだ逆向きに動いてた場合
				Trape4[0] -= SUM_per;
				Trape4[1] = 1 - Trape4[0];
				_P4.write(Trape4[ Digital_old[3] ]);
			} else {
				//動いてない場合
				_D4 = Digital_old[3] = Digital[3];
				if(JoyY <= 127) { //右前
					PwmSUM[3] = PwmM / 5;
				} else if(JoyY > 127) { //右後
					PwmSUM[3] = PwmM;
				}
				if((Trape4[0] > PwmSUM[3]) || (Trape4[0] > Restrict)) {
					Trape4[0] -= SUM_per;
				} else {
					Trape4[0] += SUM_per;
				}
				_P4.write(Trape4[0]);
			}
		}
	} else { //動かさないとき
		_D1 = Digital_old[0];
		_D3 = Digital_old[2];
		_D2 = Digital_old[1];
		_D4 = Digital_old[3];
		if(Trape1[0] > 0.0) {
			Trape1[0] -= SUM_per;
			Trape1[1] = 1 - Trape1[0];
		}
		if(Trape3[0] > 0.0) {
			Trape3[0] -= SUM_per;
			Trape3[1] = 1 - Trape3[0];
		}
		if(Trape2[0] > 0.0) {
			Trape2[0] -= SUM_per;
			Trape2[1] = 1 - Trape2[0];
		}
		if(Trape4[0] > 0.0) {
			Trape4[0] -= SUM_per;
			Trape4[1] = 1 - Trape4[0];
		}
		_P1.write(Trape1[Digital_old[0]]);
		_P3.write(Trape3[Digital_old[2]]);
		_P2.write(Trape2[Digital_old[1]]);
		_P4.write(Trape4[Digital_old[3]]);
	}
}

//ユーティリティ関数
void MoveCtrl::Slow(float value) { //速度制限用
	if(value >= 1.0) {
		Restrict = 1.0;
	} else if((value < 1.0) && ((value > 0.0))) {
		Restrict = value;
	} else {
		Restrict = 0.0;
	}
}

void MoveCtrl::Slow() { //速度制限用
		Restrict = 1.0;
}

void MoveCtrl::Limiter(float value) { //反応下限設定用
	if(value >= 1.0) {
		LowLimit = 1.0;
	} else if((value < 1.0) && ((value > 0.0))) {
		LowLimit = value;
	} else {
		LowLimit = 0.1;
	}
}

void MoveCtrl::Limiter() { //反応下限設定用
	LowLimit = 0.1;
}

void MoveCtrl::Per_SUM(float value) { //台形制御増減割合設定用
	if(value >= 1.0) {
		SUM_per = 1.0;
	} else if((value < 1.0) && ((value > 0.0))) {
		SUM_per = value;
	} else {
		SUM_per = 0.05;
	}
}

void MoveCtrl::Per_SUM() {
	SUM_per = 0.05;
}

void MoveCtrl::Debug() { //回路デバッグ用
	_D1 = _D2 = _D3 = _D4 = 0;
	_P1.write(0.7);
	_P2.write(0.7);
	_P3.write(0.7);
	_P4.write(0.7);
}

void MoveCtrl::Emergency() { //緊急停止
	_D1 = _D2 = _D3 = _D4 = 0;
	_P1.write(0.0);
	_P2.write(0.0);
	_P3.write(0.0);
	_P4.write(0.0);
	_D1 = _D2 = _D3 = _D4 = 0;
}

//以下Private
float MoveCtrl::sub(unsigned char JoyN) { //ジョイスティック数値補正関数(-128~127補正)
	return (float)JoyN - 128.0;
}
float MoveCtrl::sub2(unsigned char JoyN) { //ジョイスティック数値補正関数(反転)
	return 255 - JoyN;
}
unsigned char MoveCtrl::sub3(unsigned char JoyN) { //ジョイスティック数値補正関数(反転-128~127補正)
	return 127.0 - (float)JoyN;
}
