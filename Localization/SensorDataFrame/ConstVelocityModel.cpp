#include "ConstVelocityModel.h"
#include <iostream>


namespace Localization{

	ConstVelocityModel::ConstVelocityModel() : _X(6, 1), _P(6, 6), _A(6, 6), _Q(6, 6), _H(3, 6), _R(3, 3), _I6(6, 6){
		_P.MakeUnitMatrix(6);
		_A.MakeUnitMatrix(6);
		_Q.MakeUnitMatrix(6);
		_H.ResetToZeroMatrix();
		_R.MakeUnitMatrix(3);
		_I6.MakeUnitMatrix(6);

		_H(0, 0) = 1;   _H(1, 1) = 1;   _H(2, 2) = 1;

		_degToRad = 3.1415926 / 180;
		_radToDeg = 180 / 3.1415926;

		_accStd = 0.1;
		_accWDegStd = 0.5;

		initP(0.05, 0.05, 0.5, 0.02, 0.02, 0.5);
	}

	void ConstVelocityModel::initState(double x, double y, double deg, double vx, double vy, double wdeg) {
		_X(0, 0) = x;   _X(1, 0) = y;   _X(2, 0) = deg*_degToRad;
		_X(3, 0) = vx;   _X(4, 0) = vy;   _X(5, 0) = wdeg*_degToRad;
	}

	void ConstVelocityModel::initP(double std_x, double std_y, double std_deg, double std_vx, double std_vy, double std_wdeg) {
		double std_rad = std_deg*_degToRad;
		double std_wrad = std_wdeg*_degToRad;
		_P(0, 0) = std_x*std_x;    _P(1, 1) = std_y*std_y;    _P(2, 2) = std_rad*std_rad;
		_P(3, 3) = std_vx*std_vx;  _P(4, 4) = std_vy*std_vy;  _P(5, 5) = std_wrad*std_wrad;
	}

	void ConstVelocityModel::setQStd(double acc_std, double acc_w_deg_std) {
		_accStd = acc_std;   _accWDegStd = acc_w_deg_std;
	}

	void ConstVelocityModel::setR(double std_x, double std_y, double std_deg) {
		double std_rad = std_deg*_degToRad;
		_R(0, 0) = std_x*std_x;   _R(1, 1) = std_y*std_y;   _R(2, 2) = std_rad*std_rad;
	}

	void ConstVelocityModel::setAByDt(double dt) {
		_A(0, 3) = dt;   _A(1, 4) = dt;   _A(2, 5) = dt;
	}

	void ConstVelocityModel::setQByDt(double dt) {
		double acc_std = _accStd;
		double acc_w_std = _accWDegStd*_degToRad;

		double pos_std = acc_std*dt*dt / 2 + 0.01;
		double rad_std = acc_w_std*dt*dt / 2 + 0.1*_degToRad;
		double v_std = acc_std*dt + 0.01;
		double w_std = acc_w_std*dt + 0.1*_degToRad;

		_Q(0, 0) = pos_std*pos_std;   _Q(1, 1) = pos_std*pos_std;   _Q(2, 2) = rad_std*rad_std;
		_Q(3, 3) = v_std*v_std;       _Q(4, 4) = v_std*v_std;       _Q(5, 5) = w_std*w_std;
	}

	void ConstVelocityModel::predict(double dt) {
		setAByDt(dt);
		setQByDt(dt);

		_X = _A*_X;
		_X(2, 0) = normizeRad(_X(2, 0));

		_P = _A*_P*_A.Transpose() + _Q;
	}

	void ConstVelocityModel::predictDelta(double dt, double &dx, double &dy, double &ddeg) {
		dx = _X(3, 0)*dt;   dy = _X(4, 0)*dt;   ddeg = _X(5, 0)*dt*_radToDeg;
	}

	double ConstVelocityModel::getState(int index) {
		if (index >= 0 && index < 6) {
			return _X(index, 0);
		}
		else {
			return -1111;
		}
	}

	void ConstVelocityModel::observe(double x, double y, double deg) {
		CMatrix HPHR = _H*_P*_H.Transpose() + _R;

		HPHR.InvertGaussJordan();
		CMatrix HPHR_inv = HPHR;

		CMatrix K = _P*_H.Transpose()*HPHR_inv;

		CMatrix Z(3, 1);
		Z(0, 0) = x;   Z(1, 0) = y;   Z(2, 0) = deg*_degToRad;

		CMatrix Z_err = Z - _H*_X;
		Z_err(2, 0) = normizeRad(Z_err(2, 0));

		_X = _X + K*Z_err;
		_X(2, 0) = normizeRad(_X(2, 0));

		_P = (_I6 - K*_H)*_P;

		//std::cout << _X(0, 0) << " " << _X(1, 0) << " " << _X(2, 0)*_radToDeg << " " << _X(3, 0) << " " << _X(4, 0) << " " << _X(5, 0)*_radToDeg << std::endl;
	}

	double ConstVelocityModel::normizeRad(double rad) {
		while (rad > 3.1415926) {
			rad -= 2 * 3.1415926;
		}
		while (rad < -3.1415926) {
			rad += 2 * 3.1415926;
		}

		return rad;
	}

	double ConstVelocityModel::normizeDeg(double deg) {
		while (deg > 180) {
			deg -= 360;
		}
		while (deg < -180) {
			deg += 360;
		}

		return deg;
	}

}