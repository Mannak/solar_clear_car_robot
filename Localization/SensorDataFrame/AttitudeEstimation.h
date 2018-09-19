#ifndef _ATTITUDE_ESTIMATION_
#define _ATTITUDE_ESTIMATION_


#include "ComUtils\Matrix.h"
#include "Localization\PoseUtils\angle_math_utils.h"
#include "Localization\PoseUtils\rot_fuc_deg.h"
#include <vector>


namespace Localization{

	class AttitudeEstimation {
	private:
		CMatrix X0;
		CMatrix P0;
		CMatrix A;
		CMatrix Q;
		CMatrix R;
		CMatrix Eye3;

		double _g;

		bool _inited;
		double _pitch_rad;
		double _roll_rad;

	public:
		AttitudeEstimation() : X0(3, 1), P0(3, 3), A(3, 3), Q(3, 3), R(3, 3), Eye3(3, 3) {//
			_g = 9.99;
			_inited = false;
			_pitch_rad = 0;
			_roll_rad = 0;
		}

		void init(double g = 9.9, double Q_deg_std_time_scale = 0.1, double R_uxy_deg_std = 5, double R_uz_deg_std = 1.7);

		bool ifInited() {//
			return _inited;
		}

		void getAccToInit(CMatrix acc);

		void initState(double pitch_rad, double roll_rad);

		void doFilter(double gyro_deg_x, double gyro_deg_y, double gyro_deg_z, double acc_x, double acc_y, double acc_z, double t);
		void doFilter(CMatrix gyro_deg, CMatrix acc, double t);

		double getPitchDeg() {//
			return Localization::angle_math_utils::rad_to_deg(_pitch_rad);
		}

		double getRollDeg() {//
			return Localization::angle_math_utils::rad_to_deg(_roll_rad);
		}

	private:
		double _confQRadStdTimeScale;

		CMatrix angleToX(double pitch_rad, double roll_rad);

		void xToAngle(CMatrix x, double &pitch_rad, double &roll_rad);

		CMatrix getR3Cov(double pitch_rad_std, double roll_rad_std, double pitch_rad = 0, double roll_rad = 0);
	};

}

#endif