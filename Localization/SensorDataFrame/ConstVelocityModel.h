/**************************************
author: huanghong   20161201
***************************************/


#ifndef _CONST_VELOCITY_MODEL_
#define _CONST_VELOCITY_MODEL_

#include "ComUtils\Matrix.h"


namespace Localization{

	class ConstVelocityModel {
	private:
		ConstVelocityModel();

	public:
		static ConstVelocityModel* instance() {
			static ConstVelocityModel ins;
			return &ins;
		}

		void initState(double x, double y, double deg, double vx = 0, double vy = 0, double wdeg = 0);
		void initP(double std_x, double std_y, double std_deg, double std_vx, double std_vy, double std_wdeg);//optional to call
		void setQStd(double acc_std, double acc_w_deg_std);
		void setR(double std_x, double std_y, double std_deg);

		void predict(double dt);
		void observe(double x, double y, double deg);

		void predictDelta(double dt, double &dx, double &dy, double &ddeg);

	public:
		double getState(int index);

	private:
		CMatrix _X;
		CMatrix _P;
		CMatrix _A;
		CMatrix _Q;
		CMatrix _H;
		CMatrix _R;
		CMatrix _I6;

		void setAByDt(double dt);
		void setQByDt(double dt);

		double _accStd;
		double _accWDegStd;

		double _degToRad;
		double _radToDeg;
		double normizeRad(double rad);
		double normizeDeg(double deg);
	};

}
#endif