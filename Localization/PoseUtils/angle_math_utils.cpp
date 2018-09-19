#include "angle_math_utils.h"
#include <math.h>


namespace Localization {

	namespace angle_math_utils {
		const double pi = 3.1415926535;
		const double rad_to_deg_ratio = 180 / pi;
		const double deg_to_rad_ratio = pi / 180;

		double rad_to_deg(double angle) {
			angle *= rad_to_deg_ratio;
			return angle;
		}

		double deg_to_rad(double angle) {
			angle *= deg_to_rad_ratio;
			return angle;
		}

		double normalize_deg(double deg) {
			while (deg < -180) {
				deg += 360;
			}
			while (deg > 180) {
				deg -= 360;
			}
			return deg;
		}

		double normalize_rad(double rad) {
			while (rad < -pi) {
				rad += 2 * pi;
			}
			while (rad > pi) {
				rad -= 2 * pi;
			}
			return rad;
		}

		double sind(double deg) {
			return sin(deg*deg_to_rad_ratio);
		}

		double cosd(double deg) {
			return cos(deg*deg_to_rad_ratio);
		}

		double tand(double deg) {
			return tan(deg*deg_to_rad_ratio);
		}

		double cotd(double deg) {
			return 1. / tan(deg*deg_to_rad_ratio);
		}

		double asind(double val) {
			return asin(val)*rad_to_deg_ratio;
		}

		double acosd(double val) {
			return acos(val)*rad_to_deg_ratio;
		}

		double atand(double val) {
			return atan(val)*rad_to_deg_ratio;
		}
	}

}