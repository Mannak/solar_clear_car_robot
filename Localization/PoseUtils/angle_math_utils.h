/************************************************************************
Author: huanghong
Date: 2016-10-31
************************************************************************/

#ifndef _ANGLE_MATH_UTILS_H_
#define _ANGLE_MATH_UTILS_H_


namespace Localization {

	namespace angle_math_utils {
		extern const double pi;
		extern const double rad_to_deg_ratio;
		extern const double deg_to_rad_ratio;

		double rad_to_deg(double angle);
		double deg_to_rad(double angle);
		double normalize_deg(double deg);
		double normalize_rad(double rad);

		double sind(double deg);
		double cosd(double deg);
		double tand(double deg);
		double cotd(double deg);
		double asind(double val);
		double acosd(double val);
		double atand(double val);
	}

}

#endif