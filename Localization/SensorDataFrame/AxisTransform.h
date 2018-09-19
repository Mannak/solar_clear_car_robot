/************************************************************************
Author: huanghong
Date: 2016-10-31
************************************************************************/

#ifndef _AXIS_TRANSFORM_
#define _AXIS_TRANSFORM_

#include "Localization\PoseUtils\Pose3DDataStruct.h"


namespace Localization{

	class AxisTransform {
	public:
		Pose3D _poseRelativeToBase;
		Pose3D _poseRelativeToBaseInv;

		AxisTransform(){}

		void setCalibrationValue(double x_in_base, double y_in_base, double z_in_base,
			double yaw_to_base, double pitch_to_base, double roll_to_base);

		Pos3D rotVectorToBase(double x, double y, double z);
		Pos3D rotVectorToBase(const Pos3D& v);

		Pos3D transPosToBase(const Pos3D& pos);

		Pose3D getObsPoseByBasePoseInGlobal(const Pose3D& pose_base_relative_to_global);

		Pose3D getBasePoseByObsPoseInGlobal(const Pose3D& pose_obs_ralative_to_global);
	};

}
#endif


