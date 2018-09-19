#include "AxisTransform.h"


namespace Localization{

	void AxisTransform::setCalibrationValue(double x_in_base, double y_in_base, double z_in_base,
		double yaw_to_base, double pitch_to_base, double roll_to_base) {
		_poseRelativeToBase = Pose3D(x_in_base, y_in_base, z_in_base, yaw_to_base, pitch_to_base, roll_to_base);
		_poseRelativeToBaseInv = _poseRelativeToBase.getInvPose();
	}

	Pos3D AxisTransform::rotVectorToBase(double x, double y, double z) {
		return _poseRelativeToBase._orient*Pos3D(x, y, z);
	}

	Pos3D AxisTransform::rotVectorToBase(const Pos3D& v) {
		return _poseRelativeToBase._orient*v;
	}

	Pos3D AxisTransform::transPosToBase(const Pos3D& pos) {
		return _poseRelativeToBase*pos;
	}

	Pose3D AxisTransform::getObsPoseByBasePoseInGlobal(const Pose3D& pose_base_relative_to_global) {
		return pose_base_relative_to_global*_poseRelativeToBase;
	}

	Pose3D AxisTransform::getBasePoseByObsPoseInGlobal(const Pose3D& pose_obs_ralative_to_global) {
		return pose_obs_ralative_to_global*_poseRelativeToBaseInv;
	}

}