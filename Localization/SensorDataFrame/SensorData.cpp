#include "SensorData.h"
#include "Localization\PoseUtils\angle_math_utils.h"
#include "Localization\PoseUtils\rot_fuc_deg.h"


using namespace Localization::angle_math_utils;

namespace Localization{

	std::map<SENSOR_TYPE, std::string> BaseSensorData::_sensorTypeNameStr;

	void BaseSensorData::createNameStrMap() {
		if (_sensorTypeNameStr.size() != 0) {
			return;
		}

		_sensorTypeNameStr[NONE] = "NONE";
		_sensorTypeNameStr[ODO] = "ODO";
		_sensorTypeNameStr[IMU] = "IMU";
		_sensorTypeNameStr[LASER2D] = "LASER2D";
		_sensorTypeNameStr[LASER3D_SEGMENT] = "LASER3D_SEGMENT";
		_sensorTypeNameStr[LASER3D] = "LASER3D";
		_sensorTypeNameStr[CAMERA] = "CAMERA";
		_sensorTypeNameStr[GNSS] = "GNSS";
		_sensorTypeNameStr[MAGENETIC] = "MAGENETIC";
	}

	SENSOR_TYPE BaseSensorData::getSensorTypeByNameStr(std::string str) {
		if (_sensorTypeNameStr.size() == 0) {
			createNameStrMap();
		}

		for (std::map<SENSOR_TYPE, std::string>::iterator iter = _sensorTypeNameStr.begin(); iter != _sensorTypeNameStr.end(); ++iter) {
			if (iter->second == str) {
				return iter->first;
			}
		}

		return SENSOR_TYPE::NONE;
	}

	BaseSensorData& BaseSensorData::operator=(const BaseSensorData& other) {
		_localTime = other._localTime;
		_intvalTime = other._intvalTime;
		_state = other._state;
		_sensorType = other._sensorType;

		return *this;
	}

	OdoSensorData& OdoSensorData::operator=(const OdoSensorData &other) {
		_localTime = other._localTime;
		_intvalTime = other._intvalTime;
		_state = other._state;
		_sensorType = other._sensorType;

		_ifUseOdoData = other._ifUseOdoData;

		_odoX = other._odoX;  _odoY = other._odoY;   _odoAngleDeg = other._odoAngleDeg;
		_speedX = other._speedX;  _speedY = other._speedY;   _angleSpeedDeg = other._angleSpeedDeg;
		//std::vector<double> _wheelOdoVec;
		_wheelSpeedVec = other._wheelSpeedVec;
		_wheelAngleDegVec = other._wheelAngleDegVec;

		return *this;
	}

	Pose2D OdoSensorData::getInterpolationDeltaPose2D(OdoSensorData *data_pre, OdoSensorData *data_post, double delta_t) {
		Pose2D pose;
		if (data_pre == NULL || data_post == NULL) {
			return pose;
		}

		double dt21 = data_post->_localTime - data_pre->_localTime;

		if (dt21 <= 0 || delta_t <= 0) {
			return pose;
		}

		Pose2D pose_post(data_post->_odoX, data_post->_odoY, data_post->_odoAngleDeg);
		Pose2D pose_pre(data_pre->_odoX, data_pre->_odoY, data_pre->_odoAngleDeg);
		Pose2D temp_pose = pose_post / pose_pre;

		double ratio = delta_t / dt21;
		pose = Pose2D(temp_pose._pos._x*ratio, temp_pose._pos._y*ratio, temp_pose._orient._deg*ratio);
		return pose;
	}

	double OdoSensorData::getWheelAverageSpeed() {
		if (_wheelSpeedVec.size() == 0) {
			return 0;
		}

		double average_speed = 0;
		for (int i = 0; i < _wheelSpeedVec.size(); ++i) {
			average_speed += _wheelSpeedVec[i];
		}
		average_speed /= _wheelSpeedVec.size();
		return average_speed;
	}

	ImuSensorData::ImuSensorData(double gyro_deg[3], double quat[4], double acc[3], bool if_use_quat, double local_time, double intval_time, bool state) :
		BaseSensorData(local_time, intval_time, state, SENSOR_TYPE::IMU) {
		rot_fuc_deg::array_copy(gyro_deg, _gyroDeg, 3);
		rot_fuc_deg::array_copy(quat, _quat, 4);
		rot_fuc_deg::array_copy(acc, _acc, 3);
		_ifUseQuat = if_use_quat;
	}

	ImuSensorData& ImuSensorData::operator = (const ImuSensorData& imu_sensor_data) {
		_localTime = imu_sensor_data._localTime;
		_intvalTime = imu_sensor_data._intvalTime;
		_state = imu_sensor_data._state;
		_sensorType = imu_sensor_data._sensorType;

		rot_fuc_deg::array_copy(imu_sensor_data._gyroDeg, _gyroDeg, 3);
		rot_fuc_deg::array_copy(imu_sensor_data._quat, _quat, 4);
		rot_fuc_deg::array_copy(imu_sensor_data._acc, _acc, 3);
		_ifUseQuat = imu_sensor_data._ifUseQuat;

		return *this;
	}

	void ImuSensorData::axisTransformSensorData(AxisTransform& axis_transform) {
		Pos3D temp = axis_transform.rotVectorToBase(_gyroDeg[0], _gyroDeg[1], _gyroDeg[2]);
		_gyroDeg[0] = temp._xyz[0];   _gyroDeg[1] = temp._xyz[1];   _gyroDeg[2] = temp._xyz[2];

		Pos3D temp1 = axis_transform.rotVectorToBase(_acc[0], _acc[1], _acc[2]);
		_acc[0] = temp1._xyz[0];   _acc[1] = temp1._xyz[1];   _acc[2] = temp1._xyz[2];
	}

	Laser2dSensorData &Laser2dSensorData::operator=(const Laser2dSensorData& other) {
		_localTime = other._localTime;
		_intvalTime = other._intvalTime;
		_state = other._state;
		_sensorType = other._sensorType;

		_beamVec = other._beamVec;

		_sumValidBeam = other._sumValidBeam;

		return *this;
	}

	void Laser2dSensorData::axisTransformSensorData(AxisTransform& axis_transform) {
		Pos3D beam(0, 0, 0);
		Pos3D temp(0, 0, 0);
		for (int i = 0; i < _beamVec.size(); ++i) {
			beam._xyz[0] = _beamVec[i]._pos._x;
			beam._xyz[1] = _beamVec[i]._pos._y;

			temp = axis_transform.transPosToBase(beam);

			_beamVec[i]._pos._x = temp._xyz[0];
			_beamVec[i]._pos._y = temp._xyz[1];
		}
	}

	Laser3dSensorDataSegment &Laser3dSensorDataSegment::operator=(const Laser3dSensorDataSegment& other) {
		_localTime = other._localTime;
		_intvalTime = other._intvalTime;
		_state = other._state;
		_sensorType = other._sensorType;

		_beamVec = other._beamVec;
		_sumValidBeam = other._sumValidBeam;

		return *this;
	}

	void Laser3dSensorDataSegment::axisTransformSensorData(AxisTransform& axis_transform) {
		for (int i = 0; i < _beamVec.size(); ++i) {
			_beamVec[i]._pos = axis_transform.transPosToBase(_beamVec[i]._pos);
		}
	}

	Laser3dSensorData& Laser3dSensorData::operator=(const Laser3dSensorData& other) {
		_localTime = other._localTime;
		_intvalTime = other._intvalTime;
		_state = other._state;
		_sensorType = other._sensorType;

		_segmentVec = other._segmentVec;
		return *this;
	}

	void Laser3dSensorData::axisTransformSensorData(AxisTransform& axis_transform) {
		for (int i = 0; i < _segmentVec.size(); ++i) {
			_segmentVec[i].axisTransformSensorData(axis_transform);
		}
	}

	double Laser3dSensorData::getSegmentAverageLocalTime() {
		if (_segmentVec.size() == 0) {
			return 0;
		}

		double avg = 0;

		for (int i = 0; i < _segmentVec.size(); ++i) {
			avg += _segmentVec[i]._localTime;
		}

		avg /= _segmentVec.size();

		return avg;
	}

	CameraSensorData& CameraSensorData::operator=(const CameraSensorData& other) {
		_localTime = other._localTime;
		_intvalTime = other._intvalTime;
		_state = other._state;
		_sensorType = other._sensorType;

#ifdef _USE_OPENCV_
		_image = other._image.clone();
#endif

		return *this;
	}

}