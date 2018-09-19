/************************************************************************
Author: huanghong
Date: 2016-10-31
************************************************************************/

#ifndef _SENSOR_DATA_H_
#define _SENSOR_DATA_H_

#include <vector>
#include "Localization\PoseUtils\Pose2DDataStruct.h"
#include "Localization\PoseUtils\Pose3DDataStruct.h"
#include "Localization\SensorDataFrame\AxisTransform.h"
#include <string>
#include <math.h>
#include <map>

#ifdef _USE_OPENCV_
#include <opencv2\core\core.hpp>
#endif


namespace Localization{

	typedef enum {
		NONE = 0,
		ODO = 1,
		IMU,
		LASER2D,
		LASER3D_SEGMENT,
		LASER3D,
		CAMERA,
		GNSS,
		MAGENETIC
	} SENSOR_TYPE;

	class BaseSensorData {
	private:
		static std::map<SENSOR_TYPE, std::string> _sensorTypeNameStr;
		static void createNameStrMap();
	public:
		inline std::string getNameStr() {
			if (_sensorTypeNameStr.size() == 0) {
				createNameStrMap();
			}
			return _sensorTypeNameStr[_sensorType];
		}
		inline static std::string getNameStrBySensorType(SENSOR_TYPE sensor_type) {
			if (_sensorTypeNameStr.size() == 0) {
				createNameStrMap();
			}
			return _sensorTypeNameStr[sensor_type];
		}
		static SENSOR_TYPE getSensorTypeByNameStr(std::string str);

	public:
		double _localTime;
		double _intvalTime;
		bool _state;
		SENSOR_TYPE _sensorType;

		BaseSensorData(double local_time = 0, double intval_time = 0, bool state = false, SENSOR_TYPE sensor_type = SENSOR_TYPE::NONE) {
			_localTime = local_time;
			_intvalTime = intval_time;
			_state = state;
			_sensorType = sensor_type;
		}

		BaseSensorData& operator=(const BaseSensorData& other);

		virtual void axisTransformSensorData(AxisTransform& axis_transform) {}

		virtual ~BaseSensorData() {}
	};

	class OdoSensorData : public BaseSensorData {
	public:
		bool _ifUseOdoData;

		double _odoX, _odoY, _odoAngleDeg;
		double _speedX, _speedY, _angleSpeedDeg;
		//std::vector<double> _wheelOdoVec;
		std::vector<double> _wheelSpeedVec;
		std::vector<double> _wheelAngleDegVec;

		OdoSensorData(bool if_use_odo_data, double local_time = 0, double intval_time = 0, bool state = false) :
			BaseSensorData(local_time, intval_time, state, SENSOR_TYPE::ODO) {
			_ifUseOdoData = if_use_odo_data;
			_odoX = _odoY = _odoAngleDeg = 0;
			_speedX = _speedY = _angleSpeedDeg = 0;
		}

		OdoSensorData& operator=(const OdoSensorData &other);

		static Pose2D getInterpolationDeltaPose2D(OdoSensorData *data_pre, OdoSensorData *data_post, double delta_t);

		void setOdoData(double odo_x, double odo_y, double odo_angle_deg) {
			_odoX = odo_x;   _odoY = odo_y;   _odoAngleDeg = odo_angle_deg;
		}

		void setSpeedData(double speed_x, double speed_y, double angle_speed_deg) {
			_speedX = speed_x;   _speedY = speed_y;   _angleSpeedDeg = angle_speed_deg;
		}

		void setWheelSpeedVec(std::vector<double>& wheel_speed_vec, std::vector<double>& wheel_angle_deg_vec) {
			_wheelSpeedVec = wheel_speed_vec;
			_wheelAngleDegVec = wheel_angle_deg_vec;
		}

		double getWheelAverageSpeed();
	};

	class ImuSensorData : public BaseSensorData {
	public:
		double _gyroDeg[3];
		double _quat[4];
		double _acc[3];
		bool _ifUseQuat;

		ImuSensorData(bool if_use_quat, double local_time = 0, double intval_time = 0, bool state = false)
			: BaseSensorData(local_time, intval_time, state, SENSOR_TYPE::IMU) {
			_gyroDeg[0] = 0;   _gyroDeg[1] = 0;   _gyroDeg[2] = 0;
			_quat[0] = 1;   _quat[1] = 0;   _quat[2] = 0;   _quat[3] = 0;
			_acc[0] = 0;   _acc[1] = 0;   _acc[2] = 9.9;
			_ifUseQuat = if_use_quat;
		}

		ImuSensorData(double gyro_deg[3], double quat[4], double acc[3], bool if_use_quat, double local_time = 0, double intval_time = 0, bool state = false);
		ImuSensorData& operator=(const ImuSensorData& imu_sensor_data);

		void setGyroDeg(double gyro_deg[3]) {
			_gyroDeg[0] = gyro_deg[0];   _gyroDeg[1] = gyro_deg[1];   _gyroDeg[2] = gyro_deg[2];
		}
		void setGyroDeg(double gyro_x_deg, double gyro_y_deg, double gyro_z_deg) {
			_gyroDeg[0] = gyro_x_deg;   _gyroDeg[1] = gyro_y_deg;   _gyroDeg[2] = gyro_z_deg;
		}
		void setQuat(double quat[4]) {
			_quat[0] = quat[0];   _quat[1] = quat[1];   _quat[2] = quat[2];   _quat[3] = quat[3];
		}
		void setQuat(double q0, double q1, double q2, double q3) {
			_quat[0] = q0;   _quat[1] = q1;   _quat[2] = q2;   _quat[3] = q3;
		}
		void setAcc(double acc[3]) {
			_acc[0] = acc[0];   _acc[1] = acc[1];   _acc[2] = acc[2];
		}
		void setAcc(double ax, double ay, double az) {
			_acc[0] = ax;   _acc[1] = ay;   _acc[2] = az;
		}

		void axisTransformSensorData(AxisTransform& axis_transform);

	};

	class Laser2dSensorData : public BaseSensorData {
	public:
		class Beam {
		public:
			Polar2D _polar;
			Pos2D _pos;
			int _rssi;
			bool _validity;
			Beam(double angle_deg = 0, double dis = 0, int rssi = 0, bool validity = true) :_polar(angle_deg, dis) {
				_pos = _polar.toPos();
				_rssi = rssi;
				_validity = validity;
			}

			Beam& operator=(const Beam& other) {
				_polar = other._polar;
				_pos = other._pos;
				_rssi = other._rssi;
				_validity = other._validity;
				return *this;
			}
		};

		std::vector<Beam> _beamVec;
		int _sumValidBeam;

		Laser2dSensorData(double beam_size = 0, double local_time = 0, double intvl_time = 0, bool state = false)
			: BaseSensorData(local_time, intvl_time, state, SENSOR_TYPE::LASER2D) {
			_beamVec.resize(beam_size);
			_sumValidBeam = 0;
		}
		Laser2dSensorData &operator=(const Laser2dSensorData& other);

		void axisTransformSensorData(AxisTransform& axis_transform);

	};

	class Laser3dSensorDataSegment : public BaseSensorData {
	public:
		class Beam3D {
		public:
			Pos3D _pos;
			int _rssi;
			bool _validity;
			Beam3D(double x = 0, double y = 0, double z = 0, int rssi = 0, bool validity = true) {
				_pos._xyz[0] = x;
				_pos._xyz[1] = y;
				_pos._xyz[2] = z;
				_rssi = rssi;
				_validity = validity;
			}
			Beam3D &operator=(const Beam3D& other) {
				_pos = other._pos;
				_rssi = other._rssi;
				_validity = other._validity;
				return *this;
			}
		};

		std::vector<Beam3D> _beamVec;
		int _sumValidBeam;

		Laser3dSensorDataSegment(int beam_size = 0, double local_time = 0, double intvl_time = 0, bool state = false)
			: BaseSensorData(local_time, intvl_time, state, SENSOR_TYPE::LASER3D_SEGMENT) {
			_sumValidBeam = 0;
			_beamVec.resize(beam_size);
		}
		Laser3dSensorDataSegment &operator=(const Laser3dSensorDataSegment& other);

		void axisTransformSensorData(AxisTransform& axis_transform);

	};

	class Laser3dSensorData : public BaseSensorData {
	public:
		std::vector<Laser3dSensorDataSegment> _segmentVec;

		Laser3dSensorData(int segment_size = 0, int segment_beam_size = 0, double local_time = 0, double intvl_time = 0, bool state = false)
			: BaseSensorData(local_time, intvl_time, state, SENSOR_TYPE::LASER3D) {
			_segmentVec.resize(segment_size, Laser3dSensorDataSegment(segment_beam_size));
		}

		Laser3dSensorData& operator=(const Laser3dSensorData& other);

		void axisTransformSensorData(AxisTransform& axis_transform);

		double getSegmentAverageLocalTime();
	};

	class CameraSensorData : public BaseSensorData {
	public:
		CameraSensorData(double local_time = 0, double intvl_time = 0, bool state = false)
			: BaseSensorData(local_time, intvl_time, state, SENSOR_TYPE::CAMERA) {
		}

#ifdef _USE_OPENCV_
		CameraSensorData(cv::Mat image, double local_time = 0, double intvl_time = 0, bool state = false)
			: BaseSensorData(local_time, intvl_time, state, SENSOR_TYPE::CAMERA) {
			_image = image;
		}

		CameraSensorData& operator=(const CameraSensorData& other);

		cv::Mat _image;
#endif
	};

	class GnssSensorData : public BaseSensorData {
	public:
		GnssSensorData() : BaseSensorData(0, 0, false, SENSOR_TYPE::GNSS) {
		}

	};

	class MagneticSensorData : public BaseSensorData {
	public:
		MagneticSensorData() : BaseSensorData(0, 0, false, SENSOR_TYPE::MAGENETIC) {
		}

	};

}
#endif