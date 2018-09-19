/************************************************************************
Author: huanghong
Date: 2016-10-31
************************************************************************/

#ifndef _SENSOR_DEVICE_DATA_H_
#define _SENSOR_DEVICE_DATA_H_

#include "Localization\SensorDataFrame\SensorData.h"
#include "Localization\PoseUtils\Pose3DDataStruct.h"


namespace Localization{

	class SynDataListLogicVar {
	public:
		SynDataListLogicVar() {
			_ifPosInDataListLocked = false;
			_ifIsStatic = false;
		}

		SynDataListLogicVar &operator=(const SynDataListLogicVar &other) {
			_ifPosInDataListLocked = other._ifPosInDataListLocked;
			_ifIsStatic = other._ifIsStatic;

			return *this;
		}

		bool _ifPosInDataListLocked;
		bool _ifIsStatic;
	};

	class SynDataListDr {
	public:
		Pose3D _drPose;

		double _attitudePitchDeg;
		double _attitudeRollDeg;

		Pose3D _deltaDrPose;

		bool _ifDrCaculated;

		double _sumMoveDis;
		double _sumMoveDeg;

		SynDataListDr();

		SynDataListDr &operator=(const SynDataListDr& other);
	};

	class SensorDevice {
	public:
		SENSOR_TYPE _sensorType;
		int _deviceId;

		SensorDevice(const SensorDevice& other) {
			_sensorType = other._sensorType;
			_deviceId = other._deviceId;
		}
		SensorDevice(SENSOR_TYPE sensor_type, int device_id) {
			_sensorType = sensor_type;
			_deviceId = device_id;
		}
		SensorDevice& operator=(const SensorDevice& other) {
			_sensorType = other._sensorType;
			_deviceId = other._deviceId;
			return *this;
		}
		bool operator==(const SensorDevice& other) {
			return _sensorType == other._sensorType&&_deviceId == other._deviceId;
		}
		bool operator<(const SensorDevice& other) const {
			if (_sensorType < other._sensorType) {
				return true;
			}
			else if (_sensorType == other._sensorType) {
				return _deviceId < other._deviceId;
			}
			else {
				return false;
			}
		}
		bool ifIsZeroIdSensor(SENSOR_TYPE sensor_type) {
			return _sensorType == sensor_type && _deviceId == 0;
		}
	};

	class BaseSensorDeviceData {
	public:
		BaseSensorDeviceData(SENSOR_TYPE sensor_type, int device_id) :_sensorDevice(sensor_type, device_id) {
		}
		BaseSensorDeviceData(const SensorDevice& sensor_device) : _sensorDevice(sensor_device) {
		}
		BaseSensorDeviceData &operator=(const BaseSensorDeviceData& other);
		SensorDevice _sensorDevice;

		SynDataListLogicVar _synDataListLogicVar;
		SynDataListDr _synDataListDr;

		virtual BaseSensorData* getSensorDataPtr() = 0;

		SENSOR_TYPE getSensorType() {
			return _sensorDevice._sensorType;
		}
		bool ifIsSensorType(SENSOR_TYPE sensor_type) {
			return _sensorDevice._sensorType == sensor_type;
		}
		bool ifIsZeroIdSensor(SENSOR_TYPE sensor_type) {
			return _sensorDevice._sensorType == sensor_type && _sensorDevice._deviceId == 0;
		}

		virtual ~BaseSensorDeviceData() {}
	};

	class OdoDeviceData : public BaseSensorDeviceData {
	public:
		OdoDeviceData(bool if_use_odo_data, double local_time = 0, double intval_time = 0, bool state = false, int device_id = 0)
			: BaseSensorDeviceData(SENSOR_TYPE::ODO, device_id), _sensorData(if_use_odo_data, local_time, intval_time, state){
			_ifIsOdoStatic = false;
		}

		OdoDeviceData& operator=(const OdoDeviceData& other);

		bool _ifIsOdoStatic;

		OdoSensorData _sensorData;

		BaseSensorData* getSensorDataPtr() {
			return &_sensorData;
		}
		OdoSensorData* getOdoSensorDataPtr() {
			return &_sensorData;
		}
	};

	class ImuDeviceData : public BaseSensorDeviceData {
	public:
		ImuDeviceData(bool if_use_quat, double local_time = 0, double intval_time = 0, bool state = false, int device_id = 0)
			: BaseSensorDeviceData(SENSOR_TYPE::IMU, device_id), _sensorData(if_use_quat, local_time, intval_time, state){
		}
		ImuDeviceData &operator=(const ImuDeviceData& other);

		ImuSensorData _sensorData;

		BaseSensorData* getSensorDataPtr() {
			return &_sensorData;
		}
		ImuSensorData* getImuSensorDataPtr() {
			return &_sensorData;
		}
	};

	class Laser2dDeviceData : public BaseSensorDeviceData {
	public:
		Laser2dDeviceData(int beam_size = 0, double local_time = 0, double intval_time = 0, bool state = false, int device_id = 0)
			: BaseSensorDeviceData(SENSOR_TYPE::LASER2D, device_id), _sensorData(beam_size, local_time, intval_time, state){
		}
		Laser2dDeviceData &operator=(const Laser2dDeviceData& other);

		Laser2dSensorData _sensorData;

		BaseSensorData* getSensorDataPtr() {
			return &_sensorData;
		}
		Laser2dSensorData* getLaser2dSensorDataPtr() {
			return &_sensorData;
		}
	};

	class Laser3dDeviceData : public BaseSensorDeviceData {
	public:
		Laser3dDeviceData(int segment_size = 0, int segment_beam_size = 0, double local_time = 0, double intval_time = 0, bool state = false, int device_id = 0)
			: BaseSensorDeviceData(SENSOR_TYPE::LASER3D, device_id), _sensorData(segment_size, segment_beam_size, local_time, intval_time, state){
		}
		Laser3dDeviceData &operator=(const Laser3dDeviceData& other);

		Laser3dSensorData _sensorData;

		BaseSensorData* getSensorDataPtr() {
			return &_sensorData;
		}
		Laser3dSensorData* getLaser3dSensorDataPtr() {
			return &_sensorData;
		}
	};

	class CameraDeviceData : public BaseSensorDeviceData {
	public:
		CameraDeviceData(double local_time = 0, double intval_time = 0, bool state = false, int device_id = 0)
			: BaseSensorDeviceData(SENSOR_TYPE::CAMERA, device_id), _sensorData(local_time, intval_time, state) {
		}

#ifdef _USE_OPENCV_
		CameraDeviceData(cv::Mat image, double local_time = 0, double intval_time = 0, bool state = false, int device_id = 0)
			: BaseSensorDeviceData(SENSOR_TYPE::CAMERA, device_id), _sensorData(image, local_time, intval_time, state) {
		}

		CameraDeviceData& operator=(const CameraDeviceData& other);
#endif

		CameraSensorData _sensorData;

		BaseSensorData* getSensorDataPtr() {
			return &_sensorData;
		}
		CameraSensorData* getCameraSensorDataPtr() {
			return &_sensorData;
		}
	};

	class GnssDeviceData : public BaseSensorDeviceData {
	public:
		GnssDeviceData(int device_id = 0)
			: BaseSensorDeviceData(SENSOR_TYPE::GNSS, device_id) {
		}

		GnssSensorData _sensorData;

		BaseSensorData* getSensorDataPtr() {
			return &_sensorData;
		}
		GnssSensorData* getGnssSensorDataPtr() {
			return &_sensorData;
		}
	};

	class MageneticDeviceData : public BaseSensorDeviceData {
	public:
		MageneticDeviceData(int device_id = 0)
			: BaseSensorDeviceData(SENSOR_TYPE::MAGENETIC, device_id) {
		}

		MagneticSensorData _sensorData;

		BaseSensorData* getSensorDataPtr() {
			return &_sensorData;
		}
		MagneticSensorData* getMagneticSensorData() {
			return &_sensorData;
		}
	};

}
#endif