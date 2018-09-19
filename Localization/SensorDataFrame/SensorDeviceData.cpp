#include "SensorDeviceData.h"


namespace Localization{

	SynDataListDr::SynDataListDr() {
		_attitudePitchDeg = 0;
		_attitudeRollDeg = 0;

		_ifDrCaculated = false;

		_sumMoveDis = 0;
		_sumMoveDeg = 0;
	}

	SynDataListDr &SynDataListDr::operator=(const SynDataListDr& other) {
		_drPose = other._drPose;

		_attitudePitchDeg = other._attitudePitchDeg;
		_attitudeRollDeg = other._attitudeRollDeg;

		_deltaDrPose = other._deltaDrPose;

		_ifDrCaculated = other._ifDrCaculated;

		_sumMoveDis = other._sumMoveDis;
		_sumMoveDeg = other._sumMoveDeg;

		return *this;
	}

	BaseSensorDeviceData &BaseSensorDeviceData::operator=(const BaseSensorDeviceData& other) {
		_sensorDevice = other._sensorDevice;

		_synDataListLogicVar = other._synDataListLogicVar;
		_synDataListDr = other._synDataListDr;

		return *this;
	}

	OdoDeviceData& OdoDeviceData::operator=(const OdoDeviceData& other) {
		_sensorDevice = other._sensorDevice;

		_synDataListLogicVar = other._synDataListLogicVar;
		_synDataListDr = other._synDataListDr;

		_ifIsOdoStatic = other._ifIsOdoStatic;

		_sensorData = other._sensorData;

		return *this;
	}

	ImuDeviceData &ImuDeviceData::operator=(const ImuDeviceData& other) {
		_sensorDevice = other._sensorDevice;

		_synDataListLogicVar = other._synDataListLogicVar;
		_synDataListDr = other._synDataListDr;

		_sensorData = other._sensorData;

		return *this;
	}

	Laser2dDeviceData &Laser2dDeviceData::operator=(const Laser2dDeviceData& other) {
		_sensorDevice = other._sensorDevice;

		_synDataListLogicVar = other._synDataListLogicVar;
		_synDataListDr = other._synDataListDr;

		_sensorData = other._sensorData;

		return *this;
	}

	Laser3dDeviceData &Laser3dDeviceData::operator=(const Laser3dDeviceData& other) {
		_sensorDevice = other._sensorDevice;

		_synDataListLogicVar = other._synDataListLogicVar;
		_synDataListDr = other._synDataListDr;

		_sensorData = other._sensorData;

		return *this;
	}

#ifdef _USE_OPENCV_
	CameraDeviceData& CameraDeviceData::operator=(const CameraDeviceData& other) {
		_sensorDevice = other._sensorDevice;

		_synDataListLogicVar = other._synDataListLogicVar;
		_synDataListDr = other._synDataListDr;

		_sensorData = other._sensorData;

		return *this;
	}
#endif

}