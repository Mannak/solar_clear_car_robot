/************************************************************************
Author: huanghong
Date: 2016-10-31
************************************************************************/

#ifndef _SYN_SENSOR_DATA_
#define _SYN_SENSOR_DATA_

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/function.hpp>
#include "Localization\SensorDataFrame\SensorData.h"
#include "Localization\SensorDataFrame\SensorDeviceData.h"
#include "Localization\SensorDataFrame\AxisTransform.h"
#include <list>
#include <map>
#include <vector>
#include "ComUtils\TimeStampAbs.h"


namespace Localization{

	typedef enum {
		DIMENSION2 = 0,
		DIMENSION3
	} DIMENSION_SITUATION;

	class SensorDeviceAttribute {
	public:
		SensorDeviceAttribute() {
			_ifReceivedData = false;

			_latestSensorDeviceState = true;
			_ifSensorDeviceDataOverTime = false;

			_lastSensorLocalTime = -1;

			_lastDeviceSynDrSensorLocalTime = -1;
		}

		bool _ifReceivedData;

		AxisTransform _axisTransformToBaseOdo;

		bool _latestSensorDeviceState;
		bool _ifSensorDeviceDataOverTime;

		double _lastSensorLocalTime;
		boost::shared_ptr<BaseSensorDeviceData> _lastDeviceData;

		Pose3D _lastDeviceSynDrPose;
		double _lastDeviceSynDrSensorLocalTime;
		boost::shared_ptr<BaseSensorDeviceData> _lastDeviceSynDrData;
	};

	class BiasEsti;
	class StaticJudgement;
	class AttitudeEstimation;
	class SynSensorData {
	private:
		SynSensorData();
	public:
		~SynSensorData();

	public:
		static SynSensorData *instance() {
			static SynSensorData ins;
			return &ins;
		}

		void init(int bias_max_data_size, int bias_level_up_data_size,
			double static_stop_speed_thres, bool if_run_on_time,
			double sensor_max_tolerate_delay_time = 1, int odo_static_judge_num_margin = 20,
			double sensor_max_valid_dr_time = 2, double sensor_over_time_margin = 0.5,
			TIME_STAMP_TYPE time_stamp_type = TIME_STAMP_TYPE::ABS_TIME_STAMP);

		void insertSensorData(boost::shared_ptr<BaseSensorDeviceData> sensor_device_data_ptr);

	private:
		boost::mutex _mutSensorDeviceDataPtrList_Syn;
		std::list<boost::shared_ptr<BaseSensorDeviceData> > _sensorDeviceDataPtrList_Syn;

		std::vector<std::list<boost::shared_ptr<BaseSensorDeviceData> >::iterator> _odoStaticMarginIterVec;

	public:
		void registrateSensorDevice(SENSOR_TYPE sensor_type, int device_id = 0);
		int getSensorDeviceNumByType(SENSOR_TYPE sensor_type);
		void setSensorDeviceCalibrationValue(double x_in_base, double y_in_base, double z_in_base,
			double yaw_to_base, double pitch_to_base, double roll_to_base, SENSOR_TYPE sensor_type, int device_id = 0);
		void bindCallBackForBaseDeviceDataOutput(boost::function<void(boost::shared_ptr<BaseSensorDeviceData>)> call_back_base_device_data_output);
		void bindCallBackForGyroBiasOutput(boost::function<void(double gx_bias_deg_s, double gy_bias_deg_s, double gz_bias_deg_s)> call_back_gyro_bias_output);

	private:
		Pose3D _synListDrPose_lastDrPose;
		Pose3D _synListDrPose;
		double _synListDrPoseTime;
		boost::mutex _mutSynListDrPose;
		void setSynListDrPose(Pose3D last_dr_pose, Pose3D dr_pose, double dr_pose_time) {
			boost::mutex::scoped_lock l(_mutSynListDrPose);
			_synListDrPose_lastDrPose = last_dr_pose;
			_synListDrPose = dr_pose;
			_synListDrPoseTime = dr_pose_time;
		}
		Pose3D caculateSynListDrPose();

		boost::mutex _mutLastDeviceSynDr;
		void setLastDeviceSynDrValue(SensorDevice sensor_device, boost::shared_ptr<BaseSensorDeviceData> device_data_ptr, double local_time, Pose3D dr_pose) {
			boost::mutex::scoped_lock l(_mutLastDeviceSynDr);
			_sensorDeviceAttributeMap[sensor_device]._lastDeviceSynDrPose = dr_pose;
			_sensorDeviceAttributeMap[sensor_device]._lastDeviceSynDrSensorLocalTime = local_time;
			_sensorDeviceAttributeMap[sensor_device]._lastDeviceSynDrData = device_data_ptr;
		}

	public:
		void getSynListDrPose(Pose3D &last_dr_pose, Pose3D &pose, double &pose_time) {
			boost::mutex::scoped_lock l(_mutSynListDrPose);
			last_dr_pose = _synListDrPose_lastDrPose;
			pose = _synListDrPose;
			pose_time = _synListDrPoseTime;
		}

		void getLastDeviceSynDrValue(SensorDevice sensor_device, boost::shared_ptr<BaseSensorDeviceData>& device_data_ptr, double& local_time, Pose3D& dr_pose) {
			boost::mutex::scoped_lock l(_mutLastDeviceSynDr);
			dr_pose = _sensorDeviceAttributeMap[sensor_device]._lastDeviceSynDrPose;
			local_time = _sensorDeviceAttributeMap[sensor_device]._lastDeviceSynDrSensorLocalTime;
			device_data_ptr = _sensorDeviceAttributeMap[sensor_device]._lastDeviceSynDrData;
		}

		boost::shared_ptr<BaseSensorDeviceData> getLastSensorDeviceData(SENSOR_TYPE sensor_type, int device_id) {
			SensorDevice sensor_device(sensor_type, device_id);
			if (_sensorDeviceAttributeMap.find(sensor_device) == _sensorDeviceAttributeMap.end()) {
				boost::shared_ptr<BaseSensorDeviceData> empty_ptr;
				return empty_ptr;
			}
			else {
				return _sensorDeviceAttributeMap[sensor_device]._lastDeviceData;
			}
		}

	private:
		std::map<SensorDevice, SensorDeviceAttribute> _sensorDeviceAttributeMap;
		bool _ifHaveZeroOdoDevice;
		bool _ifHaveZeroImuDevice;

	public:
		const std::map<SensorDevice, SensorDeviceAttribute>& getSensorDeviceAtributeMap() {
			return _sensorDeviceAttributeMap;
		}

	private:
		void dataHandleLoop();
		double _lastDrTime;
		boost::shared_ptr<OdoDeviceData> _lastDrOdoDeviceDataPtr;
		boost::shared_ptr<OdoDeviceData> _lastLastDrOdoDeviceDataPtr;
		boost::shared_ptr<ImuDeviceData> _lastDrImuDeviceDataPtr;
		boost::shared_ptr<ImuDeviceData> _lastLastDrImuDeviceDataPtr;
		Pose3D _lastDrPose;
		void caculatePosByOdo(double local_time, double dr_dt, Pos3D &delta_dr_pos, OdoSensorData *last_odo_sensor_data, boost::shared_ptr<OdoDeviceData> last_last_dr_odo_device_data_ptr, Pose3D& last_dr_pose);
		void caculateDegByOdo(double local_time, double dr_dt, Orient3D &delta_dr_orient, OdoSensorData *last_odo_sensor_data, boost::shared_ptr<OdoDeviceData> last_last_dr_odo_device_data_ptr, Pose3D& last_dr_pose);
		void caculateDegByImu(double local_time, double dr_dt, Orient3D &delta_dr_orient, ImuSensorData *last_imu_sensor_data, boost::shared_ptr<ImuDeviceData> last_last_dr_imu_device_data_ptr, Pose3D& last_dr_pose);

		bool _ifBindCallBackBaseDeviceDataOutput;
		boost::function<void(boost::shared_ptr<BaseSensorDeviceData>)> _callBackBaseDeviceDataOutput;
		bool _ifBindCallBackGyroBiasOutput;
		boost::function<void(double gx_bias_deg_s, double gy_bias_deg_s, double gz_bias_deg_s)> _callBackGyroBiasOutput;

	private:
		BiasEsti *_gyroBias[3];
		StaticJudgement *_staticJudgement;
		AttitudeEstimation *_attitudeEstimation;

	private:
		double _confSensorMaxTolerateDelayTime;
		int _confOdoStaticJudgeNumMargin;
		double _confSensorMaxValidDrTime;
		double _confSensorDataOverTimeMargin;

	private:
		TIME_STAMP_TYPE _usedTimeStampType;
	public:
		double getUsedTimeStamp();

	public:
		bool ifReceiveAllSensorData() {
			return _ifReceiveAllSensorData;
		}
	private:
		bool _ifReceiveAllSensorData;
		bool _ifRunOnTime;

	private:
        double _sumMoveDis;
		double _sumMoveDeg;
		const double _maxSumMoveDis;
		const double _maxSumMoveDeg;
	public:
		double getDeltaMoveDis(boost::shared_ptr<BaseSensorDeviceData> pre_device_data, boost::shared_ptr<BaseSensorDeviceData> post_device_data);
		double getDeltaMoveDeg(boost::shared_ptr<BaseSensorDeviceData> pre_device_data, boost::shared_ptr<BaseSensorDeviceData> post_device_data);

	private:
		DIMENSION_SITUATION _dimensionSituation;
	public:
		void setDimensionSituation(DIMENSION_SITUATION ds) {
			_dimensionSituation = ds;
		}
	};


#define SYN_SENSOR_DATA SynSensorData::instance()


}
#endif