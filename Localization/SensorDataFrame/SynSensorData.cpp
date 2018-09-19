#include "SynSensorData.h"
#include "SensorDataHandle.h"
#include "AttitudeEstimation.h"
#include <boost/thread.hpp>
#include <iostream>
#include "Localization\PoseUtils\rot_fuc_deg.h"
#include "Localization\PoseUtils\rot_fuc_deg2d.h"
#include "Localization\PoseUtils\angle_math_utils.h"
#include <fstream>


namespace Localization{

	using namespace angle_math_utils;
	SynSensorData::SynSensorData() :_lastDrPose(0, 0, 0, 0, 0, 0), _maxSumMoveDis(5000000), _maxSumMoveDeg(5000000) {
		_gyroBias[0] = new BiasEsti;   _gyroBias[1] = new BiasEsti;   _gyroBias[2] = new BiasEsti;
		_staticJudgement = new StaticJudgement;
		_attitudeEstimation = new AttitudeEstimation;
		_ifHaveZeroOdoDevice = false;
		_ifHaveZeroImuDevice = false;

		_lastDrTime = -1;

		_confSensorMaxTolerateDelayTime = 1;
		_confOdoStaticJudgeNumMargin = 20;
		_confSensorMaxValidDrTime = 2;
		_confSensorDataOverTimeMargin = 0.5;

		_usedTimeStampType = TIME_STAMP_TYPE::ABS_TIME_STAMP;

		_ifReceiveAllSensorData = false;
		_ifRunOnTime = false;

		_ifBindCallBackBaseDeviceDataOutput = false;
		_ifBindCallBackGyroBiasOutput = false;

		_sumMoveDis = 0;
		_sumMoveDeg = 0;

		_dimensionSituation = DIMENSION_SITUATION::DIMENSION2;
	}

	SynSensorData::~SynSensorData() {
		if (_gyroBias[0]) {
			delete _gyroBias[0];
			_gyroBias[0] = NULL;
		}
		if (_gyroBias[1]) {
			delete _gyroBias[1];
			_gyroBias[1] = NULL;
		}
		if (_gyroBias[2]) {
			delete _gyroBias[2];
			_gyroBias[2] = NULL;
		}
		if (_staticJudgement) {
			delete _staticJudgement;
			_staticJudgement = NULL;
		}
		if (_attitudeEstimation) {
			delete _attitudeEstimation;
			_attitudeEstimation = NULL;
		}
	}

	void SynSensorData::init(int bias_max_data_size, int bias_level_up_data_size,
		double static_stop_speed_thres, bool if_run_on_time,
		double sensor_max_tolerate_delay_time, int odo_static_judge_num_margin,
		double sensor_max_valid_dr_time, double sensor_over_time_margin,
		TIME_STAMP_TYPE time_stamp_type) {
		BiasEsti::init(bias_max_data_size, bias_level_up_data_size);
		_staticJudgement->init(static_stop_speed_thres);
		_attitudeEstimation->init();
		_attitudeEstimation->initState(0, 0);

		_ifRunOnTime = if_run_on_time;

		_confSensorMaxTolerateDelayTime = sensor_max_tolerate_delay_time;
		_confOdoStaticJudgeNumMargin = odo_static_judge_num_margin;
		_confSensorMaxValidDrTime = sensor_max_valid_dr_time;
		_confSensorDataOverTimeMargin = sensor_over_time_margin;

		_usedTimeStampType = time_stamp_type;

		boost::thread t(&SynSensorData::dataHandleLoop, this);
	}

	void SynSensorData::registrateSensorDevice(SENSOR_TYPE sensor_type, int device_id) {
		SensorDevice sensor_device(sensor_type, device_id);
		_sensorDeviceAttributeMap[sensor_device] = SensorDeviceAttribute();

		if (sensor_device.ifIsZeroIdSensor(SENSOR_TYPE::ODO)) {
			_ifHaveZeroOdoDevice = true;
		}
		if (sensor_device.ifIsZeroIdSensor(SENSOR_TYPE::IMU)) {
			_ifHaveZeroImuDevice = true;
		}
	}

	int SynSensorData::getSensorDeviceNumByType(SENSOR_TYPE sensor_type) {
		int num = 0;
		for (std::map<SensorDevice, SensorDeviceAttribute>::iterator iter = _sensorDeviceAttributeMap.begin();
			iter != _sensorDeviceAttributeMap.end(); ++iter) {
			SensorDevice sensor_device = iter->first;
			if (sensor_device._sensorType == sensor_type) {
				num++;
			}
		}
		return num;
	}

	void SynSensorData::setSensorDeviceCalibrationValue(double x_in_base, double y_in_base, double z_in_base,
		double yaw_to_base, double pitch_to_base, double roll_to_base, SENSOR_TYPE sensor_type, int device_id) {
		SensorDevice sensor_device(sensor_type, device_id);
		if (_sensorDeviceAttributeMap.find(sensor_device) == _sensorDeviceAttributeMap.end()) {
			_sensorDeviceAttributeMap[sensor_device] = SensorDeviceAttribute();
		}
		_sensorDeviceAttributeMap[sensor_device]._axisTransformToBaseOdo.setCalibrationValue(x_in_base, y_in_base, z_in_base,
			yaw_to_base, pitch_to_base, roll_to_base);
	}

	void SynSensorData::bindCallBackForBaseDeviceDataOutput(boost::function<void(boost::shared_ptr<BaseSensorDeviceData>)> call_back_base_device_data_output) {
		_callBackBaseDeviceDataOutput = call_back_base_device_data_output;
		_ifBindCallBackBaseDeviceDataOutput = true;
	}

	void SynSensorData::bindCallBackForGyroBiasOutput(boost::function<void(double gx_bias_deg_s, double gy_bias_deg_s, double gz_bias_deg_s)> call_back_gyro_bias_output) {
		_callBackGyroBiasOutput = call_back_gyro_bias_output;
		_ifBindCallBackGyroBiasOutput = true;
	}

	//syn
	//judge_data_if_pos_locked
	//judge if is static
	//get locked static data list
	void SynSensorData::insertSensorData(boost::shared_ptr<BaseSensorDeviceData> sensor_device_data_ptr) {
		double max_delay_time = _confSensorMaxTolerateDelayTime;

		SENSOR_TYPE sensor_type = sensor_device_data_ptr->getSensorType();
		SensorDevice& sensor_device = sensor_device_data_ptr->_sensorDevice;
		BaseSensorData* sensor_data_ptr = sensor_device_data_ptr->getSensorDataPtr();

		_sensorDeviceAttributeMap[sensor_device]._latestSensorDeviceState = sensor_data_ptr->_state;

		if (sensor_data_ptr->_state == false) {
			return;
		}

		boost::mutex::scoped_lock l(_mutSensorDeviceDataPtrList_Syn);//lock insert

		if (_ifReceiveAllSensorData == false) {
			_sensorDeviceAttributeMap[sensor_device]._ifReceivedData = true;
			for (std::map<SensorDevice, SensorDeviceAttribute>::iterator iter = _sensorDeviceAttributeMap.begin(); iter != _sensorDeviceAttributeMap.end(); ++iter) {
				if (iter->second._ifReceivedData == false) {
					return;
				}
			}

			std::cout << "SynSensorData: receive all sensor data!------------------------" << std::endl;
			_ifReceiveAllSensorData = true;
		}

		{//stop misorder sensor data
			double last_sensor_local_time = _sensorDeviceAttributeMap[sensor_device]._lastSensorLocalTime;
			if (last_sensor_local_time >= sensor_data_ptr->_localTime) {
				return;//same sensor misorder
			}

			if (_sensorDeviceDataPtrList_Syn.size() > 0) {
				double delay_t = _sensorDeviceDataPtrList_Syn.back()->getSensorDataPtr()->_localTime - sensor_data_ptr->_localTime;
				if (delay_t >= max_delay_time) {
					std::cout << "SynSensorData: " << sensor_data_ptr->getNameStr() << " data delay over margin " << delay_t - max_delay_time << std::endl;
					return;//cur data delay too much
				}
			}

			if (_ifHaveZeroOdoDevice == true
				&& sensor_device.ifIsZeroIdSensor(SENSOR_TYPE::ODO) == false) {
				if (_sensorDeviceAttributeMap[SensorDevice(SENSOR_TYPE::ODO, 0)]._lastSensorLocalTime < 0) {
					return;//must odo enter first
				}

				double temp_t = sensor_data_ptr->_localTime - _sensorDeviceAttributeMap[SensorDevice(SENSOR_TYPE::ODO, 0)]._lastSensorLocalTime;
				if (temp_t > max_delay_time + 5) {
					std::cout << "SynSensorData: " << sensor_data_ptr->getNameStr() << " time-pre-odo0 " << temp_t << std::endl;
					return;//if odo stop, then other sensor data won't enter too much
				}
			}

			_sensorDeviceAttributeMap[sensor_device]._lastSensorLocalTime = sensor_data_ptr->_localTime;
			_sensorDeviceAttributeMap[sensor_device]._lastDeviceData = sensor_device_data_ptr;
		}

		//insert data
		if (_sensorDeviceDataPtrList_Syn.size() == 0 || sensor_data_ptr->_localTime >= _sensorDeviceDataPtrList_Syn.back()->getSensorDataPtr()->_localTime) {
			_sensorDeviceDataPtrList_Syn.push_back(sensor_device_data_ptr);
		}
		else if (sensor_data_ptr->_localTime <= _sensorDeviceDataPtrList_Syn.front()->getSensorDataPtr()->_localTime) {
			_sensorDeviceDataPtrList_Syn.push_front(sensor_device_data_ptr);
		}
		else {
			for (std::list<boost::shared_ptr<BaseSensorDeviceData> >::reverse_iterator iter = _sensorDeviceDataPtrList_Syn.rbegin(); iter != _sensorDeviceDataPtrList_Syn.rend(); ++iter) {
				std::list<boost::shared_ptr<BaseSensorDeviceData> >::reverse_iterator front_iter = iter;
				front_iter++;
				if (front_iter != _sensorDeviceDataPtrList_Syn.rend()) {
					if ((*front_iter)->getSensorDataPtr()->_localTime <= sensor_data_ptr->_localTime && (*iter)->getSensorDataPtr()->_localTime > sensor_data_ptr->_localTime) {
						std::list<boost::shared_ptr<BaseSensorDeviceData> >::iterator base_iter(front_iter.base());
						_sensorDeviceDataPtrList_Syn.insert(base_iter, sensor_device_data_ptr);
						break;
					}
				}
			}
		}

		//judge odo static
		if (sensor_device.ifIsZeroIdSensor(SENSOR_TYPE::ODO)) {
			OdoSensorData* odo_data_ptr = dynamic_cast<OdoSensorData *>(sensor_data_ptr);
			_staticJudgement->judgeStatic(odo_data_ptr->_wheelSpeedVec);

			boost::shared_ptr<OdoDeviceData> odo_device_data_ptr = boost::dynamic_pointer_cast<OdoDeviceData>(sensor_device_data_ptr);
			odo_device_data_ptr->_ifIsOdoStatic = _staticJudgement->isStatic();
		}

		//lock data pos
		std::vector<SensorDevice> different_device_type_vec;
		for (std::list<boost::shared_ptr<BaseSensorDeviceData> >::reverse_iterator iter = _sensorDeviceDataPtrList_Syn.rbegin(); iter != _sensorDeviceDataPtrList_Syn.rend(); ++iter) {
			boost::shared_ptr<BaseSensorDeviceData> base_sensor_device_data_ptr = (*iter);
			if (base_sensor_device_data_ptr->_synDataListLogicVar._ifPosInDataListLocked == true) {
				break;
			}

			if (different_device_type_vec.size() >= _sensorDeviceAttributeMap.size() ||
				(base_sensor_device_data_ptr->getSensorDataPtr()->_localTime + max_delay_time) < _sensorDeviceDataPtrList_Syn.back()->getSensorDataPtr()->_localTime) {
				base_sensor_device_data_ptr->_synDataListLogicVar._ifPosInDataListLocked = true;
				continue;
			}

			SensorDevice cur_type = base_sensor_device_data_ptr->_sensorDevice;
			bool if_had_this_type = false;
			for (int i = 0; i < different_device_type_vec.size(); ++i) {
				if (different_device_type_vec[i] == cur_type) {
					if_had_this_type = true;
					break;
				}
			}
			if (if_had_this_type == false) {
				different_device_type_vec.push_back(cur_type);
			}
		}

		//judge if is static
		if (_sensorDeviceDataPtrList_Syn.size() == 0 || _sensorDeviceDataPtrList_Syn.front()->_synDataListLogicVar._ifPosInDataListLocked == false) {
			return;
		}
		if (_ifHaveZeroOdoDevice == true && _confOdoStaticJudgeNumMargin >= 2) {
			static std::list<boost::shared_ptr<BaseSensorDeviceData> >::iterator static_judge_iter = _sensorDeviceDataPtrList_Syn.begin();
			static int static_num_count = 0;
			int static_judge_margin = _confOdoStaticJudgeNumMargin;

			while (static_judge_iter != _sensorDeviceDataPtrList_Syn.end()
				&& (*static_judge_iter)->_synDataListLogicVar._ifPosInDataListLocked == true) {
				if ((*static_judge_iter)->ifIsZeroIdSensor(SENSOR_TYPE::ODO)) {
					boost::shared_ptr<OdoDeviceData> odo_device_data_ptr = boost::dynamic_pointer_cast<OdoDeviceData>(*static_judge_iter);

					_odoStaticMarginIterVec.push_back(static_judge_iter);
					static_num_count += odo_device_data_ptr->_ifIsOdoStatic ? 1 : 0;

					if (_odoStaticMarginIterVec.size() > static_judge_margin) {
						boost::shared_ptr<BaseSensorDeviceData> begin_device_data_ptr = *_odoStaticMarginIterVec.front();
						boost::shared_ptr<OdoDeviceData> begin_odo_device_ptr = boost::dynamic_pointer_cast<OdoDeviceData>(begin_device_data_ptr);
						static_num_count -= begin_odo_device_ptr->_ifIsOdoStatic ? 1 : 0;
						_odoStaticMarginIterVec.erase(_odoStaticMarginIterVec.begin());
					}

					if (static_num_count == static_judge_margin) {//margin middle is static
						int half_length = _odoStaticMarginIterVec.size() / 2;
						std::list<boost::shared_ptr<BaseSensorDeviceData> >::iterator is_static_begin_iter = _odoStaticMarginIterVec[half_length - 1];
						std::list<boost::shared_ptr<BaseSensorDeviceData> >::iterator is_static_end_iter = _odoStaticMarginIterVec[half_length];
						for (std::list<boost::shared_ptr<BaseSensorDeviceData> >::iterator iter = is_static_begin_iter; iter != is_static_end_iter; ++iter) {
							(*iter)->_synDataListLogicVar._ifIsStatic = true;
						}
					}
				}
				static_judge_iter++;
			}
		}
	}

	void SynSensorData::dataHandleLoop() {
		std::cout << "SynSensorData: " << "data handle loop..." << std::endl;

		while (1) {
			if (_ifReceiveAllSensorData == false) {
				boost::this_thread::sleep(boost::posix_time::milliseconds(30));
				continue;
			}

			//judge sensor data over time
			if (_ifRunOnTime == true) {
				double cur_local_time = getUsedTimeStamp();

				for (std::map<SensorDevice, SensorDeviceAttribute>::iterator iter = _sensorDeviceAttributeMap.begin(); iter != _sensorDeviceAttributeMap.end(); ++iter) {
					SensorDevice sensor_device = iter->first;
					double last_sensor_local_time = iter->second._lastSensorLocalTime;
					double over_time = cur_local_time - last_sensor_local_time;
					if (last_sensor_local_time > 0 && over_time > _confSensorDataOverTimeMargin) {
						_sensorDeviceAttributeMap[sensor_device]._ifSensorDeviceDataOverTime = true;
					}
					else {
						_sensorDeviceAttributeMap[sensor_device]._ifSensorDeviceDataOverTime = false;
					}
				}
			}

			//-----------------_ifHaveZeroOdoDevice == false
			if (_ifHaveZeroOdoDevice == false) {
				static bool if_continue_sleep = false;
				if (if_continue_sleep == true) {
					boost::this_thread::sleep(boost::posix_time::milliseconds(10));
				}
				std::list<boost::shared_ptr<BaseSensorDeviceData> >::iterator syn_begin;
				{
					boost::mutex::scoped_lock l(_mutSensorDeviceDataPtrList_Syn);
					if_continue_sleep = _sensorDeviceDataPtrList_Syn.front()->_synDataListLogicVar._ifPosInDataListLocked == false;
					if (if_continue_sleep == true) {
						continue;
					}
					syn_begin = _sensorDeviceDataPtrList_Syn.begin();
				}

				boost::shared_ptr<BaseSensorDeviceData> base_sensor_device_data_ptr = *syn_begin;
				//call back model
				if (_ifBindCallBackBaseDeviceDataOutput == true) {
					_callBackBaseDeviceDataOutput(base_sensor_device_data_ptr);
				}

				boost::mutex::scoped_lock l(_mutSensorDeviceDataPtrList_Syn);
				_sensorDeviceDataPtrList_Syn.erase(syn_begin);
				continue;
			}
			else {
				//-----------------_ifHaveZeroOdoDevice == true
				static bool if_continue_sleep = false;
				if (if_continue_sleep == true) {
					boost::this_thread::sleep(boost::posix_time::milliseconds(10));
				}
				boost::shared_ptr<BaseSensorDeviceData> syn_begin_device_data_ptr, odo_static_margin_begin_device_data_ptr;
				{
					boost::mutex::scoped_lock l(_mutSensorDeviceDataPtrList_Syn);
					if_continue_sleep = _odoStaticMarginIterVec.size() == 0;
					if (if_continue_sleep == true) {
						continue;
					}
					std::list<boost::shared_ptr<BaseSensorDeviceData> >::iterator syn_begin = _sensorDeviceDataPtrList_Syn.begin();
					std::list<boost::shared_ptr<BaseSensorDeviceData> >::iterator odo_static_margin_begin = _odoStaticMarginIterVec.front();
					syn_begin_device_data_ptr = *syn_begin;
					odo_static_margin_begin_device_data_ptr = *odo_static_margin_begin;

					//-----get syn_dr_pose
					Pose3D syn_dr_pose = caculateSynListDrPose();
					double syn_dr_pose_time = _sensorDeviceDataPtrList_Syn.back()->getSensorDataPtr()->_localTime;
					setSynListDrPose(_lastDrPose, syn_dr_pose, syn_dr_pose_time);
				}

				if (syn_begin_device_data_ptr == odo_static_margin_begin_device_data_ptr) {
					boost::this_thread::sleep(boost::posix_time::milliseconds(10));
					continue;
				}

				//---
				BaseSensorData *base_sensor_data_ptr = syn_begin_device_data_ptr->getSensorDataPtr();
				SensorDevice &sensor_device = syn_begin_device_data_ptr->_sensorDevice;

				//gyro bias
				if ((syn_begin_device_data_ptr)->ifIsZeroIdSensor(SENSOR_TYPE::IMU)) {
					boost::shared_ptr<ImuDeviceData> imu_device_ptr = boost::dynamic_pointer_cast<ImuDeviceData>(syn_begin_device_data_ptr);
					ImuSensorData* imu_sensor_data = imu_device_ptr->getImuSensorDataPtr();
					if ((syn_begin_device_data_ptr)->_synDataListLogicVar._ifIsStatic == true) {
						_gyroBias[0]->insertData(imu_sensor_data->_gyroDeg[0]);
						_gyroBias[1]->insertData(imu_sensor_data->_gyroDeg[1]);
						_gyroBias[2]->insertData(imu_sensor_data->_gyroDeg[2]);
					}

					imu_sensor_data->_gyroDeg[0] -= _gyroBias[0]->getBias();
					imu_sensor_data->_gyroDeg[1] -= _gyroBias[1]->getBias();
					imu_sensor_data->_gyroDeg[2] -= _gyroBias[2]->getBias();

					if (_ifBindCallBackGyroBiasOutput == true && (syn_begin_device_data_ptr)->_synDataListLogicVar._ifIsStatic == true) {
						static double last_bias_out_t = base_sensor_data_ptr->_localTime;
						double cur_bias_out_t = base_sensor_data_ptr->_localTime;
						double dt = cur_bias_out_t - last_bias_out_t;
						if (dt > 10) {
							//std::cout << "gyro bias: " << _gyroBias[0]->getBias()<<" "<< _gyroBias[1]->getBias()<<" "<< _gyroBias[2]->getBias() << std::endl;
							last_bias_out_t = cur_bias_out_t;
							_callBackGyroBiasOutput(_gyroBias[0]->getBias(), _gyroBias[1]->getBias(), _gyroBias[2]->getBias());
						}
					}
				}

				//calib sensor data
				base_sensor_data_ptr->axisTransformSensorData(_sensorDeviceAttributeMap[sensor_device]._axisTransformToBaseOdo);

				Pose3D delta_dr_pose(0, 0, 0, 0, 0, 0);
				if (_lastDrTime > 0) {
					double local_time = base_sensor_data_ptr->_localTime;
					double dr_dt = local_time - _lastDrTime;
					if ((syn_begin_device_data_ptr)->_synDataListLogicVar._ifIsStatic == true) {//static use
						dr_dt = 0;
					}
					if (_lastDrOdoDeviceDataPtr != NULL) {
						caculatePosByOdo(local_time, dr_dt, delta_dr_pose._pos, _lastDrOdoDeviceDataPtr->getOdoSensorDataPtr(), _lastLastDrOdoDeviceDataPtr, _lastDrPose);
						if (_ifHaveZeroImuDevice == false) {
							caculateDegByOdo(local_time, dr_dt, delta_dr_pose._orient, _lastDrOdoDeviceDataPtr->getOdoSensorDataPtr(), _lastLastDrOdoDeviceDataPtr, _lastDrPose);
						}
					}
					if (_ifHaveZeroImuDevice == true && _lastDrImuDeviceDataPtr != NULL) {
						caculateDegByImu(local_time, dr_dt, delta_dr_pose._orient, _lastDrImuDeviceDataPtr->getImuSensorDataPtr(), _lastLastDrImuDeviceDataPtr, _lastDrPose);
					}
				}

				//-----
				_sumMoveDis += delta_dr_pose._pos.getNormal();
				_sumMoveDeg += fabs(delta_dr_pose._orient._eulerDeg[0]);
				if (_sumMoveDis > _maxSumMoveDis) {
					_sumMoveDis -= _maxSumMoveDis;
				}
				if (_sumMoveDeg > _maxSumMoveDeg) {
					_sumMoveDeg -= _maxSumMoveDeg;
				}

				//attitude estimation to do
				if (_dimensionSituation == DIMENSION_SITUATION::DIMENSION3 &&(syn_begin_device_data_ptr)->ifIsZeroIdSensor(SENSOR_TYPE::IMU)) {
					boost::shared_ptr<ImuDeviceData> imu_device_ptr = boost::dynamic_pointer_cast<ImuDeviceData>(syn_begin_device_data_ptr);
					ImuSensorData* imu_sensor_data = imu_device_ptr->getImuSensorDataPtr();
					double dt = imu_sensor_data->_intvalTime;
					if (_lastDrImuDeviceDataPtr != NULL) {
						dt = imu_sensor_data->_localTime - _lastDrImuDeviceDataPtr->getImuSensorDataPtr()->_localTime;
					}
					_attitudeEstimation->doFilter(imu_sensor_data->_gyroDeg[0], imu_sensor_data->_gyroDeg[1], imu_sensor_data->_gyroDeg[2], 
						imu_sensor_data->_acc[0], imu_sensor_data->_acc[1], imu_sensor_data->_acc[2], dt);
				}

				if ((syn_begin_device_data_ptr)->ifIsZeroIdSensor(SENSOR_TYPE::ODO)) {
					boost::shared_ptr<OdoDeviceData> odo_device_ptr = boost::dynamic_pointer_cast<OdoDeviceData>(syn_begin_device_data_ptr);
					_lastLastDrOdoDeviceDataPtr = _lastDrOdoDeviceDataPtr;
					_lastDrOdoDeviceDataPtr = odo_device_ptr;
				}
				if ((syn_begin_device_data_ptr)->ifIsZeroIdSensor(SENSOR_TYPE::IMU)) {
					boost::shared_ptr<ImuDeviceData> imu_device_ptr = boost::dynamic_pointer_cast<ImuDeviceData>(syn_begin_device_data_ptr);
					_lastLastDrImuDeviceDataPtr = _lastDrImuDeviceDataPtr;
					_lastDrImuDeviceDataPtr = imu_device_ptr;
				}

				_lastDrTime = base_sensor_data_ptr->_localTime;

				syn_begin_device_data_ptr->_synDataListDr._drPose = _lastDrPose;
				syn_begin_device_data_ptr->_synDataListDr._attitudePitchDeg = (_dimensionSituation == DIMENSION_SITUATION::DIMENSION3 ? _attitudeEstimation->getPitchDeg() : 0);
				syn_begin_device_data_ptr->_synDataListDr._attitudeRollDeg = (_dimensionSituation == DIMENSION_SITUATION::DIMENSION3 ? _attitudeEstimation->getRollDeg() : 0);
				syn_begin_device_data_ptr->_synDataListDr._deltaDrPose = delta_dr_pose;
				syn_begin_device_data_ptr->_synDataListDr._ifDrCaculated = true;
                syn_begin_device_data_ptr->_synDataListDr._sumMoveDis = _sumMoveDis;
				syn_begin_device_data_ptr->_synDataListDr._sumMoveDeg = _sumMoveDeg;

				{
					if (_ifBindCallBackBaseDeviceDataOutput == true) {
						//call back model
						_callBackBaseDeviceDataOutput(syn_begin_device_data_ptr);
					}

					boost::mutex::scoped_lock l(_mutSensorDeviceDataPtrList_Syn);
					_sensorDeviceDataPtrList_Syn.erase(_sensorDeviceDataPtrList_Syn.begin());
				}
			}

		}//while(1)
	}

	Pose3D SynSensorData::caculateSynListDrPose() {
		std::map<SensorDevice, Pose3D> syn_dr_pose_map;
		std::map<SensorDevice, double> syn_dr_local_time_map;
		std::map<SensorDevice, boost::shared_ptr<BaseSensorDeviceData> > syn_dr_device_data_map;


		boost::shared_ptr<OdoDeviceData> last_dr_odo_device_data_ptr = _lastDrOdoDeviceDataPtr;
		boost::shared_ptr<OdoDeviceData> last_last_dr_odo_device_data_ptr = _lastLastDrOdoDeviceDataPtr;
		boost::shared_ptr<ImuDeviceData> last_dr_imu_device_data_ptr = _lastDrImuDeviceDataPtr;
		boost::shared_ptr<ImuDeviceData> last_last_dr_imu_device_data_ptr = _lastLastDrImuDeviceDataPtr;

		double last_dr_time = _lastDrTime;
		Pose3D syn_dr_pose, syn_delta_dr_pose;
		for (std::list<boost::shared_ptr<BaseSensorDeviceData> >::iterator iter = _sensorDeviceDataPtrList_Syn.begin();
			iter != _sensorDeviceDataPtrList_Syn.end(); ++iter) {
			BaseSensorData *base_sensor_data_ptr = (*iter)->getSensorDataPtr();
			SensorDevice &sensor_device = (*iter)->_sensorDevice;
			if (last_dr_time > 0) {
				double local_time = base_sensor_data_ptr->_localTime;
				double dr_dt = local_time - last_dr_time;
				if (last_dr_odo_device_data_ptr != NULL) {
					caculatePosByOdo(local_time, dr_dt, syn_delta_dr_pose._pos, last_dr_odo_device_data_ptr->getOdoSensorDataPtr(), last_last_dr_odo_device_data_ptr, syn_dr_pose);
					if (_ifHaveZeroImuDevice == false) {
						caculateDegByOdo(local_time, dr_dt, syn_delta_dr_pose._orient, last_dr_odo_device_data_ptr->getOdoSensorDataPtr(), last_last_dr_odo_device_data_ptr, syn_dr_pose);
					}
				}
				if (_ifHaveZeroImuDevice == true && last_dr_imu_device_data_ptr != NULL) {
					ImuSensorData imu_sensor_data = *(last_dr_imu_device_data_ptr->getImuSensorDataPtr());
					imu_sensor_data._gyroDeg[0] -= _gyroBias[0]->getBias();
					imu_sensor_data._gyroDeg[1] -= _gyroBias[1]->getBias();
					imu_sensor_data._gyroDeg[2] -= _gyroBias[2]->getBias();
					imu_sensor_data.axisTransformSensorData(_sensorDeviceAttributeMap[last_dr_imu_device_data_ptr->_sensorDevice]._axisTransformToBaseOdo);
					caculateDegByImu(local_time, dr_dt, syn_delta_dr_pose._orient, &imu_sensor_data, last_last_dr_imu_device_data_ptr, syn_dr_pose);
				}
			}
			if ((*iter)->ifIsZeroIdSensor(SENSOR_TYPE::ODO)) {
				boost::shared_ptr<OdoDeviceData> odo_device_ptr = boost::dynamic_pointer_cast<OdoDeviceData>(*iter);
				last_last_dr_odo_device_data_ptr = last_dr_odo_device_data_ptr;
				last_dr_odo_device_data_ptr = odo_device_ptr;
			}
			if ((*iter)->ifIsZeroIdSensor(SENSOR_TYPE::IMU)) {
				boost::shared_ptr<ImuDeviceData> imu_device_ptr = boost::dynamic_pointer_cast<ImuDeviceData>(*iter);
				last_last_dr_imu_device_data_ptr = last_dr_imu_device_data_ptr;
				last_dr_imu_device_data_ptr = imu_device_ptr;
			}

			last_dr_time = base_sensor_data_ptr->_localTime;

			syn_dr_pose_map[sensor_device] = syn_dr_pose;
			syn_dr_local_time_map[sensor_device] = base_sensor_data_ptr->_localTime;
			syn_dr_device_data_map[sensor_device] = (*iter);
		}

		for (std::map<SensorDevice, Pose3D>::iterator iter = syn_dr_pose_map.begin(); iter != syn_dr_pose_map.end(); ++iter) {
			SensorDevice sensor_device = iter->first;
			setLastDeviceSynDrValue(sensor_device, syn_dr_device_data_map[sensor_device], syn_dr_local_time_map[sensor_device], syn_dr_pose_map[sensor_device]);
		}

		return syn_dr_pose;
	}

	void SynSensorData::caculatePosByOdo(double local_time, double dr_dt, Pos3D &delta_dr_pos, OdoSensorData *last_odo_sensor_data, boost::shared_ptr<OdoDeviceData> last_last_dr_odo_device_data_ptr, Pose3D& last_dr_pose) {
		if (local_time - last_odo_sensor_data->_localTime > _confSensorMaxValidDrTime) {
			dr_dt = 0;
		}

		Pos3D delta_odo;
		if (last_odo_sensor_data->_ifUseOdoData) {
			if (last_last_dr_odo_device_data_ptr != NULL) {
				Pose2D delta_pose = OdoSensorData::getInterpolationDeltaPose2D(last_last_dr_odo_device_data_ptr->getOdoSensorDataPtr(), last_odo_sensor_data, dr_dt);
				delta_odo._xyz[0] = delta_pose._pos._x;
				delta_odo._xyz[1] = delta_pose._pos._y;
				delta_odo._xyz[2] = 0;
			}
		}
		else {
			delta_odo._xyz[0] = last_odo_sensor_data->_speedX*dr_dt;
			delta_odo._xyz[1] = last_odo_sensor_data->_speedY*dr_dt;
			delta_odo._xyz[2] = 0;
		}

		delta_dr_pos = delta_odo;
		last_dr_pose._pos = last_dr_pose*delta_dr_pos;
	}

	void SynSensorData::caculateDegByOdo(double local_time, double dr_dt, Orient3D &delta_dr_orient, OdoSensorData *last_odo_sensor_data, boost::shared_ptr<OdoDeviceData> last_last_dr_odo_device_data_ptr, Pose3D& last_dr_pose) {
		if (local_time - last_odo_sensor_data->_localTime > _confSensorMaxValidDrTime) {
			dr_dt = 0;
		}

		double d_odo_deg = 0;
		if (last_odo_sensor_data->_ifUseOdoData) {
			if (last_last_dr_odo_device_data_ptr != NULL) {
				Pose2D delta_pose = OdoSensorData::getInterpolationDeltaPose2D(last_last_dr_odo_device_data_ptr->getOdoSensorDataPtr(), last_odo_sensor_data, dr_dt);
				d_odo_deg = delta_pose._orient._deg;
			}
		}
		else {
			d_odo_deg = last_odo_sensor_data->_angleSpeedDeg*dr_dt;
		}

		delta_dr_orient = Orient3D(d_odo_deg, 0, 0);
		last_dr_pose._orient = last_dr_pose._orient*delta_dr_orient;
	}

	void SynSensorData::caculateDegByImu(double local_time, double dr_dt, Orient3D &delta_dr_orient, ImuSensorData *last_imu_sensor_data, boost::shared_ptr<ImuDeviceData> last_last_dr_imu_device_data_ptr, Pose3D& last_dr_pose) {
		if (local_time - last_imu_sensor_data->_localTime > _confSensorMaxValidDrTime) {
			dr_dt = 0;
		}

		Orient3D d_orient;
		if (last_imu_sensor_data->_ifUseQuat) {
			//to do
		}
		else {
			rot_fuc_deg::d_quat_by_wt(last_imu_sensor_data->_gyroDeg[0], last_imu_sensor_data->_gyroDeg[1], last_imu_sensor_data->_gyroDeg[2],
				dr_dt, d_orient._quat);
			d_orient.uniformByQuat();

		}

		delta_dr_orient = d_orient;
		last_dr_pose._orient = last_dr_pose._orient*delta_dr_orient;
	}

	double SynSensorData::getUsedTimeStamp() {
		return TimeStampAbs::getTimeStamp(_usedTimeStampType);
	}

	double SynSensorData::getDeltaMoveDis(boost::shared_ptr<BaseSensorDeviceData> pre_device_data, boost::shared_ptr<BaseSensorDeviceData> post_device_data) {
		double delta_dis = post_device_data->_synDataListDr._sumMoveDis - pre_device_data->_synDataListDr._sumMoveDis;
		if (delta_dis < -_maxSumMoveDis/2) {
			delta_dis += _maxSumMoveDis;
		}
		return delta_dis;
	}
	double SynSensorData::getDeltaMoveDeg(boost::shared_ptr<BaseSensorDeviceData> pre_device_data, boost::shared_ptr<BaseSensorDeviceData> post_device_data) {
		double delta_deg = post_device_data->_synDataListDr._sumMoveDeg - pre_device_data->_synDataListDr._sumMoveDeg;
		if (delta_deg < -_maxSumMoveDeg/2) {
			delta_deg += _maxSumMoveDeg;
		}
		return delta_deg;
	}

}