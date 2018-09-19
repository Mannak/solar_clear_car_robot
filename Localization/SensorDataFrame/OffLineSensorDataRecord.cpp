#include "OffLineSensorDataRecord.h"
#include <vector>
#include "ComUtils\Comm.h"
#include "ComUtils\NormalSleep.h"
#include <boost\thread.hpp>
#include "SynSensorData.h"
#include <iostream>

#ifdef _USE_OPENCV_
#include <opencv2\highgui\highgui.hpp>
#endif


namespace Localization{

	void OffLineSensorDataRecord::setOnRecordData(std::string record_file_dir) {
		::system(("md " + record_file_dir).c_str());
		record_file_dir = record_file_dir + "\\";
		::system((std::string("del ") + record_file_dir + std::string("* /q")).c_str());
		_recordFileDir = record_file_dir;
		_recordSensorFile.open((record_file_dir + "sensor.txt").c_str(), std::ios::out);
		_odoRecordFile.open((record_file_dir + "sensor_odo.txt").c_str(), std::ios::out);
		_imuRecordFile.open((record_file_dir + "sensor_imu.txt").c_str(), std::ios::out);

		boost::thread t(&OffLineSensorDataRecord::writeBigDataLoop, this);

		_ifStartRecord = true;
		_ifEndRecord = false;

		std::cout << "OffLineSensorDataRecord: setOnRecordData" << std::endl;
	}

	void OffLineSensorDataRecord::endRecordData() {
		boost::mutex::scoped_lock l(_mutRecordFile);
		_ifEndRecord = true;
		_recordSensorFile.close();
		_odoRecordFile.clear();
		_imuRecordFile.clear();
		std::cout << "OffLineSensorDataRecord: endRecordData" << std::endl;
	}

	void OffLineSensorDataRecord::setOnReadOfflineData(std::string read_file_dir, int sleep_line_intvel) {
		_readFileDir = read_file_dir + "\\";
		_sleepLineIntvel = sleep_line_intvel;

		boost::thread t(&OffLineSensorDataRecord::readOfflineDataLoop, this);
		_endReadOfflineData = false;
		std::cout << "OffLineSensorDataRecord: setOnReadOfflineData" << std::endl;
	}

	void OffLineSensorDataRecord::endReadOfflineData() {
		_endReadOfflineData = true;

		std::cout << "OffLineSensorDataRecord: endReadOfflineData" << std::endl;
	}

	void OffLineSensorDataRecord::callByBaseDeviceDataPtr(boost::shared_ptr<BaseSensorDeviceData> base_device_data_ptr, std::string image_suffix) {
		boost::mutex::scoped_lock l(_mutRecordFile);

		if (_ifStartRecord == false || _ifEndRecord == true) {
			return;
		}

		SensorDevice sensor_device = base_device_data_ptr->_sensorDevice;
		SENSOR_TYPE sensor_type = sensor_device._sensorType;

		if (sensor_type == SENSOR_TYPE::ODO) {
			boost::shared_ptr<OdoDeviceData> odo_device_ptr = boost::dynamic_pointer_cast<OdoDeviceData>(base_device_data_ptr);
			callByOdoDeviceDataPtr(odo_device_ptr);
		}
		else if (sensor_type == SENSOR_TYPE::IMU) {
			boost::shared_ptr<ImuDeviceData> imu_device_ptr = boost::dynamic_pointer_cast<ImuDeviceData>(base_device_data_ptr);
			callByImuDeviceDataPtr(imu_device_ptr);
		}
		else if (sensor_type == SENSOR_TYPE::LASER2D) {
			boost::shared_ptr<Laser2dDeviceData> laser2d_device_ptr = boost::dynamic_pointer_cast<Laser2dDeviceData>(base_device_data_ptr);
			callByLaser2dDeviceDataPtr(laser2d_device_ptr);
		}
		else if (sensor_type == SENSOR_TYPE::LASER3D) {
			boost::shared_ptr<Laser3dDeviceData> laser3d_device_ptr = boost::dynamic_pointer_cast<Laser3dDeviceData>(base_device_data_ptr);
			callByLaser3dDeviceDataPtr(laser3d_device_ptr);
		}
		else if (sensor_type == SENSOR_TYPE::CAMERA) {
			boost::shared_ptr<CameraDeviceData> camera_device_ptr = boost::dynamic_pointer_cast<CameraDeviceData>(base_device_data_ptr);
			callByCameraDeviceDataPtr(camera_device_ptr, image_suffix);
		}
	}

	void OffLineSensorDataRecord::insertLineStrToRecordFile(std::string str) {
		boost::mutex::scoped_lock l(_mutRecordFile);

		if (_ifStartRecord == false || _ifEndRecord == true) {
			return;
		}

		_recordSensorFile << "inserted_str " << str << std::endl;
	}

	void OffLineSensorDataRecord::callByOdoDeviceDataPtr(boost::shared_ptr<OdoDeviceData> odo_device_data_ptr) {
		SensorDevice sensor_device = odo_device_data_ptr->_sensorDevice;
		OdoSensorData* odo_sensor_data_ptr = odo_device_data_ptr->getOdoSensorDataPtr();
		std::vector<double>& wheel_speed_vec = odo_sensor_data_ptr->_wheelSpeedVec;
		std::vector<double>& wheel_angle_deg_vec = odo_sensor_data_ptr->_wheelAngleDegVec;

		_recordSensorFile << std::setprecision(10) << odo_sensor_data_ptr->getNameStr() << " " << sensor_device._deviceId << " "
			<< (odo_sensor_data_ptr->_state ? "true" : "false") << " " << odo_sensor_data_ptr->_localTime << " " << odo_sensor_data_ptr->_intvalTime << " "
			<< (odo_sensor_data_ptr->_ifUseOdoData ? "true" : "false") << " " << std::setprecision(6)
			<< odo_sensor_data_ptr->_odoX << " " << odo_sensor_data_ptr->_odoY << " " << odo_sensor_data_ptr->_odoAngleDeg << " "
			<< odo_sensor_data_ptr->_speedX << " " << odo_sensor_data_ptr->_speedY << " " << odo_sensor_data_ptr->_angleSpeedDeg;
		for (int i = 0; i < wheel_speed_vec.size(); ++i) {
			_recordSensorFile << " " << wheel_speed_vec[i] << " " << wheel_angle_deg_vec[i];
		}
		_recordSensorFile << std::endl;

		_odoRecordFile << std::setprecision(10) << odo_sensor_data_ptr->getNameStr() << " " << sensor_device._deviceId << " "
			<< (odo_sensor_data_ptr->_state ? "true" : "false") << " " << odo_sensor_data_ptr->_localTime << " " << odo_sensor_data_ptr->_intvalTime << " "
			<< (odo_sensor_data_ptr->_ifUseOdoData ? "true" : "false") << " " << std::setprecision(6)
			<< odo_sensor_data_ptr->_odoX << " " << odo_sensor_data_ptr->_odoY << " " << odo_sensor_data_ptr->_odoAngleDeg << " "
			<< odo_sensor_data_ptr->_speedX << " " << odo_sensor_data_ptr->_speedY << " " << odo_sensor_data_ptr->_angleSpeedDeg;
		for (int i = 0; i < wheel_speed_vec.size(); ++i) {
			_odoRecordFile << " " << wheel_speed_vec[i] << " " << wheel_angle_deg_vec[i];
		}
		_odoRecordFile << std::endl;
	}

	void OffLineSensorDataRecord::callByImuDeviceDataPtr(boost::shared_ptr<ImuDeviceData> imu_device_data_ptr) {
		SensorDevice sensor_device = imu_device_data_ptr->_sensorDevice;
		ImuSensorData* imu_sensor_data_ptr = imu_device_data_ptr->getImuSensorDataPtr();

		_recordSensorFile << std::setprecision(10) << imu_sensor_data_ptr->getNameStr() << " " << sensor_device._deviceId << " "
			<< (imu_sensor_data_ptr->_state ? "true" : "false") << " " << imu_sensor_data_ptr->_localTime << " " << imu_sensor_data_ptr->_intvalTime << " "
			<< (imu_sensor_data_ptr->_ifUseQuat ? "true" : "false") << " " << std::setprecision(6)
			<< imu_sensor_data_ptr->_gyroDeg[0] << " " << imu_sensor_data_ptr->_gyroDeg[1] << " " << imu_sensor_data_ptr->_gyroDeg[2] << " "
			<< imu_sensor_data_ptr->_acc[0] << " " << imu_sensor_data_ptr->_acc[1] << " " << imu_sensor_data_ptr->_acc[2] << " "
			<< imu_sensor_data_ptr->_quat[0] << " " << imu_sensor_data_ptr->_quat[1] << " " << imu_sensor_data_ptr->_quat[2] << " " << imu_sensor_data_ptr->_quat[3] << std::endl;
	
		_imuRecordFile << std::setprecision(10) << imu_sensor_data_ptr->getNameStr() << " " << sensor_device._deviceId << " "
			<< (imu_sensor_data_ptr->_state ? "true" : "false") << " " << imu_sensor_data_ptr->_localTime << " " << imu_sensor_data_ptr->_intvalTime << " "
			<< (imu_sensor_data_ptr->_ifUseQuat ? "true" : "false") << " " << std::setprecision(6)
			<< imu_sensor_data_ptr->_gyroDeg[0] << " " << imu_sensor_data_ptr->_gyroDeg[1] << " " << imu_sensor_data_ptr->_gyroDeg[2] << " "
			<< imu_sensor_data_ptr->_acc[0] << " " << imu_sensor_data_ptr->_acc[1] << " " << imu_sensor_data_ptr->_acc[2] << " "
			<< imu_sensor_data_ptr->_quat[0] << " " << imu_sensor_data_ptr->_quat[1] << " " << imu_sensor_data_ptr->_quat[2] << " " << imu_sensor_data_ptr->_quat[3] << std::endl;
	}

	void OffLineSensorDataRecord::callByLaser2dDeviceDataPtr(boost::shared_ptr<Laser2dDeviceData> laser_2d_device_data_ptr) {
		boost::mutex::scoped_lock l(_mutLaser2dBufferMap);

		SensorDevice sensor_device = laser_2d_device_data_ptr->_sensorDevice;
		Laser2dSensorData* laser2d_sensor_data_ptr = laser_2d_device_data_ptr->getLaser2dSensorDataPtr();

		if (_laser2dBufferMap.find(sensor_device) == _laser2dBufferMap.end()) {
			_laser2dBufferMap[sensor_device] = RecordBuffer<Laser2dDeviceData>();
		}

		_recordSensorFile << std::setprecision(10) << laser2d_sensor_data_ptr->getNameStr() << " " << sensor_device._deviceId << " "
			<< (laser2d_sensor_data_ptr->_state ? "true" : "false") << " " << laser2d_sensor_data_ptr->_localTime << " " << laser2d_sensor_data_ptr->_intvalTime << " "
			<< _laser2dBufferMap[sensor_device]._dataIndex << " " << laser2d_sensor_data_ptr->_beamVec.size() << std::endl;

		_laser2dBufferMap[sensor_device]._dataIndexList.push_back(_laser2dBufferMap[sensor_device]._dataIndex);
		_laser2dBufferMap[sensor_device]._deviceDataPtrList.push_back(laser_2d_device_data_ptr);
		_laser2dBufferMap[sensor_device]._dataIndex++;
	}

	void OffLineSensorDataRecord::callByLaser3dDeviceDataPtr(boost::shared_ptr<Laser3dDeviceData> laser_3d_device_data_ptr) {
		boost::mutex::scoped_lock l(_mutLaser3dBufferMap);

		SensorDevice sensor_device = laser_3d_device_data_ptr->_sensorDevice;
		Laser3dSensorData* laser3d_sensor_data_ptr = laser_3d_device_data_ptr->getLaser3dSensorDataPtr();

		if (_laser3dBufferMap.find(sensor_device) == _laser3dBufferMap.end()) {
			_laser3dBufferMap[sensor_device] = RecordBuffer<Laser3dDeviceData>();
		}

		int seg_beam_size = 0;
		if (laser3d_sensor_data_ptr->_segmentVec.size() > 0) {
			seg_beam_size = laser3d_sensor_data_ptr->_segmentVec[0]._beamVec.size();
		}
		_recordSensorFile << std::setprecision(10) << laser3d_sensor_data_ptr->getNameStr() << " " << sensor_device._deviceId << " "
			<< (laser3d_sensor_data_ptr->_state ? "true" : "false") << " " << laser3d_sensor_data_ptr->_localTime << " " << laser3d_sensor_data_ptr->_intvalTime << " "
			<< _laser3dBufferMap[sensor_device]._dataIndex << " "
			<< laser3d_sensor_data_ptr->_segmentVec.size() << " " << seg_beam_size << std::endl;

		_laser3dBufferMap[sensor_device]._dataIndexList.push_back(_laser3dBufferMap[sensor_device]._dataIndex);
		_laser3dBufferMap[sensor_device]._deviceDataPtrList.push_back(laser_3d_device_data_ptr);
		_laser3dBufferMap[sensor_device]._dataIndex++;
	}

	void OffLineSensorDataRecord::callByCameraDeviceDataPtr(boost::shared_ptr<CameraDeviceData> camera_device_data_ptr, std::string image_suffix) {
		boost::mutex::scoped_lock l(_mutCameraBufferMap);

		SensorDevice sensor_device = camera_device_data_ptr->_sensorDevice;
		CameraSensorData* camera_sensor_data_ptr = camera_device_data_ptr->getCameraSensorDataPtr();

		if (_cameraBufferMap.find(sensor_device) == _cameraBufferMap.end()) {
			_cameraBufferMap[sensor_device] = RecordBuffer<CameraDeviceData>();
		}

		_recordSensorFile << std::setprecision(10) << camera_sensor_data_ptr->getNameStr() << " " << sensor_device._deviceId << " "
			<< (camera_sensor_data_ptr->_state ? "true" : "false") << " " << camera_sensor_data_ptr->_localTime << " " << camera_sensor_data_ptr->_intvalTime << " "
			<< _cameraBufferMap[sensor_device]._dataIndex << " " << image_suffix << std::endl;

		_cameraBufferMap[sensor_device]._dataIndexList.push_back(_cameraBufferMap[sensor_device]._dataIndex);
		_cameraBufferMap[sensor_device]._deviceDataPtrList.push_back(camera_device_data_ptr);
		_cameraBufferMap[sensor_device]._dataIndex++;

		_cameraBufferMap[sensor_device]._infoStrList.push_back(image_suffix);
	}

	void OffLineSensorDataRecord::writeBigDataLoop() {
		std::cout << "OffLineSensorDataRecord: " << "write big data loop" << std::endl;

		while (1) {
			SLEEP(3);

			if (_ifEndRecord == true) {
				break;
			}

			{//laser2d
				std::vector<int> laser2_index_vec;
				std::vector<boost::shared_ptr<Laser2dDeviceData> > laser2_vec;
				{
					boost::mutex::scoped_lock l(_mutLaser2dBufferMap);
					for (std::map<SensorDevice, RecordBuffer<Laser2dDeviceData> >::iterator iter = _laser2dBufferMap.begin(); iter != _laser2dBufferMap.end(); ++iter) {
						if (iter->second._dataIndexList.size() > 0) {
							laser2_index_vec.push_back(iter->second._dataIndexList.front());
							laser2_vec.push_back(iter->second._deviceDataPtrList.front());

							double local_time = (iter->second._deviceDataPtrList.front())->getSensorDataPtr()->_localTime;
							static double last_t = local_time;
							if (local_time - last_t > 3) {
								std::cout << "OffLineSensorDataRecord: " << "write laser2d " << iter->second._dataIndexList.front() << ", list size " << iter->second._deviceDataPtrList.size() << std::endl;
								last_t = local_time;
							}

							iter->second._dataIndexList.pop_front();
							iter->second._deviceDataPtrList.pop_front();
						}
					}
				}

				for (int i = 0; i < laser2_index_vec.size(); ++i) {
					int index = laser2_index_vec[i];
					boost::shared_ptr<Laser2dDeviceData> laser2 = laser2_vec[i];
					SensorDevice sensor_device = laser2->_sensorDevice;
					Laser2dSensorData* laser2d_sensor_data_ptr = laser2->getLaser2dSensorDataPtr();

					std::ostringstream ss;
					ss << laser2d_sensor_data_ptr->getNameStr() << "_" << sensor_device._deviceId << "_" << index << ".txt";
					std::string str = ss.str();
					std::ofstream file((_recordFileDir + str).c_str());

					for (int j = 0; j < laser2d_sensor_data_ptr->_beamVec.size(); ++j) {
						file << std::setprecision(6) << laser2d_sensor_data_ptr->_beamVec[j]._polar._deg << " " << laser2d_sensor_data_ptr->_beamVec[j]._polar._radius << " " << laser2d_sensor_data_ptr->_beamVec[j]._rssi << " " << (laser2d_sensor_data_ptr->_beamVec[j]._validity ? 0 : 1) << std::endl;
					}

					file.close();
				}
			}

			{//laser3d
				std::vector<int> laser3_index_vec;
				std::vector<boost::shared_ptr<Laser3dDeviceData> > laser3_vec;
				{
					boost::mutex::scoped_lock l(_mutLaser3dBufferMap);
					for (std::map<SensorDevice, RecordBuffer<Laser3dDeviceData> >::iterator iter = _laser3dBufferMap.begin(); iter != _laser3dBufferMap.end(); ++iter) {
						if (iter->second._dataIndexList.size() > 0) {
							laser3_index_vec.push_back(iter->second._dataIndexList.front());
							laser3_vec.push_back(iter->second._deviceDataPtrList.front());

							double local_time = (iter->second._deviceDataPtrList.front())->getSensorDataPtr()->_localTime;
							static double last_t = local_time;
							if (local_time - last_t > 3) {
								std::cout << "OffLineSensorDataRecord: " << "write laser3d " << iter->second._dataIndexList.front() << " , list size " << iter->second._deviceDataPtrList.size() << std::endl;
								last_t = local_time;
							}

							iter->second._dataIndexList.pop_front();
							iter->second._deviceDataPtrList.pop_front();
						}
					}
				}

				for (int i = 0; i < laser3_index_vec.size(); ++i) {
					int index = laser3_index_vec[i];
					boost::shared_ptr<Laser3dDeviceData> laser3 = laser3_vec[i];
					SensorDevice sensor_device = laser3->_sensorDevice;
					Laser3dSensorData* laser3d_sensor_data_ptr = laser3->getLaser3dSensorDataPtr();

					std::ostringstream ss;
					ss << laser3d_sensor_data_ptr->getNameStr() << "_" << sensor_device._deviceId << "_" << index << ".txt";
					std::string str = ss.str();
					std::ofstream file((_recordFileDir + str).c_str());

					for (int j = 0; j < laser3d_sensor_data_ptr->_segmentVec.size(); ++j) {
						file << std::setprecision(10) << laser3d_sensor_data_ptr->_segmentVec[j]._localTime << std::endl << std::setprecision(6);
						for (int k = 0; k < laser3d_sensor_data_ptr->_segmentVec[j]._beamVec.size(); ++k) {
							file << std::setprecision(6) << laser3d_sensor_data_ptr->_segmentVec[j]._beamVec[k]._pos._xyz[0] << " " << laser3d_sensor_data_ptr->_segmentVec[j]._beamVec[k]._pos._xyz[1] << " "
								<< laser3d_sensor_data_ptr->_segmentVec[j]._beamVec[k]._pos._xyz[2] << " " << laser3d_sensor_data_ptr->_segmentVec[j]._beamVec[k]._rssi << std::endl;
						}
					}

					file.close();
				}
			}

			{//camera
				std::vector<int> camera_index_vec;
				std::vector<boost::shared_ptr<CameraDeviceData> > camera_vec;
				std::vector<std::string> info_str_vec;
				{
					boost::mutex::scoped_lock l(_mutCameraBufferMap);
					for (std::map<SensorDevice, RecordBuffer<CameraDeviceData> >::iterator iter = _cameraBufferMap.begin(); iter != _cameraBufferMap.end(); ++iter) {
						if (iter->second._dataIndexList.size() > 0) {
							camera_index_vec.push_back(iter->second._dataIndexList.front());
							camera_vec.push_back(iter->second._deviceDataPtrList.front());
							info_str_vec.push_back(iter->second._infoStrList.front());

							double local_time = (iter->second._deviceDataPtrList.front())->getSensorDataPtr()->_localTime;
							static double last_t = local_time;
							if (local_time - last_t > 3) {
								std::cout << "OffLineSensorDataRecord: " << "write camera " << iter->second._dataIndexList.front() << ", list size " << iter->second._deviceDataPtrList.size() << std::endl;
								last_t = local_time;
							}

							iter->second._dataIndexList.pop_front();
							iter->second._deviceDataPtrList.pop_front();
							iter->second._infoStrList.pop_front();
						}
					}
				}

				for (int i = 0; i < camera_index_vec.size(); ++i) {
					int index = camera_index_vec[i];
					boost::shared_ptr<CameraDeviceData> camera = camera_vec[i];
					SensorDevice sensor_device = camera->_sensorDevice;
					CameraSensorData* camera_sensor_data_ptr = camera->getCameraSensorDataPtr();

					std::ostringstream ss;
					ss << camera_sensor_data_ptr->getNameStr() << "_" << sensor_device._deviceId << "_" << index << info_str_vec[i];
					std::string str = ss.str();

#ifdef _USE_OPENCV_
					cv::imwrite(_recordFileDir + str, camera_sensor_data_ptr->_image);
#endif
				}
			}
		}

		std::cout << "OffLineSensorDataRecord: " << "end write big data loop!" << std::endl;
	}

	void OffLineSensorDataRecord::readOfflineDataLoop() {
		//calib txt
		std::ifstream calib_file;
		calib_file.open((_readFileDir + "calib.txt").c_str(), std::ios::in);
		if (calib_file.is_open()) {
			std::cout << "OffLineSensorDataRecord: " << "calib_file open success!" << std::endl;

			std::string c_line_str;
			while (getline(calib_file, c_line_str)) {
				if (c_line_str.empty()) {
					continue;
				}

				std::vector<std::string> c_str_vec;
				cComm::SplitString(c_line_str, " ", c_str_vec);

				SENSOR_TYPE sensor_type = BaseSensorData::getSensorTypeByNameStr(c_str_vec[0]);
				int device_id = 0;
				if (c_str_vec.size() > 1) {
					device_id = atoi(c_str_vec[1].c_str());
				}
				SensorDevice sensor_device(sensor_type, device_id);

				Pose3D pose;
				if (c_str_vec.size() >= 8) {
					pose = Pose3D(atof(c_str_vec[2].c_str()), atof(c_str_vec[3].c_str()), atof(c_str_vec[4].c_str()),
						atof(c_str_vec[5].c_str()), atof(c_str_vec[6].c_str()), atof(c_str_vec[7].c_str()));
				}

				_sensorCalibDataMap[sensor_device] = pose;
			}
			calib_file.close();
		}
		std::cout << "OffLineSensorDataRecord: " << "read sensor calib data num = " << _sensorCalibDataMap.size() << std::endl;


		//sensor txt--------------------------
		std::ifstream sensor_file;
		sensor_file.open((_readFileDir + "sensor.txt").c_str(), std::ios::in);
		if (sensor_file.is_open() == false) {
			std::cout << "OffLineSensorDataRecord: " << "sensor_file open fail!" << std::endl;
			return;
		}

		std::cout << "OffLineSensorDataRecord: " << "sensor_file open success!" << std::endl;

		std::string line_str;
		bool if_by_end_read_order = false;
		while (getline(sensor_file, line_str)) {
			if (_endReadOfflineData == true) {
				if_by_end_read_order = true;
				break;
			}

			SLEEP(_sleepLineIntvel);
			if (line_str.empty()) {
				continue;
			}

			std::vector<std::string> str_vec;
			cComm::SplitString(line_str, " ", str_vec);

			if (str_vec[0] == BaseSensorData::getNameStrBySensorType(SENSOR_TYPE::ODO)) {
				int device_id = atoi(str_vec[1].c_str());
				bool state = str_vec[2] == "true";
				double local_time = atof(str_vec[3].c_str());
				double intvel_time = atof(str_vec[4].c_str());

				bool if_use_odo_data = str_vec[5] == "true";
				double odo_x = atof(str_vec[6].c_str()), odo_y = atof(str_vec[7].c_str()), odo_angle_deg = atof(str_vec[8].c_str());
				double speed_x = atof(str_vec[9].c_str()), speed_y = atof(str_vec[10].c_str()), angle_speed_deg = atof(str_vec[11].c_str());
				std::vector<double> speed_vec;
				std::vector<double> wheel_angle_deg_vec;
				for (int i = 12; i < str_vec.size(); i += 2) {
					speed_vec.push_back(atof(str_vec[i].c_str()));
					if ((i + 1) < str_vec.size()) {
						wheel_angle_deg_vec.push_back(atof(str_vec[i + 1].c_str()));
					}
				}

				boost::shared_ptr<OdoDeviceData> odo_device_data_ptr(new OdoDeviceData(if_use_odo_data, local_time, intvel_time, state, device_id));
				if (odo_device_data_ptr == NULL) {
					std::cout << "readOfflineDataLoop: " << "odo_device_data_ptr == NULL!!!" << std::endl;
					continue;
				}
				odo_device_data_ptr->getOdoSensorDataPtr()->setOdoData(odo_x, odo_y, odo_angle_deg);
				odo_device_data_ptr->getOdoSensorDataPtr()->setSpeedData(speed_x, speed_y, angle_speed_deg);
				odo_device_data_ptr->getOdoSensorDataPtr()->setWheelSpeedVec(speed_vec, wheel_angle_deg_vec);

				if (_ifBindCallBackOfflineBaseDeviceDataOutput == true) {
					_callBackOfflineBaseDeviceDataOutput(odo_device_data_ptr);
				}
			}
			else if (str_vec[0] == BaseSensorData::getNameStrBySensorType(SENSOR_TYPE::IMU)) {
				int device_id = atoi(str_vec[1].c_str());
				bool state = str_vec[2] == "true";
				double local_time = atof(str_vec[3].c_str());
				double intvel_time = atof(str_vec[4].c_str());

				bool if_use_quat = str_vec[5] == "true";

				boost::shared_ptr<ImuDeviceData> imu_device_data_ptr(new ImuDeviceData(if_use_quat, local_time, intvel_time, state, device_id));
				if (imu_device_data_ptr == NULL) {
					std::cout << "readOfflineDataLoop: " << "imu_device_data_ptr == NULL!!!" << std::endl;
					continue;
				}
				imu_device_data_ptr->getImuSensorDataPtr()->_gyroDeg[0] = atof(str_vec[6].c_str());
				imu_device_data_ptr->getImuSensorDataPtr()->_gyroDeg[1] = atof(str_vec[7].c_str());
				imu_device_data_ptr->getImuSensorDataPtr()->_gyroDeg[2] = atof(str_vec[8].c_str());
				imu_device_data_ptr->getImuSensorDataPtr()->_acc[0] = atof(str_vec[9].c_str());
				imu_device_data_ptr->getImuSensorDataPtr()->_acc[1] = atof(str_vec[10].c_str());
				imu_device_data_ptr->getImuSensorDataPtr()->_acc[2] = atof(str_vec[11].c_str());
				imu_device_data_ptr->getImuSensorDataPtr()->_quat[0] = atof(str_vec[12].c_str());
				imu_device_data_ptr->getImuSensorDataPtr()->_quat[1] = atof(str_vec[13].c_str());
				imu_device_data_ptr->getImuSensorDataPtr()->_quat[2] = atof(str_vec[14].c_str());
				imu_device_data_ptr->getImuSensorDataPtr()->_quat[3] = atof(str_vec[15].c_str());

				if (_ifBindCallBackOfflineBaseDeviceDataOutput == true) {
					_callBackOfflineBaseDeviceDataOutput(imu_device_data_ptr);
				}
			}
			else if (str_vec[0] == BaseSensorData::getNameStrBySensorType(SENSOR_TYPE::LASER2D)) {
				int device_id = atoi(str_vec[1].c_str());
				bool state = str_vec[2] == "true";
				double local_time = atof(str_vec[3].c_str());
				double intvel_time = atof(str_vec[4].c_str());
				int data_index = atoi(str_vec[5].c_str());
				int beam_size = atoi(str_vec[6].c_str());

				std::ostringstream ss;
				ss << str_vec[0] << "_" << device_id << "_" << data_index << ".txt";
				std::string str = ss.str();
				std::ifstream file((_readFileDir + str).c_str());
				if (file.is_open() == false) {
					continue;
				}

				boost::shared_ptr<Laser2dDeviceData> laser2d_device_data_ptr(new Laser2dDeviceData(beam_size, local_time, intvel_time, state, device_id));
				if (laser2d_device_data_ptr == NULL) {
					std::cout << "readOfflineDataLoop: " << "laser2d_device_data_ptr == NULL!!!" << std::endl;
					file.close();
					continue;
				}
				Laser2dSensorData *laser2d_sensor_data_ptr = laser2d_device_data_ptr->getLaser2dSensorDataPtr();

				std::string laser_line;
				int beam_index = 0;
				while (getline(file, laser_line)) {
					if (laser_line.empty()) {
						continue;
					}

					std::vector<std::string> laser_str_vec;
					cComm::SplitString(laser_line, " ", laser_str_vec);
					if (laser_str_vec.size() == 4) {
						double deg = atof(laser_str_vec[0].c_str());
						double dis = atof(laser_str_vec[1].c_str());
						int rssi = atoi(laser_str_vec[2].c_str());
						double valid = atoi(laser_str_vec[3].c_str()) == 0;

						if (beam_index < beam_size) {
							laser2d_sensor_data_ptr->_beamVec[beam_index++] = Laser2dSensorData::Beam(deg, dis, rssi, valid);
						}
					}
				}

				if (_ifBindCallBackOfflineBaseDeviceDataOutput == true) {
					_callBackOfflineBaseDeviceDataOutput(laser2d_device_data_ptr);
				}

				file.close();
			}
			else if (str_vec[0] == BaseSensorData::getNameStrBySensorType(SENSOR_TYPE::LASER3D)) {
				int device_id = atoi(str_vec[1].c_str());
				bool state = str_vec[2] == "true";
				double local_time = atof(str_vec[3].c_str());
				double intvel_time = atof(str_vec[4].c_str());
				int data_index = atoi(str_vec[5].c_str());
				int segment_size = atoi(str_vec[6].c_str());
				int seg_beam_size = atoi(str_vec[7].c_str());

				std::ostringstream ss;
				ss << str_vec[0] << "_" << device_id << "_" << data_index << ".txt";
				std::string str = ss.str();
				std::ifstream file((_readFileDir + str).c_str());
				if (file.is_open() == false) {
					continue;
				}

				boost::shared_ptr<Laser3dDeviceData> laser3d_device_data_ptr(new Laser3dDeviceData(segment_size, seg_beam_size, local_time, intvel_time, state, device_id));
				if (laser3d_device_data_ptr == NULL) {
					std::cout << "readOfflineDataLoop: " << "laser3d_device_data_ptr == NULL!!!" << std::endl;
					file.close();
					continue;
				}
				Laser3dSensorData *laser3d_sensor_data_ptr = laser3d_device_data_ptr->getLaser3dSensorDataPtr();
				int frame_point_count = 0;
				int segment_frame_count = 0;

				std::string laser_line;
				while (getline(file, laser_line)) {
					if (laser_line.empty()) {
						continue;
					}

					std::vector<std::string> laser_str_vec;
					cComm::SplitString(laser_line, " ", laser_str_vec);
					if (laser_str_vec.size() == 1) {
						double frame_time_sample = atof(laser_str_vec[0].c_str());
						laser3d_sensor_data_ptr->_segmentVec[segment_frame_count]._localTime = frame_time_sample;
						frame_point_count = 0;
					}
					else if (laser_str_vec.size() == 4) {
						laser3d_sensor_data_ptr->_segmentVec[segment_frame_count]._beamVec[frame_point_count]._pos._xyz[0] = atof(laser_str_vec[0].c_str());
						laser3d_sensor_data_ptr->_segmentVec[segment_frame_count]._beamVec[frame_point_count]._pos._xyz[1] = atof(laser_str_vec[1].c_str());
						laser3d_sensor_data_ptr->_segmentVec[segment_frame_count]._beamVec[frame_point_count]._pos._xyz[2] = atof(laser_str_vec[2].c_str());
						laser3d_sensor_data_ptr->_segmentVec[segment_frame_count]._beamVec[frame_point_count]._rssi = atoi(laser_str_vec[3].c_str());

						frame_point_count++;
						if (frame_point_count == seg_beam_size) {
							segment_frame_count++;
						}
					}
				}

				if (_ifBindCallBackOfflineBaseDeviceDataOutput == true) {
					_callBackOfflineBaseDeviceDataOutput(laser3d_device_data_ptr);
				}

				file.close();
			}
			else if (str_vec[0] == "inserted_str") {
				std::string result_str;
				for (int i = 1; i < str_vec.size(); ++i) {
					if (i != str_vec.size() - 1) {
						result_str = result_str + str_vec[i] + " ";
					}
					else {
						result_str = result_str + str_vec[i];
					}
				}

				if (_ifBindCallBackOfflineInsertedLineStrOutput == true) {
					_callBackOfflineInsertedLineStrOutput(result_str);
				}
			}
			else if (str_vec[0] == BaseSensorData::getNameStrBySensorType(SENSOR_TYPE::CAMERA)) {
				int device_id = atoi(str_vec[1].c_str());
				bool state = str_vec[2] == "true";
				double local_time = atof(str_vec[3].c_str());
				double intvel_time = atof(str_vec[4].c_str());
				int data_index = atoi(str_vec[5].c_str());
				std::string image_suffix = str_vec[6];

				std::ostringstream ss;
				ss << str_vec[0] << "_" << device_id << "_" << data_index << image_suffix;
				std::string str = ss.str();

#ifdef _USE_OPENCV_
				cv::Mat image = cv::imread(_readFileDir + str);
				if (image.empty()) {
					continue;
				}

				boost::shared_ptr<CameraDeviceData> camera_device_data_ptr(new CameraDeviceData(image, local_time, intvel_time, state, device_id));
				if (camera_device_data_ptr == NULL) {
					std::cout << "readOfflineDataLoop: " << "camera_device_data_ptr == NULL!!!" << std::endl;
					continue;
				}

				if (_ifBindCallBackOfflineBaseDeviceDataOutput == true) {
					_callBackOfflineBaseDeviceDataOutput(camera_device_data_ptr);
				}
#endif
			}
		}

		sensor_file.close();
		std::cout << "OffLineSensorDataRecord: " << "sensor data read finish!" << std::endl;

		if (_ifBindCallBackOfflineFinishRead == true) {
			_callBackOfflineFinishRead(if_by_end_read_order);
		}
	}

	void OffLineSensorDataRecord::bindCallBackOffLineBaseDeviceDataOutputToSynSensorData() {
		bindCallBackOfflineBaseDeviceDataOutput(boost::bind(&SynSensorData::insertSensorData, SynSensorData::instance(), _1));
	}

}