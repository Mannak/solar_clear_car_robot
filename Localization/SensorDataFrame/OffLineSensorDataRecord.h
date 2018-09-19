/************************************************************************
Author: huanghong
Date: 2016-10-31                                                   
************************************************************************/

#ifndef _OFF_LINE_SENSOR_DATA_RECORD_H_
#define _OFF_LINE_SENSOR_DATA_RECORD_H_


#include <string>
#include <fstream>
#include <list>
#include <map>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include "Localization\SensorDataFrame\SensorDeviceData.h"


namespace Localization{

	template<class T>
	class RecordBuffer {
	public:
		RecordBuffer() {
			_dataIndex = 0;
		}

		int _dataIndex;
		std::list<int> _dataIndexList;
		std::list<boost::shared_ptr<T> > _deviceDataPtrList;
		std::list<std::string> _infoStrList;
	};

	class OffLineSensorDataRecord {
	public:
		static OffLineSensorDataRecord *instance() {
			static OffLineSensorDataRecord ins;
			return &ins;
		}

	private:
		OffLineSensorDataRecord() {
			_ifStartRecord = false;
			_ifEndRecord = false;

			_endReadOfflineData = false;

			_ifBindCallBackOfflineBaseDeviceDataOutput = false;
			_ifBindCallBackOfflineInsertedLineStrOutput = false;
		}

	public:
		void setOnRecordData(std::string record_file_dir);
		void endRecordData();
		void callByBaseDeviceDataPtr(boost::shared_ptr<BaseSensorDeviceData> base_device_data_ptr, std::string image_suffix = ".jpg");
		void insertLineStrToRecordFile(std::string str);

		void setOnReadOfflineData(std::string read_file_dir, int sleep_line_intvel);
		void endReadOfflineData();
		void bindCallBackOfflineBaseDeviceDataOutput(boost::function<void(boost::shared_ptr<BaseSensorDeviceData>)> call_back_offline_base_device_data_output) {
			_callBackOfflineBaseDeviceDataOutput = call_back_offline_base_device_data_output;
			_ifBindCallBackOfflineBaseDeviceDataOutput = true;
		}
		void bindCallBackOfflineInsertedLineStrOutput(boost::function<void(std::string)> call_back_offline_inserted_line_str_output) {
			_callBackOfflineInsertedLineStrOutput = call_back_offline_inserted_line_str_output;
			_ifBindCallBackOfflineInsertedLineStrOutput = true;
		}
		void bindCallBackOffLineBaseDeviceDataOutputToSynSensorData();
		void bindCallBackOfflineFinishRead(boost::function<void(bool)> call_back_offline_finish_read) {
			_callBackOfflineFinishRead = call_back_offline_finish_read;
			_ifBindCallBackOfflineFinishRead = true;
		}

		std::map<SensorDevice, Pose3D> _sensorCalibDataMap;

	private:
		bool _ifBindCallBackOfflineBaseDeviceDataOutput;
		bool _ifBindCallBackOfflineInsertedLineStrOutput;
		bool _ifBindCallBackOfflineFinishRead;
		boost::function<void(boost::shared_ptr<BaseSensorDeviceData>)> _callBackOfflineBaseDeviceDataOutput;
		boost::function<void(std::string)> _callBackOfflineInsertedLineStrOutput;
		boost::function<void(bool)> _callBackOfflineFinishRead;

	private:
		void callByOdoDeviceDataPtr(boost::shared_ptr<OdoDeviceData> odo_device_data_ptr);
		void callByImuDeviceDataPtr(boost::shared_ptr<ImuDeviceData> imu_device_data_ptr);
		void callByLaser2dDeviceDataPtr(boost::shared_ptr<Laser2dDeviceData> laser_2d_device_data_ptr);
		void callByLaser3dDeviceDataPtr(boost::shared_ptr<Laser3dDeviceData> laser_3d_device_data_ptr);
		void callByCameraDeviceDataPtr(boost::shared_ptr<CameraDeviceData> camera_device_data_ptr, std::string image_suffix);


		void writeBigDataLoop();
		void readOfflineDataLoop();

		std::string _recordFileDir;
		std::ofstream _recordSensorFile;
		std::ofstream _odoRecordFile;
		std::ofstream _imuRecordFile;
		bool _ifStartRecord;
		bool _ifEndRecord;
		boost::mutex _mutRecordFile;

		std::string _readFileDir;
		int _sleepLineIntvel;
		bool _endReadOfflineData;

		boost::mutex _mutLaser2dBufferMap;
		boost::mutex _mutLaser3dBufferMap;
		boost::mutex _mutCameraBufferMap;
		std::map<SensorDevice, RecordBuffer<Laser2dDeviceData> > _laser2dBufferMap;
		std::map<SensorDevice, RecordBuffer<Laser3dDeviceData> > _laser3dBufferMap;
		std::map<SensorDevice, RecordBuffer<CameraDeviceData> > _cameraBufferMap;
	};

}
#endif