/************************************************************************
Author: huanghong
Date: 2016-10-31
************************************************************************/

#ifndef _SENSOR_DATA_HANDLE_
#define _SENSOR_DATA_HANDLE_

#include <boost/thread.hpp>
#include <list>
#include <vector>


namespace Localization{

	class BiasEsti {
	public:
		static void init(int max_data_size = 3000, int level_up_data_size = 2000);

		BiasEsti() {
			_bias = 0;
		}

		double getBias() {
			return _bias;
		}

		int getDataSize() {
			return _dataList.size();
		}

		void insertData(double w);

		bool ifDataSizeLevelUp() {
			return _dataList.size() >= _levelUpDataSize;
		}

	private:
		double _bias;
		std::list<double> _dataList;
		static int _maxDataSize;
		static int _levelUpDataSize;
	};

	class StaticJudgement {
	public:
		StaticJudgement() {
			_isStatic = false;
			_lastIsStatic = false;
		}

	public:
		/*static StaticJudgement* instance() {
			static StaticJudgement ins;
			return &ins;
			}*/

		void init(double stop_speed_thres = 0.01);

		void isStaticByOut(bool is_static_by_out);
		void judgeStatic(std::vector<double>& speedVec);

		void setStatic(bool is_static);
		bool isStatic() {
			boost::mutex::scoped_lock loc(_mutIsStatic);
			return _isStatic;
		}
		bool lastIsStatic() {
			return _lastIsStatic;
		}

	private:
		boost::mutex _mutIsStatic;
		bool _isStatic;
		bool _lastIsStatic;
		std::vector<std::vector<double> > _speedVecVec;
		double _stopSpeedThres;
	};

}
#endif