#include "SensorDataHandle.h"
#include <numeric>
#include <iostream>


namespace Localization{

	int BiasEsti::_maxDataSize = 3000;
	int BiasEsti::_levelUpDataSize = 2000;
	void BiasEsti::init(int max_data_size, int level_up_data_size) {
		_maxDataSize = max_data_size;
		_levelUpDataSize = level_up_data_size;
	}

	void BiasEsti::insertData(double w) {
		_dataList.push_back(w);

		int list_size = _dataList.size();

		if (list_size > _maxDataSize) {
			_bias += (w - _dataList.front()) / (list_size - 1);

			_dataList.erase(_dataList.begin());
		}
		else {
			_bias += (w - _bias) / list_size;
		}
	}

	void StaticJudgement::init(double stop_speed_thres) {
		_stopSpeedThres = stop_speed_thres;
	}

	void StaticJudgement::judgeStatic(std::vector<double>& speedVec) {
		while (_speedVecVec.size() < speedVec.size()) {
			std::vector<double> vw;
			_speedVecVec.push_back(vw);
		}

		for (int i = 0; i < _speedVecVec.size() && i<speedVec.size(); ++i) {
			_speedVecVec[i].push_back(speedVec[i]);
			if (_speedVecVec[i].size()>50) {
				_speedVecVec[i].erase(_speedVecVec[i].begin());
			}
		}

		double maxAccSpeed = 0;
		for (int i = 0; i < _speedVecVec.size(); ++i) {
			double accSpeed = 0;
			for (int j = 0; j<_speedVecVec[i].size(); ++j) {
				accSpeed += fabs(_speedVecVec[i][j]);
			}
			accSpeed = accSpeed;

			maxAccSpeed = maxAccSpeed>accSpeed ? maxAccSpeed : accSpeed;
		}

		bool is_static = (maxAccSpeed <= _stopSpeedThres);
		setStatic(is_static);
	}

	void StaticJudgement::setStatic(bool is_static) {
		boost::mutex::scoped_lock loc(_mutIsStatic);
		if (is_static && !_isStatic) {
			std::cout << "StaticJudgement: judge static " << "车静止" << std::endl;
		}
		else if (!is_static && _isStatic) {
			std::cout << "StaticJudgement: judge static " << "车运动" << std::endl;
		}
		_lastIsStatic = _isStatic;
		_isStatic = is_static;
	}

	void StaticJudgement::isStaticByOut(bool is_static_by_out) {
		setStatic(is_static_by_out);
	}

}