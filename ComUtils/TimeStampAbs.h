#ifndef _TIME_STAMP_ABS_H_
#define _TIME_STAMP_ABS_H_


typedef enum {
	ABS_TIME_STAMP = 0,
	TIME_OF_DAY,
	TICK_TIME_STAMP,
	QUERY_PERFORMANCE
} TIME_STAMP_TYPE;

class TimeStampAbs {
public:
	static double getTimeStamp(TIME_STAMP_TYPE type);

	static double getAbsTimeStamp();

	static double getTimeOfDay();

	static double getTickTimeStamp();

	static double getQueryPerformanceTime();
};

#endif