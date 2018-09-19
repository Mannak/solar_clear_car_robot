#include "NormalSleep.h"
#include <string.h>
#include <iostream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>


void NormalSleep::sleep(float time){
	boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds((long)time);
	boost::thread::sleep(timeout);
}