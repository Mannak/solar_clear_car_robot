#ifndef NORMAL_SLEEP_HPP
#define NORMAL_SLEEP_HPP


class NormalSleep {

public:
	//����
	static void sleep(float time);

};

#define SLEEP(time) NormalSleep::sleep(time)

#endif
