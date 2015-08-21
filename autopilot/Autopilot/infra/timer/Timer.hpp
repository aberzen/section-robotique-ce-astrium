/*
 * Timer.hpp
 *
 *  Created on: 14 mai 2015
 *      Author: Aberzen
 */

#ifndef TIMER_HPP_
#define TIMER_HPP_

#include <inttypes.h>

namespace infra {

class Timer {
public:
	Timer();
	virtual ~Timer();

	virtual void start(int16_t durMs);
	virtual void restart();
	virtual void stop();
	virtual bool eval();

	virtual Timer& operator=(int16_t durMs) ;

//protected:
	uint16_t _dateStartMs;
	int16_t _durMs;
};

} /* namespace infra */
#endif /* TIMER_HPP_ */
