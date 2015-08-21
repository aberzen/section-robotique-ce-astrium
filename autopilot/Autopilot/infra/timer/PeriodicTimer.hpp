/*
 * PeriodicTimer.hpp
 *
 *  Created on: 14 mai 2015
 *      Author: Aberzen
 */

#ifndef PERIODICTIMER_HPP_
#define PERIODICTIMER_HPP_

#include "Timer.hpp"

namespace infra {

class PeriodicTimer : public Timer {
public:
	PeriodicTimer();
	virtual ~PeriodicTimer();

	virtual bool eval();

protected:

};

} /* namespace infra */
#endif /* PERIODICTIMER_HPP_ */
