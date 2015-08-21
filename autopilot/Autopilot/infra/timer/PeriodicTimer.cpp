/*
 * PeriodicTimer.cpp
 *
 *  Created on: 14 mai 2015
 *      Author: Aberzen
 */

#include "PeriodicTimer.hpp"
#include <Arduino.h>

namespace infra {

PeriodicTimer::PeriodicTimer()
: Timer()
{
}

PeriodicTimer::~PeriodicTimer()
{
}

bool PeriodicTimer::eval()
{
	uint16_t now = millis();
	uint16_t dur = (now - _dateStartMs) ;
	uint16_t dt = dur - ((uint16_t)_durMs);
	bool result = ((_durMs>=0) && (dt < 2^15));
	if (result)
		_dateStartMs = now - dt;

	return result;
}

} /* namespace infra */
