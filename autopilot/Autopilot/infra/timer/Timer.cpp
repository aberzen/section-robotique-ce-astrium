/*
 * Timer.cpp
 *
 *  Created on: 14 mai 2015
 *      Author: Aberzen
 */

#include "Timer.hpp"
#include <Arduino.h>

namespace infra {

Timer::Timer()
: _dateStartMs(0),
  _durMs(-1)
{

}

Timer::~Timer()
{
}

Timer& Timer::operator=(int16_t durMs)
{
	start(durMs);
	return *this;
}

void Timer::start(int16_t durMs)
{
	if (durMs>0)
	{
		_dateStartMs = millis();
		_durMs = durMs;
	}
	else
		stop();
}

void Timer::restart()
{
	_dateStartMs = millis();
	if (_durMs<0)
		_durMs = -_durMs;
}

void Timer::stop()
{
	if (_durMs>0)
		_durMs = -_durMs;
}

bool Timer::eval()
{
	uint16_t dur = (millis() - _dateStartMs) ;
	uint16_t dt = dur - ((uint16_t)_durMs);
	bool result = ((_durMs>0) && (dt < (2^15)));
	if (result)
		stop();

	return result;
}


} /* namespace infra */
