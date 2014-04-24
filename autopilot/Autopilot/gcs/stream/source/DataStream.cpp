/*
 * DataStream.cpp
 *
 *  Created on: 22 juil. 2013
 *      Author: Aberzen
 */

#include <infra/include/Task.hpp>
#include <Arduino.h>
#include "../include/DataStream.hpp"
#include <system/params/include/Nrd.hpp>


namespace mavlink {

DataStream::DataStream(mavlink_channel_t port) :
		_port(port),
		_period(0),
		_counter(0),
		_date(0)
{
}

DataStream::~DataStream()
{

}

/** @brief Update stream counter */
bool DataStream::increment()
{
	bool result;
	if (_period != 0)
		if (_counter != 0)
		{
			infra::Task::enterCritical();
			// Ensure atomic execution
			result = ((--_counter) ==  0);
			infra::Task::leaveCritical();
			return result;
		}
	return false;
}

/** @brief Reset stream counter */
void DataStream::reset()
{
	_counter = _period;
}


/** @brief Sample the stream */
void DataStream::sample()
{
	/* Sample the date */
	sampleDate();

	/* Sample the data */
	sampleData();
}

/** @brief Sample the date */
void DataStream::sampleDate()
{
	_date = (uint64_t) micros();
}

/** @brief Update data stream */
void DataStream::update()
{
	/* Update counter */
	if (increment())
	{
		/* Time to sample data */
		sample();
	}
}

/** @brief Init the process */
void DataStream::initialize()
{
	_counter = 0;
	_period = 0;
	_date = 0;
}

/** @brief Execute the process */
void DataStream::execute()
{
	if ((_period != 0) && (_counter == 0))
	{
		/* Send data */
		sendData();

		/* Reset counters */
		reset();
	}
}

/** @brief Stop streaming */
void DataStream::stop()
{
	infra::Task::enterCritical();
	_period = 0;
	_counter = 0;
	infra::Task::leaveCritical();
}

/** @brief Start streaming */
void DataStream::start(uint16_t periodMs)
{
	infra::Task::enterCritical();
	_period = periodMs/FSW_TASK_CTRL_PERIOD_MSEC; // convert into period
	if (_period*FSW_TASK_CTRL_PERIOD_MSEC < periodMs)
	{
		/* Round to upper value */
		_period ++;
	}
	_counter = _period;
	infra::Task::leaveCritical();
}



} /* namespace mavlink */
