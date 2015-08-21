/*
 * CyclicTask.cpp
 *
 *  Created on: 18 mai 2013
 *      Author: Aberzen
 */

#include "CyclicTask.hpp"
#include "CyclicMgr.hpp"

namespace infra {

CyclicTask::CyclicTask(
		const signed char * pcName,
        unsigned portSHORT usStackDepth,
        unsigned portBASE_TYPE uxPriority,
        uint16_t period,
        uint16_t delay):
        Task(pcName, usStackDepth, uxPriority),
        _periodCnf(period),
        _periodTicks(period),
        _delayCnf(delay),
		_delayTicks(delay),
		_missed(0),
		_isWaiting(false)
        {
	/* Create synchronization */
	_synchSem = xSemaphoreCreateCounting(1, 0);
	CyclicMgr::registerTask(this);
}

CyclicTask::~CyclicTask() {

}

void CyclicTask::run(void)
{
	/* Start cyclic */
	while(true)
	{
		/* Wait new cycle */
		waitNextPeriod();

		/* Process current exec frame */
		runCycle();
	}
}

void CyclicTask::tick() {
	if (0 != _delayTicks)
	{
		_delayTicks--;
	}
	else if (0==(--_periodTicks))
	{
		/*  New period */
		_periodTicks = _periodCnf;

		/* Increment counter */
		if (!_isWaiting)
			_missed++;

		/* Release */
		xSemaphoreGiveFromISR(_synchSem,NULL);
	}
}

void CyclicTask::waitNextPeriod()
{
	_isWaiting = true;
	/* Release */
	xSemaphoreTake(_synchSem,0xFFFF/*_periodCnf*/);
	_isWaiting = false;
}

} /* namespace hw */
