/*
 * ControlCyclicTask.cpp
 *
 *  Created on: 23 juil. 2013
 *      Author: Aberzen
 */

#include <stdio.h>

#include "ControlCyclicTask.hpp"
#include <system/system/System.hpp>

namespace system {

ControlCyclicTask::ControlCyclicTask(
		const signed char * pcName,
        unsigned portSHORT usStackDepth,
        unsigned portBASE_TYPE uxPriority,
        uint16_t period,
        uint16_t delay)
: infra::CyclicTask(pcName,usStackDepth,uxPriority,period,delay)
{
}

ControlCyclicTask::~ControlCyclicTask() {
}

/** @brief Non returning function executed as the task body function. */
void ControlCyclicTask::init(void)
{
	/* Execute Sensors */
	system.initialize();

//	hw::Serial& com = system::getCom0();
//	com.write((uint8_t *)"Hello\n",6);

	Task::init();
}

void ControlCyclicTask::runCycle(void)
{
	if (_missed != 0)
	{
		_missed = 0;
	}

	// Execute
//	hw::Serial& com = system::getCom0();
//	com.write((uint8_t*)"Execute\n",8);
	system.execute();
}


} /* namespace test */
