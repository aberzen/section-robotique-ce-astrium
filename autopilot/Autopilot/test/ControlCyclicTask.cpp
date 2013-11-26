/*
 * ControlCyclicTask.cpp
 *
 *  Created on: 23 juil. 2013
 *      Author: Aberzen
 */

#include "ControlCyclicTask.hpp"
#include <board/gen/include/Board.hpp>
#include <system/system/include/System.hpp>

#include <hw/serial/include/FastSerial.hpp>

#include <Arduino.h>

#include "MavTask.hpp"
namespace test {

ControlCyclicTask::ControlCyclicTask(
		const signed char * pcName,
        unsigned portSHORT usStackDepth,
        unsigned portBASE_TYPE uxPriority,
        uint16_t period,
        uint16_t delay) :
	infra::CyclicTask(pcName,usStackDepth,uxPriority,period,delay)
{
}

ControlCyclicTask::~ControlCyclicTask() {
}

/** @brief Non returning function executed as the task body function. */
void ControlCyclicTask::init(void)
{
	/* Execute Sensors */
	system::System::system.initialize();
	Serial.printf("ControlCyclicTask::init\n");
	Task::init();
}

void ControlCyclicTask::runCycle(void)
{
	if (_missed != 0)
	{
		Serial.printf("ControlCyclicTask::runCycle _missed = %d\n",_missed);
		_missed = 0;
	}


	/* Execute Sensors */
	system::System::system.execute();
}


} /* namespace test */
