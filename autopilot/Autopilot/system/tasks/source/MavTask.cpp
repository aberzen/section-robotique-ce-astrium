/*
 * MavTask.cpp
 *
 *  Created on: 26 juin 2013
 *      Author: Aberzen
 */

#include <hw/serial/include/FastSerial.hpp>
#include "../include/MavTask.hpp"
#include <system/system/include/System.hpp>

namespace test {

MavTask::MavTask(const signed char * pcName,
        unsigned portSHORT usStackDepth,
        unsigned portBASE_TYPE uxPriority) :
        	infra::Task(pcName,usStackDepth,uxPriority)
{
}

MavTask::~MavTask() {
}


/** @brief Non returning function executed as the task body function. */
void MavTask::init(void)
{
	Task::init();
}

/** @brief Non returning function executed as the task body function. */
void MavTask::run(void)
{
	while(true)
	{
#if 1
		/* Run mavlink */
		system::System::system.getMavSvcMgr().execute();
#endif
	}
}

} /* namespace test */
