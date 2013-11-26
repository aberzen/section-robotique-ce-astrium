/*
 * Task.cpp
 *
 *  Created on: 8 janv. 2013
 *      Author: Aberzen
 */

#include "../include/Task.hpp"

namespace infra {

Task::Task(
		const signed char * pcName,
        unsigned portSHORT usStackDepth,
        unsigned portBASE_TYPE uxPriority):
        _isInitialized(false) {

	/* Create the task */
	_status = xTaskCreate(runTask, (const signed char *) pcName, usStackDepth, (void *)this, uxPriority, &_pvCreatedTask);

}


Task::~Task() {
#if INCLUDE_vTaskDelete == 1
	/* Delete the task */
	_status = xTaskDelete(_pvCreatedTask);
#endif
}

void runTask(void *pvParameters)
{
	/* Initialize task */
	((Task*)pvParameters)->init();

	/* Execute */
	((Task*)pvParameters)->run();
}

/** @brief Task initialization */
void Task::init(void)
{
	_isInitialized = true;
}

} /* namespace arducopter */
