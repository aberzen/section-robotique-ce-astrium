/*
 * main.cpp
 *
 *  Created on: 17 janv. 2013
 *      Author: Aberzen
 */

#include <gcs/channel/mavlink_bridge.hpp>
#include <Arduino.h>
#include <system/system/System.hpp>
#include <system/tasks/ControlCyclicTask.hpp>
#include <gcs/channel/SerialChannel.hpp>


system::ControlCyclicTask taskControl(
		(const signed char*)"CTRL",
		(unsigned portSHORT)650,
		(unsigned portBASE_TYPE)2,
		FSW_TASK_CTRL_PERIOD_TICK,
        1);

int main(int argc, char **argv)
{
	/* Initialize Arduino library */
	arduino_init();

	/* End by starting scheduler */
	vTaskStartScheduler();

}

