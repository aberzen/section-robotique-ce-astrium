/*
 * main.cpp
 *
 *  Created on: 17 janv. 2013
 *      Author: Aberzen
 */

#include <Arduino.h>

#include <board/apm/include/Apm25.hpp>

#include <system/tasks/include/ControlCyclicTask.hpp>
#include <system/tasks/include/MavTask.hpp>
#include <system/system/include/System.hpp>

#include <gcs/channel/include/SerialChannel.hpp>
board::Apm25::Param param = {
	{ /* imu */
		hw::HalImuMpu6000::E_GYR_CNF_500DPS, /* gyrCnf */
		hw::HalImuMpu6000::E_ACC_CNF_4G, /* accCnf */
		hw::HalImuMpu6000::E_UPT_FREQ_100HZ /* frequence */
	} /* imu */
};

static board::Apm25 boardApm25(param);
static system::System sys(boardApm25);
system::System& system::System::system = sys;

mavlink_system_t mavlink_system = {7, 1, 0, 0};
mavlink::SerialChannel mavChan0(MAVLINK_COMM_0, Serial);

test::MavTask task(
		(const signed char*)"MAVL",
		(unsigned portSHORT)720,
		(unsigned portBASE_TYPE)1
);

test::ControlCyclicTask taskControl(
		(const signed char*)"CTRL",
		(unsigned portSHORT)720,
		(unsigned portBASE_TYPE)2,
        10,
        1);


int main(int argc, char **argv)
{
	/* Initialize Arduino library */
	arduino_init();

	/* End by starting scheduler */
	vTaskStartScheduler();

}

