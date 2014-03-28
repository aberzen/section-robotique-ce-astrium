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
	uint16_t* channels = &system::System::system.board.pwmVal.channels[0];
	uint16_t* radio = &system::System::system.board.radio.channels[0];
	bool* isAvailable = &system::System::system.board.radio.isAvailable;
//	const hw::Pwm* pwm = &system::System::system.board.pwm;
	channels[4] = 900;
	system::System::system.board.pwm.force_out(4);
	system::System::system.board.pwm.enable_out(4);

	while(true)
	{
#if 0
		/* Run mavlink */
		system::System::system.getMavSvcMgr().execute();
#endif

		system::System::system.board.execute();
		if (*isAvailable)
		{
			channels[4] = radio[2];
			Serial.printf("%u\n",radio[2]);
		}
//		char buffer[128];
//		size_t tmp = Serial.readBytes(buffer, 127);
//		buffer[tmp] = 0;
//		Serial.print(buffer);
	}
}

} /* namespace test */
