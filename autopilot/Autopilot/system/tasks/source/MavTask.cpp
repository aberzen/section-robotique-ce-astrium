/*
 * MavTask.cpp
 *
 *  Created on: 26 juin 2013
 *      Author: Aberzen
 */

#include "../include/MavTask.hpp"
#include <system/params/include/Parameters.hpp>
#include <system/system/include/System.hpp>
#include <hw/serial/include/FastSerial.hpp>

namespace test {

//mavlink::HouseKeepingMgt* hkMgt = NULL;

MavTask::MavTask(const signed char * pcName,
        unsigned portSHORT usStackDepth,
        unsigned portBASE_TYPE uxPriority) :
        	infra::Task(pcName,usStackDepth,uxPriority)
//			_modeMgt(),
//			_paramMgt(config, CONFIG_PARAMETERS_COUNT),
//			_wpMgt(),
//			_hkMgt(MAVLINK_COMM_0),
//			_cmdMgt(),
//			_mountMgt(),
//			_fenceMgt(),
//			_digivcamMgt(),
//			_mavSvcMgt (
//					MAVLINK_COMM_0,
//					11,
//					MAV_COMP_ID_SYSTEM_CONTROL,
//					_modeMgt,
//					_paramMgt,
//					_wpMgt,
//					_hkMgt,
//					_cmdMgt,
//					_mountMgt,
//					_fenceMgt,
//					_digivcamMgt)
{
//	hkMgt = &_hkMgt;
}

MavTask::~MavTask() {
}


/** @brief Non returning function executed as the task body function. */
void MavTask::init(void)
{
//	/* Parameter management */
//	if (!_paramMgt.checkEeprom())
//	{
//		_paramMgt.resetEeprom();
//	}
//
//	/* Initialize */
//	_hkMgt.initialize();

	/* @TEST */
//	system::System::system.getMavHouseKeeping().setDataStream(100,MAV_DATA_STREAM_ALL,true);

	/* Initialize the Mavlink service management */
//	_mavSvcMgt.initialize();

//	system::System::system.initialize();
	/* Mark as initialized */
	Task::init();
}

/** @brief Non returning function executed as the task body function. */
void MavTask::run(void)
{
	while(true)
	{
//		_hkMgt.execute();
//#if 0
		/* Run mavlink */
		system::System::system.getMavSvcMgr().execute();
//		system::System::system.getMavHouseKeeping().execute();
//#endif
		infra::Task::delay(10);
	}
}

} /* namespace test */
