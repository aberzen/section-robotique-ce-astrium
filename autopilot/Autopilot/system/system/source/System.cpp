/*
 * System.cpp
 *
 *  Created on: 24 juil. 2013
 *      Author: Aberzen
 */

#include <hw/serial/include/FastSerial.hpp>
#include "../include/System.hpp"
#include <system/params/include/Nrd.hpp>
#include <system/params/include/Parameters.hpp>

namespace system {


const float config_dt_HF = 0.01; /* sec */
const float config_dt_LF = 0.1; /* sec */


System::System(board::Board& board) :
	Process(),
	board(board),
	_modeMgt(),
	_paramMgt(test::config, CONFIG_PARAMETERS_COUNT),
	_wpMgt(),
	ancs(
			config_dt_HF,
			config_dt_LF,
			test::config_ancs),
	_hkMgt(MAVLINK_COMM_0, ancs.estimations, board.meas, board.rawMeas),
	_cmdMgt(),
	_mountMgt(),
	_fenceMgt(),
	_digivcamMgt(),
	_mavSvcMgt(MAVLINK_COMM_0,
			11,
			MAV_COMP_ID_SYSTEM_CONTROL,
			System::_modeMgt,
			System::_paramMgt,
			System::_wpMgt,
			System::_hkMgt,
			System::_cmdMgt,
			System::_mountMgt,
			System::_fenceMgt,
			System::_digivcamMgt
	)
{
}

System::~System()
{

}

/** @brief Init the process */
void System::initialize()
{
	/* Parameter management */
	if (!getParameterMgt().checkEeprom())
		getParameterMgt().resetEeprom();

	/* Load all values */
	getParameterMgt().loadAllValues();

	/* Initialize the board */
	board.initialize();

	/* Update house keeping */
	getMavHouseKeeping().initialize();

	/* Initialize Mavlink */
	getMavSvcMgr().initialize();

	/* Set mav period to 100ms */
	getMavHouseKeeping().setDataStream(
			10,
			MAV_DATA_STREAM_ALL,
			1
	);

	/* Initialize the attitude and navigation control system */
	system::System::system.ancs.initialize();
}

/** @brief Execute the process */
void System::execute()
{
	/* Update house keeping */
	getMavHouseKeeping().update();
}

} /* namespace system */
