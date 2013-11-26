/*
 * System.cpp
 *
 *  Created on: 24 juil. 2013
 *      Author: Aberzen
 */

#include <hw/serial/include/FastSerial.hpp>
#include "../include/System.hpp"
#include <config/include/nrd.h>
#include <test/Parameters.hpp>

namespace system {

System::System(board::Board& board) :
	Process(),
	_board(board),
	_modeMgt(),
	_paramMgt(test::config, CONFIG_PARAMETERS_COUNT),
	_wpMgt(),
	_hkMgt(MAVLINK_COMM_0),
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
	),
	_est(board.getImu(), board.getBarometer(), _board.getMagnetometer())
{

}

System::~System()
{

}

/** @brief Init the process */
status System::initialize()
{
	/* Execute board */
	getBoard().initialize();

	/* Execute estimator */
	getEstimator().initialize();

	/* Parameter management */
	if (!getParameterMgt().checkEeprom())
		getParameterMgt().resetEeprom();

	/* Update house keeping */
	getMavHouseKeeping().initialize();

	/* Initialize Mavlink */
	getMavSvcMgr().initialize();

	/* Set mav period to 100ms */
	getMavHouseKeeping().setDataStream(
			1000,
			MAV_DATA_STREAM_ALL,
			1
	);

	return 0;
}

/** @brief Execute the process */
status System::execute()
{
	/* Execute board */
	getBoard().execute();

	/* Execute estimator */
	getEstimator().execute();

	/* Update house keeping */
	getMavHouseKeeping().update();

	/* No error */
	return 0;
}

} /* namespace system */
