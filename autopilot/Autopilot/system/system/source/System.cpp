/*
 * System.cpp
 *
 *  Created on: 24 juil. 2013
 *      Author: Aberzen
 */

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

//	Serial.printf("pInfMat=[\n");
//	for (int iMotor=0 ; iMotor<4 ; iMotor++)
//	{
//		Serial.printf("%3f %3f %3f %3f ;\n",
//				test::config_ancs.modPinv.pInvInfMat[iMotor][0],
//				test::config_ancs.modPinv.pInvInfMat[iMotor][1],
//				test::config_ancs.modPinv.pInvInfMat[iMotor][2],
//				test::config_ancs.modPinv.pInvInfMat[iMotor][3],
//				test::config_ancs.modPinv.pInvInfMat[iMotor][4],
//				test::config_ancs.modPinv.pInvInfMat[iMotor][5]
//				);
//	}
//	Serial.printf("];\n");
//
//	Serial.printf("infMat=[\n");
//	for (int iDoF=0 ; iDoF<6 ; iDoF++)
//	{
//		Serial.printf("%3f %3f %3f %3f ;\n",
//				test::config_ancs.modGen.infMat[iDoF][0],
//				test::config_ancs.modGen.infMat[iDoF][1],
//				test::config_ancs.modGen.infMat[iDoF][2],
//				test::config_ancs.modGen.infMat[iDoF][3]
//				);
//	}
//	Serial.printf("];\n");

	/* Update house keeping */
	getMavHouseKeeping().initialize();

	/* Initialize Mavlink */
	getMavSvcMgr().initialize();

	/* Set mav period to 100ms */
//	getMavHouseKeeping().setDataStream(
//			10,
//			MAV_DATA_STREAM_ALL,
//			1
//	);

	/* Initialize the attitude and navigation control system */
	system::System::system.ancs.smGlobal.initialize();

//	Serial.printf("infMat{%p}=[\n", (void*)&test::config_ancs.modGen.infMat);
//	for (int iDoF=0 ; iDoF<6 ; iDoF++)
//	{
//		Serial.printf("%3f %3f %3f %3f ;\n",
//				test::config_ancs.modGen.infMat[iDoF][0],
//				test::config_ancs.modGen.infMat[iDoF][1],
//				test::config_ancs.modGen.infMat[iDoF][2],
//				test::config_ancs.modGen.infMat[iDoF][3]
//				);
//	}
//	Serial.printf("];\n");
}

/** @brief Execute the process */
void System::execute()
{
	/* Update house keeping */
	getMavHouseKeeping().update();
}

} /* namespace system */
