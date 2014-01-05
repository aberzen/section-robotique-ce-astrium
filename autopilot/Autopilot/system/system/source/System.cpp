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


float config_dt = 0.01; /* sec */
autom::Ancs::Param config_ancs = {
		{ /* stabilized */
			{ /* att */
				{ /* x */
					3947.8418, /* _Kp */
					0.8796459, /*_Kd */
					0., /*_Ki */
					false, /*_useOfRb */
					0., /*_Krb */
					0., /*_rbThd */
					0., /*_rb */
					0., /*_maxI */
				}, /* x */
				{ /* y */
					3947.8418, /* _Kp */
					0.8796459, /*_Kd */
					0., /*_Ki */
					false, /*_useOfRb */
					0., /*_Krb */
					0., /*_rbThd */
					0., /*_rb */
					0., /*_maxI */
				}, /* y */
				{ /* z */
				    39.478418, /* _Kp */
					0.08796459, /*_Kd */
					0., /*_Ki */
					false, /*_useOfRb */
					0., /*_Krb */
					0., /*_rbThd */
					0., /*_rb */
					0., /*_maxI */
				}, /* z */
			}, /* att */
			{ /* nav */
					{ /* x */
						0., /* _Kp */
						0., /*_Kd */
						0., /*_Ki */
						false, /*_useOfRb */
						0., /*_Krb */
						0., /*_rbThd */
						0., /*_rb */
						0., /*_maxI */
					}, /* x */
					{ /* y */
						0., /* _Kp */
						0., /*_Kd */
						0., /*_Ki */
						false, /*_useOfRb */
						0., /*_Krb */
						0., /*_rbThd */
						0., /*_rb */
						0., /*_maxI */
					}, /* y */
					{ /* z */
						0., /* _Kp */
						0., /*_Kd */
						0., /*_Ki */
						false, /*_useOfRb */
						0., /*_Krb */
						0., /*_rbThd */
						0., /*_rb */
						0., /*_maxI */
					}, /* z */
			}, /* nav */
		}, /* stabilized */
		{ /* est */
			0.05, /* gainAcco */
			0.005, /* gainCompass */
		}, /* est */
		{ /* procCalibImu */
				{0.002493765586, 0.002493765586}, /* filtCoeffNum */
				{1.0000000000000e0,-0.995012468828}, /* filtCoeffDen */
				2000, /* biasNbMeas */
				50, /* varNbMeas */
				0.001, /* gyroVarianceThd */
				0.002, /* accoVarianceThd */
		}, /* procCalibImu */
		{ /* procCompassDeclin */
				{0.002493765586, 0.002493765586}, /* filtCoeffNum */
				{1.0000000000000e0,-0.995012468828}, /* filtCoeffDen */
				2000, /* biasNbMeas */
		}, /* procCompassDeclin */
		{ /* mod */
				{ /* infMat */
					    {2.8926112,    2.8926112,  - 2.8926112,  - 2.8926112  },
					    {2.8926112,  - 2.8926112,  - 2.8926112,    2.8926112  },
					    {0.681795 ,  - 0.681795 ,    0.681795 ,  - 0.681795   },
					    {0.       ,    0.       ,    0.       ,    0.         },
					    {0.       ,    0.       ,    0.       ,    0.         },
					    {6.81795  ,    6.81795  ,    6.81795  ,    6.81795    }
				}, /* infMat */
				{ /* pInvInfMat */
						{  0.0864271,    0.0864271,    0.3666791,    0.,    0.,    0.0366679  },
						{  0.0864271,  - 0.0864271,  - 0.3666791,    0.,    0.,    0.0366679  },
					    {- 0.0864271,  - 0.0864271,    0.3666791,    0.,    0.,    0.0366679  },
					    {- 0.0864271,    0.0864271,  - 0.3666791,    0.,    0.,    0.0366679  }
				}, /* pInvInfMat */
				{ /* descVect */
						1., 1., 1., 1.
				} /* descVect */
		}, /* mod */
		{ /* modeStabilized */
				-10, /* rollPwmScale */
				-10, /* pitchPwmScale */
				-10, /* yawRatePwmScale */
				- 6, /* thrustPwmScale */
				math::Vector3f(0.,0.,1.), /* thrustDir_B */
				1.600 /* mass */
		} /* modeStabilized */
};

System::System(board::Board& board) :
	Process(),
	_board(board),
	_modeMgt(),
	_paramMgt(test::config, CONFIG_PARAMETERS_COUNT),
	_wpMgt(),
	_mgt(
			config_dt,
			config_ancs),
	_hkMgt(MAVLINK_COMM_0, _mgt.getEstimationValues(), board.meas, board.rawMeas),
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
//	,
//	_est(board.getImu(), board.getBarometer(), _board.getMagnetometer())
{

}

System::~System()
{

}

/** @brief Init the process */
infra::status System::initialize()
{
	/* Execute board */
	getBoard().initialize();

//	/* Execute estimator */
//	getEstimator().initialize();

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
infra::status System::execute()
{
	/* Execute board */
//	getBoard().execute();

//	/* Execute estimator */
//	getEstimator().execute();

	/* Update house keeping */
	getMavHouseKeeping().update();

	/* No error */
	return 0;
}

} /* namespace system */
