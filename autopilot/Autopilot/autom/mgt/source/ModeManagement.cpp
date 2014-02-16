/*
 * ModeManagement.cpp
 *
 *  Created on: 10 f�vr. 2014
 *      Author: Robotique
 */

#include <autom/mgt/include/ModeManagement.hpp>
#include <system/system/include/System.hpp>

namespace autom
{

ModeManagement::ModeManagement(
		/* Parameters */
		const float& dt,
		const Param& param,
		const GenericParam& paramGen
		)
: Process(),
  _currMode(E_MODE_STABILIZED),
  _modeStabilitized(
 		dt,
 		param.modeStabilized,
 		paramGen
 		 )
{

}

ModeManagement::~ModeManagement()
{

}

/** @brief Init the process */
void ModeManagement::initialize()
{
	/* Initialize */
	_modeStabilitized.initialize();
}

/** @brief Execute the process */
void ModeManagement::execute()
{
	if (system::System::system.ancs.smFlyingState.getState() == FlyingState::E_STATE_ARMED)
	{
		/* Execute current mode */
		executeMode();
	}
}

/** @brief Execute current mode*/
void ModeManagement::executeMode()
{
	switch(_currMode)
	{
	case E_MODE_STABILIZED:
		_modeStabilitized.execute();
		break;
	case E_MODE_ACCRO:
		break;
	case E_MODE_ALT_HOLD:
		break;
	case E_MODE_POS_HOLD:
		break;
	case E_MODE_TAKE_OFF:
		break;
	case E_MODE_LANDING:
		break;
	case E_MODE_INERT_VELOCITY:
		break;
	case E_MODE_INERT_POSITION:
		break;
	case E_MODE_WP:
		break;
	default:
		/* Unknwon mode
		 * Command a failsafe */
		// TODO command a failsafe
		break;
	}
}

} /* autom */

