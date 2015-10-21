/*
 * AttitudeManager.cpp
 *
 *  Created on: 31 août 2015
 *      Author: AdministrateurLocal
 */

//#include <system/system/System.hpp>
#include <att/mgt/AttitudeManager.hpp>

namespace attitude {

AttitudeManager::AttitudeManager(const AttitudeManager::Parameter& param)
: _guidMgt(),
  _ctrl(param.ctrl)
{
}

AttitudeManager::~AttitudeManager()
{
}

bool AttitudeManager::setMode(AttitudeManager::Mode mode)
{
	bool result = false;

	if (_mode == mode)
		return true;

	switch (mode)
	{
	case E_ATT_MODE_STABNOYAW:
		result = switchToAutoStab();
		break;
	case E_ATT_MODE_NONE:
	default:
		result = switchToNone();
		break;
	}

	if (result)
		_mode = mode;

	return result;
}

/** @brief periodic execution */
void AttitudeManager::execute()
{
	switch (_mode)
	{
	case E_ATT_MODE_STABNOYAW:
		/* Execute the guidance */
		_guidMgt.execute();

		/* Execute the controller */
		_ctrl.execute();

		break;
	case E_ATT_MODE_NONE:
	default:
		/* Nothing to do */
		break;
	}
}

/** @brief Switch to auto stab */
bool AttitudeManager::switchToAutoStab()
{
	bool result = false;

	/* switch to auto stab guidance */
	result = _guidMgt.setMode(AttitudeGuidanceManager::E_MODE_AUTOSTAB_NOYAW);

	if (result && (_mode == E_ATT_MODE_NONE))
	{
		/* Reset the controller */
		_ctrl.reset();
	}

	return result;
}

/** @brief Switch to none */
bool AttitudeManager::switchToNone()
{
	bool result = false;

	/* switch to auto stab guidance */
	result = _guidMgt.setMode(AttitudeGuidanceManager::E_MODE_NONE);

	if (result && (_mode == E_ATT_MODE_NONE))
	{
		/* Reset the controller */
		_ctrl.reset();
	}

	return result;
}

} /* namespace attitude */
