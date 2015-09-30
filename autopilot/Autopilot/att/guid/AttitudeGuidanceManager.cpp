/*
 * AttitudeGuidanceManager.cpp
 *
 *  Created on: 27 août 2015
 *      Author: AdministrateurLocal
 */

#include <system/system/System.hpp>
#include <att/guid/AttitudeGuidanceManager.hpp>

namespace attitude {

AttitudeGuidanceManager::AttitudeGuidanceManager()
: _mode(E_MODE_NONE),
  _guidAutoStab(),
  _guidAccro()
{
}

AttitudeGuidanceManager::~AttitudeGuidanceManager()
{
}

bool AttitudeGuidanceManager::setMode(Mode mode)
{
	bool result = false;

	if (_mode == mode)
		return true;

	switch (mode)
	{
	case E_MODE_AUTOSTAB:
	case E_MODE_AUTOSTAB_NOYAW:
		result = switchToAutoStab();
		break;
	case E_MODE_ACCRO:
		result = switchToAccro();
		break;
	case E_MODE_NONE:
		break;
	}

	if (result)
		_mode = mode;

	return result;
}

/** @brief Execute attitude guidance mode manager */
void AttitudeGuidanceManager::execute()
{
	switch (_mode)
	{
	case E_MODE_AUTOSTAB:
		_guidAutoStab.calcGuidance(
				system::system.getRadio(),
				system::system.dataPool.guidAtt_IB,
				system::system.dataPool.guidRate_B,
				true);
		break;
	case E_MODE_AUTOSTAB_NOYAW:
		_guidAutoStab.calcGuidance(
				system::system.getRadio(),
				system::system.dataPool.guidAtt_IB,
				system::system.dataPool.guidRate_B,
				false);
		break;
	case E_MODE_ACCRO:
		_guidAccro.calcGuidance(
				system::system.getRadio(),
				system::system.dataPool.guidAtt_IB,
				system::system.dataPool.guidRate_B);
		break;
	case E_MODE_NONE:
		break;
	}
}

bool AttitudeGuidanceManager::switchToAutoStab()
{
	if (_mode == E_MODE_NONE)
		_guidAutoStab.initialize(
				system::system.dataPool.estAtt_IB,
				system::system.dataPool.estRate_B);
	else
		_guidAutoStab.initialize(
				system::system.dataPool.guidAtt_IB,
				system::system.dataPool.guidRate_B);
	return true;
}

bool AttitudeGuidanceManager::switchToAccro()
{
	if (_mode == E_MODE_NONE)
		_guidAccro.initialize(
				system::system.dataPool.estAtt_IB,
				system::system.dataPool.estRate_B);
	else
		_guidAccro.initialize(
				system::system.dataPool.guidAtt_IB,
				system::system.dataPool.guidRate_B);
	return true;
}


} /* namespace attitude */
