/*
 * NavigationDirectThrust.cpp
 *
 *  Created on: 31 août 2015
 *      Author: AdministrateurLocal
 */

#include <nav/guid/NavigationGuidance.hpp>
#include "NavigationDirectThrust.hpp"

#include <system/system/System.hpp>

namespace navigation {

NavigationDirectThrust::NavigationDirectThrust(const Parameter& param)
: _param(param)
{

}

NavigationDirectThrust::~NavigationDirectThrust()
{
}

void NavigationDirectThrust::calcGuidance(
		hw::Radio& radio)
{
	int32_t thrustCmd = (int32_t) radio.getUnsigned(hw::Radio::E_RADIO_CHANNEL_THRUST);
	system::system.dataPool.ctrlFrcDemB(
			(_param.unitThrust[NAVIGATION_GUIDANCE_IDX_X] * thrustCmd) >> 10,
			(_param.unitThrust[NAVIGATION_GUIDANCE_IDX_Y] * thrustCmd) >> 10,
			(_param.unitThrust[NAVIGATION_GUIDANCE_IDX_Z] * thrustCmd) >> 10);
}


} /* namespace navigation */
