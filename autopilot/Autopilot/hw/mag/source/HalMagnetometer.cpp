/*
 * HalMagnetometer.cpp
 *
 *  Created on: 31 juil. 2013
 *      Author: Aberzen
 */

#include "../include/HalMagnetometer.hpp"

namespace hw {

HalMagnetometer::HalMagnetometer(Output& out, RawOutput& rawOut) :
	Driver(),
	_rawOut(rawOut),
	_out(out)
{
	// TODO Auto-generated constructor stub

}

HalMagnetometer::~HalMagnetometer() {
}

} /* namespace hw */
