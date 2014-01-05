/*
 * HalImu.cpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#include "../include/HalImu.hpp"

namespace hw {

HalImu::HalImu(HalImu::Output& out, HalImu::RawOutput& rawOut) :
	Driver(),
	_rawOut(rawOut),
	_out(out)
{
}

HalImu::~HalImu() {
}

} /* namespace hw */
