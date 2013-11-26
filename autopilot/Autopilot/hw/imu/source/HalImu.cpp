/*
 * HalImu.cpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#include "../include/HalImu.hpp"

namespace hw {

HalImu::HalImu() :
	SensorDriver(),
	_rawRateMeas(0,0,0),
	_rawAccMeas(0,0,0),
	_rawTempMeas(0),
	_rateMeas(0.,0.,0.),
	_accMeas(0.,0.,0.),
	_tempMeas(0.)
{
}

HalImu::~HalImu() {
}

} /* namespace hw */
