/*
 * HalMagnetometer.cpp
 *
 *  Created on: 31 juil. 2013
 *      Author: Aberzen
 */

#include "../include/HalMagnetometer.hpp"

namespace hw {

HalMagnetometer::HalMagnetometer() :
	SensorDriver(),
	_rawMagFieldMeas(0,0,0),
	_magFieldMeas(0.,0.,0.)
{
	// TODO Auto-generated constructor stub

}

HalMagnetometer::~HalMagnetometer() {
}

} /* namespace hw */
