/*
 * Driver.cpp
 *
 *  Created on: 7 janv. 2013
 *      Author: Aberzen
 */

#include "Driver.hpp"

namespace hw {

Driver::Driver()
: _isInitialized(false)
{
}

Driver::~Driver()
{
}

bool Driver::initialize()
{
	_isInitialized = true;
	return isInitialized();
}

void Driver::reset()
{
	_isInitialized = false;
}


} /* namespace hw */
