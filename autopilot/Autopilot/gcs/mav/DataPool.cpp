/*
 * DataPool.cpp
 *
 *  Created on: 2 juin 2013
 *      Author: Aberzen
 */

#include "../include/DataPool.hpp"

namespace mavlink {

DataPool::DataPool()
: _rawMag_B(0.,0.,0.),
  _rawImuAcc_B(0.,0.,0.),
  _rawImuRate_B(0.,0.,0.),
  _rawBaroPressure(0),
  _rawBaroTemperature(0),
  _estVel_B(0.,0.,0.),
  _estPos_B(0.,0.,0.),
  _estRate_B(0.,0.,0.),
  _estQuat_BI(1.,0.,0.,0.)

{
}


DataPool::~DataPool()
{
}

} /* namespace mavlink */
