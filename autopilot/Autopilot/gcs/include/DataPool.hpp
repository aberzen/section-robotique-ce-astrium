/*
 * DataPool.hpp
 *
 *  Created on: 2 juin 2013
 *      Author: Aberzen
 */

#ifndef DATAPOOL_HPP_
#define DATAPOOL_HPP_

#include <math/include/Quaternion.hpp>

namespace mavlink {

class DataPool {
public:
	DataPool();
	virtual ~DataPool() {
		// TODO Auto-generated destructor stub
	}

public:
	math::Vector3f _rawMag_B;
	math::Vector3f _rawImuAcc_B;
	math::Vector3f _rawImuRate_B;
	float _rawBaroPressure;
	float _rawBaroTemperature;

	math::Vector3f _estVel_B;
	math::Vector3f _estPos_B;
	math::Vector3f _estRate_B;
	math::Quaternion _estQuat_BI;

};

} /* namespace mavlink */
#endif /* DATAPOOL_HPP_ */
