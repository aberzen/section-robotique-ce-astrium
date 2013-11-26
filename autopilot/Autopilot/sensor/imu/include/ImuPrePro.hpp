/*
 * ImuPrePro.hpp
 *
 *  Created on: 5 juil. 2013
 *      Author: Aberzen
 */

#ifndef IMUPREPRO_HPP_
#define IMUPREPRO_HPP_

#include <math/include/Matrix3.hpp>

namespace sensor {

class ImuPrePro {
public:
	ImuPrePro(
			math::Matrix3f& mat_UB,
			math::Vector3f& bias_B );
	virtual ~ImuPrePro();

	void preProcess(const math::Vector3f& vec_U, math::Vector3f& vec_B);

protected:
	math::Matrix3f& _mat_UB;
	math::Vector3f& _bias_B;
};

} /* namespace sensor */
#endif /* IMUPREPRO_HPP_ */
