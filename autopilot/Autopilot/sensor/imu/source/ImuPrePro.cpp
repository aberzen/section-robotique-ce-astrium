/*
 * ImuPrePro.cpp
 *
 *  Created on: 5 juil. 2013
 *      Author: Aberzen
 */

#include "../include/ImuPrePro.hpp"

namespace sensor {

ImuPrePro::ImuPrePro(
		math::Matrix3f& mat_UB,
		math::Vector3f& bias_B ):
				_mat_UB(mat_UB),
				_bias_B(bias_B)
{
}

ImuPrePro::~ImuPrePro() {
}

void ImuPrePro::preProcess(const math::Vector3f& dataRaw_U, math::Vector3f& data_B)
{
	// Then change frame / apply scale factor
	data_B = _mat_UB * dataRaw_U;

	// Compensate bias
	data_B -= _bias_B;
}

} /* namespace sensor */
