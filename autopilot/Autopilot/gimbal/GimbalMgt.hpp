/*
 * GimbalMgt.hpp
 *
 *  Created on: 29 janv. 2014
 *      Author: Robotique
 */

#ifndef GIMBALMGT_HPP_
#define GIMBALMGT_HPP_

#include <math/Quaternion.hpp>

namespace autom {

class GimbalMgt {
public:
	GimbalMgt(
			/* Input */
			/* Output */
			/* Param */
			);
	virtual ~GimbalMgt();

	/** @brief Init the process */
	virtual void initialize() ;

	/** @brief Execute the process */
	virtual void orientate(
			const math::Quaternion& quatAtt_IB,
			const math::Quaternion& quatAtt_BT) ;
};

} /* namespace autom */

#endif /* GIMBALMGT_HPP_ */
