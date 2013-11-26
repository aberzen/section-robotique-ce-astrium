/*
 * EstimatorStream.hpp
 *
 *  Created on: 2 août 2013
 *      Author: Aberzen
 */

#ifndef ESTIMATORSTREAM_HPP_
#define ESTIMATORSTREAM_HPP_

#include "DataStream.hpp"
#include <math/include/Quaternion.hpp>

namespace mavlink {

class EstimatorStream : public DataStream {
public:
	EstimatorStream(mavlink_channel_t port);
	virtual ~EstimatorStream();

protected:
	/** @brief Sample the data */
	virtual void sampleData();

	/** @brief Send data */
	virtual void sendData();

protected:
	/** @brief Attitude quaternion */
	math::Quaternion _q_BI;

	/** @brief Rate in body frame */
	math::Vector3f _rate_B;
};

} /* namespace mavlink */
#endif /* ESTIMATORSTREAM_HPP_ */
