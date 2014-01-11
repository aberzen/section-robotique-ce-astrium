/*
 * ProcDetectContact.hpp
 *
 *  Created on: 10 janv. 2014
 *      Author: Robotique
 */

#ifndef PROCDETECTCONTACT_HPP_
#define PROCDETECTCONTACT_HPP_

#include <arch/app/include/Procedure.hpp>
#include <autom/est/include/Estimator.hpp>
#include <autom/gen/include/GenericParameters.hpp>

namespace autom {

class ProcDetectContact : public infra::Procedure {
public:
	typedef struct
	{
		float detectThd;
		uint8_t filtDur;
	} Param;
	typedef struct
	{
		math::Vector3f forceErr_I;
		bool isLanded;
	} Output;
public:
	ProcDetectContact(
			/* Input */
			const Estimator::Estimations& estVal,
			const hw::HalImu::Output& imuMeas,
			const math::Vector3f& forceReal_B,
			/* Output */
			Output& out,
			/* Param */
			const Param& param,
			const GenericParam& paramGen
			);
	virtual ~ProcDetectContact();

	/** @brief Init the process */
	virtual ::infra::status initialize() ;

	/** @brief Execute the process */
	virtual ::infra::status execute() ;

protected:
	/** @brief Cycle count for which the threshold was crossed */
	uint8_t _count;

	/** @brief Estimation values */
	const Estimator::Estimations& _estVal;

	/** @brief Imu measurements */
	const hw::HalImu::Output& _imuMeas;

	/** @brief Force realized in body frame by the modulator */
	const math::Vector3f& _forceReal_B;

	/** @brief Outputs values */
	Output& _out;

	/** @brief Parameters */
	const Param& _param;

	/** @brief Generic parameters */
	const GenericParam& _paramGen;
};

} /* namespace autom */

#endif /* PROCDETECTCONTACT_HPP_ */
