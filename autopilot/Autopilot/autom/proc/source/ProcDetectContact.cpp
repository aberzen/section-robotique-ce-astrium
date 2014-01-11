/*
 * ProcDetectContact.cpp
 *
 *  Created on: 10 janv. 2014
 *      Author: Robotique
 */

#include <autom/proc/include/ProcDetectContact.hpp>

namespace autom {

ProcDetectContact::ProcDetectContact(
		/* Input */
		const Estimator::Estimations& estVal,
		const hw::HalImu::Output& imuMeas,
		const math::Vector3f& forceReal_B,
		/* Output */
		Output& out,
		/* Param */
		const Param& param,
		const GenericParam& paramGen
		)
: infra::Procedure::Procedure(),
  _count(0),
  _estVal(estVal),
  _imuMeas(imuMeas),
  _forceReal_B(forceReal_B),
  _out(out),
  _param(param),
  _paramGen(paramGen)
{

}

ProcDetectContact::~ProcDetectContact() {
	// TODO Auto-generated destructor stub
}

/** @brief Init the process */
::infra::status ProcDetectContact::initialize()
{
	_count = 0;

	/* Assume that initialization is made on ground */
	_out.isLanded = false;

	return 0;
}

/** @brief Execute the process */
::infra::status ProcDetectContact::execute()
{
	math::Vector3f frcGrav_I(0.,0.,-9.81*_paramGen.mass);
	_out.forceErr_I = _estVal.attitude_IB.rotateQVQconj(_imuMeas.accoMeas_B*_paramGen.mass - _forceReal_B) + frcGrav_I;
	if (_out.isLanded)
	{
		/* Check if the contact with ground is lost */
		if (math_abs(_out.forceErr_I.z)<_param.detectThd)
		{
			_count++;
			if (_count>=_param.filtDur)
			{
				_out.isLanded = false;
				_count = 0;
			}
		}
	}
	else
	{
		/* Check if there is a contact with ground */
		float errZ = math_abs(_out.forceErr_I.z - frcGrav_I.z);
		if (errZ<_param.detectThd)
		{
			_count++;
			if (_count>=_param.filtDur)
			{
				_out.isLanded = true;
				_count = 0;
			}
		}
	}
	return 0;
}

} /* namespace autom */
