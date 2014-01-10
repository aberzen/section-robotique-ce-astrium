/*
 * ProcCompassDeclination.cpp
 *
 *  Created on: 30 déc. 2013
 *      Author: Robotique
 */

#include <autom/proc/include/ProcCompassDeclination.hpp>
#include <hw/serial/include/FastSerial.hpp>

namespace autom {

ProcCompassDeclination::ProcCompassDeclination(
		const board::Board::Measurements& meas,
		autom::Estimator::Estimations& est,
		ProcCompassDeclination::Output& out,
		const ProcCompassDeclination::Param& param
		)
: Procedure(),
  _meas(meas),
  _est(est),
  _out(out),
  _param(param),
  _filtX(_param.filtCoeffNum, _param.filtCoeffDen),
  _count(param.nbMeas)
{
	// TODO Auto-generated constructor stub

}

ProcCompassDeclination::~ProcCompassDeclination() {
	// TODO Auto-generated destructor stub
}

/** @brief Init the process */
infra::status ProcCompassDeclination::initialize()
{
	_count = _param.nbMeas;
}

/** @brief Execute current procedure step */
infra::status ProcCompassDeclination::step()
{
	float yaw = 0.;
	if (_meas.imu.isAvailable && _meas.compass.isAvailable)
	{
		/* Only consider when both measurements are available */
		::math::Vector3f accoMeasDir_B = (_meas.imu.accoMeas_B - _est.imuAccoBias_B);
		accoMeasDir_B.normalize(3,1./9.81);
		::math::Vector3f tmp = (accoMeasDir_B % (_meas.compass.magMeas_B % accoMeasDir_B));
		_out.invNrm = 1./_filtX.apply(tmp.norm());
		tmp *= _out.invNrm;
		::math::Vector3f x_I(1.,0.,0.);
		yaw = atan2((tmp%x_I).norm(),tmp*x_I);
		if (_count != 0)
		{
			_count --;
		}
		else
		{
			/* Use yaw to initialize the attitude */
			::math::Vector3f z(0.,0.,1.);
			z *= sin(yaw*0.5);
			_est.attitude_IB(cos(yaw*0.5),z);

			/* Procedure is over */
			_status = E_PROC_STATUS_TERMINATED;
		}
	}
	return 0;
}


} /* namespace autom */
