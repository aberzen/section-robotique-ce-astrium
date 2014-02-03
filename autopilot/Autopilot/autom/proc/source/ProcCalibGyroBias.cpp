/*
 * ProcCalibGyroBias.cpp
 *
 *  Created on: 26 déc. 2013
 *      Author: Robotique
 */

#include <hw/serial/include/FastSerial.hpp>
#include <autom/proc/include/ProcCalibGyroBias.hpp>

namespace autom {

ProcCalibGyroBias::ProcCalibGyroBias(
		/* Input */
		const hw::HalImu::Output& imu,
		/* Output */
		autom::Estimator::Estimations& est,
		/* Param */
		const ProcCalibGyroBias::Param& param
	)
: _param(param),
  _imu(imu),
  _filtGyroX(_param.filtCoeffNum, _param.filtCoeffDen),
  _filtGyroY(_param.filtCoeffNum, _param.filtCoeffDen),
  _filtGyroZ(_param.filtCoeffNum, _param.filtCoeffDen),
  _filtAccoX(_param.filtCoeffNum, _param.filtCoeffDen),
  _filtAccoY(_param.filtCoeffNum, _param.filtCoeffDen),
  _filtAccoZ(_param.filtCoeffNum, _param.filtCoeffDen),
  _est(est),
  _state(E_PROCCALIBIMU_COMP_BIAS)
{
}

ProcCalibGyroBias::~ProcCalibGyroBias() {
	// TODO Auto-generated destructor stub
}

/** @brief Start procedure */
void ProcCalibGyroBias::start()
{
	switch(_state)
	{
	case E_PROCCALIBIMU_OFF:
		_sumGyroMeas(0., 0., 0.);
		_sumAccoMeas(0., 0., 0.);
		_count = _param.biasNbMeas;
		_state = E_PROCCALIBIMU_INIT;
		break;
	case E_PROCCALIBIMU_INIT:
	case E_PROCCALIBIMU_ENDED:
	case E_PROCCALIBIMU_FAILED:
	case E_PROCCALIBIMU_COMP_BIAS:
	case E_PROCCALIBIMU_COMP_VAR:
	default:
		break;
	}
}

/** @brief On tick procedure */
void ProcCalibGyroBias::onTick()
{
	switch(_state)
	{
	case E_PROCCALIBIMU_INIT:
		onTickInitFilters();
		break;
	case E_PROCCALIBIMU_COMP_BIAS:
		onTickCompBias();
		break;
	case E_PROCCALIBIMU_COMP_VAR:
		onTickCompVar();
		break;
	case E_PROCCALIBIMU_OFF:
	case E_PROCCALIBIMU_ENDED:
	case E_PROCCALIBIMU_FAILED:
	default:
		break;
	}
}

/** @brief Stop procedure */
void ProcCalibGyroBias::stop()
{
	_state = E_PROCCALIBIMU_OFF;
}

/** @brief Initialize the filters from first measurements */
void ProcCalibGyroBias::onTickInitFilters()
{
	if (_imu.isAvailable)
	{
		float init[2];

		init[0] = _imu.gyroMeas_B.x;
		init[1] = _imu.gyroMeas_B.x;
		_filtGyroX.reset(init, init);

		init[0] = _imu.gyroMeas_B.y;
		init[1] = _imu.gyroMeas_B.y;
		_filtGyroY.reset(init, init);

		init[0] = _imu.gyroMeas_B.z;
		init[1] = _imu.gyroMeas_B.z;
		_filtGyroZ.reset(init, init);

		init[0] = _imu.accoMeas_B.x;
		init[1] = _imu.accoMeas_B.x;
		_filtAccoX.reset(init, init);

		init[0] = _imu.accoMeas_B.y;
		init[1] = _imu.accoMeas_B.y;
		_filtAccoY.reset(init, init);

		init[0] = _imu.accoMeas_B.z;
		init[1] = _imu.accoMeas_B.z;
		_filtAccoZ.reset(init, init);

		_state = E_PROCCALIBIMU_COMP_BIAS;
	}
}

/** @brief Compute bias */
void ProcCalibGyroBias::onTickCompBias()
{
	if (_imu.isAvailable)
	{
		_est.imuGyroBias_B.x = _filtGyroX.apply(_imu.gyroMeas_B.x);
		_est.imuGyroBias_B.y = _filtGyroY.apply(_imu.gyroMeas_B.y);
		_est.imuGyroBias_B.z = _filtGyroZ.apply(_imu.gyroMeas_B.z);

		_est.imuAccoBias_B.x = _filtAccoX.apply(_imu.accoMeas_B.x);
		_est.imuAccoBias_B.y = _filtAccoY.apply(_imu.accoMeas_B.y);
		_est.imuAccoBias_B.z = _filtAccoZ.apply(_imu.accoMeas_B.z);

		if (_count == 0)
		{
			/* The bias computation step is over */
//			Serial.printf("imuGyroBias_B = {%.5f %.5f %.5f}\n",_est.imuGyroBias_B.x,_est.imuGyroBias_B.y,_est.imuGyroBias_B.z);
//			Serial.printf("imuAccoBias_B = {%.5f %.5f %.5f}\n",_est.imuAccoBias_B.x,_est.imuAccoBias_B.y,_est.imuAccoBias_B.z);

			/* Reset count and measurement */
			_count = _param.varNbMeas;
			_sumGyroMeas(0.,0.,0.);
			_sumAccoMeas(0.,0.,0.);

			/* and switch to variance verification step */
			this->_state = E_PROCCALIBIMU_COMP_VAR;
		}
		else
		{
			/* Decrement count, there still some step in this state */
			_count--;
		}
	}
}

/** @brief Compute variance */
void ProcCalibGyroBias::onTickCompVar()
{
	if (_imu.isAvailable)
	{
		_sumGyroMeas.x += (_est.imuGyroBias_B.x - _imu.gyroMeas_B.x)*(_est.imuGyroBias_B.x - _imu.gyroMeas_B.x);
		_sumGyroMeas.y += (_est.imuGyroBias_B.y - _imu.gyroMeas_B.y)*(_est.imuGyroBias_B.y - _imu.gyroMeas_B.y);
		_sumGyroMeas.z += (_est.imuGyroBias_B.z - _imu.gyroMeas_B.z)*(_est.imuGyroBias_B.z - _imu.gyroMeas_B.z);

		_sumAccoMeas.x += (_est.imuAccoBias_B.x - _imu.accoMeas_B.x)*(_est.imuAccoBias_B.x - _imu.accoMeas_B.x);
		_sumAccoMeas.y += (_est.imuAccoBias_B.y - _imu.accoMeas_B.y)*(_est.imuAccoBias_B.y - _imu.accoMeas_B.y);
		_sumAccoMeas.z += (_est.imuAccoBias_B.z - _imu.accoMeas_B.z)*(_est.imuAccoBias_B.z - _imu.accoMeas_B.z);

		if (_count == 0)
		{
			/* The variance computation step is over */
			/* Compute the variance as the mean of the measurements */
			_sumGyroMeas = _sumGyroMeas / (float)_param.varNbMeas;
			_sumAccoMeas = _sumAccoMeas / (float)_param.varNbMeas;

//			Serial.printf("varGyro = {%.5f %.5f %.5f}\n",_sumGyroMeas.x,_sumGyroMeas.y,_sumGyroMeas.z);
//			Serial.printf("varAcco = {%.5f %.5f %.5f}\n",_sumAccoMeas.x,_sumAccoMeas.y,_sumAccoMeas.z);

			if ( (_sumGyroMeas.x > _param.gyroVarianceThd) ||
				 (_sumGyroMeas.y > _param.gyroVarianceThd) ||
				 (_sumGyroMeas.z > _param.gyroVarianceThd) ||
				 (_sumAccoMeas.x > _param.accoVarianceThd) ||
				 (_sumAccoMeas.y > _param.accoVarianceThd) ||
				 (_sumAccoMeas.z > _param.accoVarianceThd) )
			{
				_state = E_PROCCALIBIMU_FAILED;
			}
			else
			{
				/* Remove gravity */
				_est.imuAccoBias_B -= _est.imuAccoBias_B*(9.81/_est.imuAccoBias_B.norm());

				/* Procedure is over */
				_state = E_PROCCALIBIMU_ENDED;
			}
		}
		else
		{
			/* Decrement count, there still some step in this state */
			_count--;
		}
	}
}

} /* namespace autom */
