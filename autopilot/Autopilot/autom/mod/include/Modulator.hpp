/*
 * Modulator.hpp
 *
 *  Created on: 24 déc. 2013
 *      Author: Robotique
 */

#ifndef MODULATOR_HPP_
#define MODULATOR_HPP_

#include <arch/app/include/Process.hpp>
#include <hw/pwm/include/Pwm.hpp>
#include <math/include/Vector3.hpp>

namespace autom {

template <int8_t NB_MOTORS>
class Modulator : public infra::Process {
public:
	typedef struct {
		float infMat[6][NB_MOTORS];
		float pInvInfMat[NB_MOTORS][6];
		float descVect[NB_MOTORS];
	} Param ;
public:

	Modulator(
			/* Input */
			const ::math::Vector3f& torque_B,
			const ::math::Vector3f& force_B,
			/* Output */
			hw::Pwm::Input& out,
			/* Param */
			const Param& param);
	virtual ~Modulator();

	/** @brief Init the process */
	virtual ::infra::status initialize();

	/** @brief Execute the process */
	virtual ::infra::status execute();

protected:
	/** @brief Parameters */
	const Param& _param;

	/** @brief Demanded torque */
	const ::math::Vector3f& _torque_B;

	/** @brief Demanded force */
	const ::math::Vector3f& _force_B;

	/** @brief Pwm */
	hw::Pwm::Input& _out;
};


template <int8_t NB_MOTORS>
Modulator<NB_MOTORS>::Modulator(
		/* Input */
		const ::math::Vector3f& torque_B,
		const ::math::Vector3f& force_B,
		/* Output */
		hw::Pwm::Input& out,
		/* Param */
		const Modulator<NB_MOTORS>::Param& param)
: Process(),
  _param(param),
  _torque_B(torque_B),
  _force_B(force_B),
  _out(out)
{
	// TODO Auto-generated constructor stub

}

template <int8_t NB_MOTORS>
Modulator<NB_MOTORS>::~Modulator() {
	// TODO Auto-generated destructor stub
}


/** @brief Init the process */
template <int8_t NB_MOTORS>
::infra::status Modulator<NB_MOTORS>::initialize()
{
	uint8_t iMotor;

	for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
	{
		_out.channels[iMotor] = MIN_PULSEWIDTH;
		board::Board::board.getPwm().enable_out(iMotor);
	}

	return 0;
}

/** @brief Execute the process */
template <int8_t NB_MOTORS>
::infra::status Modulator<NB_MOTORS>::execute()
{
	float sqRatios[NB_MOTORS];
	float ratios[NB_MOTORS];
	int8_t iMotor;
	float maxSqrtRatio = 1.;

	/* 1) Compute the requested motor square of PWM ratio using pseudo inverse */
	for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
	{
		sqRatios[iMotor] =   _torque_B.x * _param.pInvInfMat[iMotor][0]
				           + _torque_B.y * _param.pInvInfMat[iMotor][1]
						   + _torque_B.z * _param.pInvInfMat[iMotor][2]
						   + _force_B.x * _param.pInvInfMat[iMotor][3]
						   + _force_B.y * _param.pInvInfMat[iMotor][4]
						   + _force_B.z * _param.pInvInfMat[iMotor][5];

//		Serial.printf("mod_1: sqRatios[%d] = %.5f\n", iMotor, sqRatios[iMotor]);
	}

	/* 2) Use descent vector to force square of the ratio to be positive */
	float lbdRes = 0.;
	float lbdPos = 0.;
	float lbdNeg = 0.;
	float lbdMax = 0.;
	bool isLbdMax = false;
	float lbdMin = 0.;
	bool isLbdMin = false;
	bool error = false;
	for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
	{
		if (_param.descVect[iMotor] == 0. && sqRatios[iMotor]<0.)
		{
			/* Not possible to force it to be positive */
			error = true;
			break;
		}
		else if (sqRatios[iMotor] < 0.)
		{
			if (_param.descVect[iMotor] > 0.)
			{
				/* Compute positive lambda */
				lbdPos = math_max(lbdPos, - sqRatios[iMotor] / _param.descVect[iMotor]);
			}
			else if (_param.descVect[iMotor] < 0.)
			{
				/* Compute negative lambda */
				lbdNeg = math_min(lbdPos, - sqRatios[iMotor] / _param.descVect[iMotor]);
			}
		}
		else
		{
			if (_param.descVect[iMotor] > 0.)
			{
				/* Define the smallest acceptable lambda */
				lbdMin = math_max(lbdMin,- sqRatios[iMotor] / _param.descVect[iMotor]);
				isLbdMin = true;
			}
			else if (_param.descVect[iMotor] < 0.)
			{
				/* Define the greatest acceptable lambda */
				lbdMax = math_min(lbdMax,- sqRatios[iMotor] / _param.descVect[iMotor]);
				isLbdMax = true;
			}
		}
	}
//	Serial.printf("mod_1: error = %d\n",error);
	/* Check computed values */
	if (!error)
	{
		if (lbdPos != 0. && lbdNeg != 0.)
		{
			/* We are in the shit */
			error = true;
		}
		else if (lbdPos != 0.)
		{
			if ((isLbdMin && (lbdPos < lbdMin)) || (isLbdMax && (lbdPos > lbdMax)))
			{
				/* We are in the shit */
				error = true;
			}
			else
			{
				lbdRes = lbdPos;
			}
		}
		else if (lbdNeg != 0.)
		{
			if ((isLbdMin && (lbdNeg < lbdMin)) || (isLbdMax && (lbdNeg > lbdMax)))
			{
				/* We are in the shit */
				error = true;
			}
			else
			{
				lbdRes = lbdNeg;
			}
		}
	}
//	Serial.printf("mod_2: error = %d\n",error);
	if (!error)
	{
		/* There is a lambda */
//		Serial.printf("mod_2: lbdRes = %.5f\n", lbdRes);
		for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
		{
			sqRatios[iMotor] += lbdRes * _param.descVect[iMotor];
			maxSqrtRatio = math_max(maxSqrtRatio, sqRatios[iMotor]);
//			Serial.printf("mod_2: sqRatios[%d] = %.5f\n", iMotor, sqRatios[iMotor]);
		}
	}
	else
	{
		/* We are in the shit */
		for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
		{
			sqRatios[iMotor] = 0.;
		}
	}

//	Serial.printf("mod_3: maxSqrtRatio = %.5f\n", maxSqrtRatio);
	for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
	{
		/* 3) Saturate to for the square ratios to be between 0 and 1 */
		sqRatios[iMotor] /= maxSqrtRatio;
//		Serial.printf("mod_3: sqRatios[%d] = %.5f\n", iMotor, sqRatios[iMotor]);

		/* 4) Compute the square root for each ratio */
		ratios[iMotor] = sqrt(sqRatios[iMotor]);

		/* 5) Convert into PWM*/
		_out.channels[iMotor] = ratios[iMotor]*(MAX_PULSEWIDTH-MIN_PULSEWIDTH)+MIN_PULSEWIDTH;
//		Serial.printf("mod_3: _out.channels[%d] = %d\n", iMotor, _out.channels[iMotor]);
	}

	return 0;

}

} /* namespace autom */

#endif /* MODULATOR_HPP_ */
