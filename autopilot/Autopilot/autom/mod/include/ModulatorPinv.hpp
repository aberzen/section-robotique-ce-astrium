/*
 * ModulatorPinv.hpp
 *
 *  Created on: 24 déc. 2013
 *      Author: Robotique
 */

#ifndef MODULATORPINV_HPP_
#define MODULATORPINV_HPP_

#include <autom/mod/include/Modulator.hpp>

namespace autom {

template <int8_t NB_MOTORS>
class ModulatorPinv : public ::autom::Modulator<NB_MOTORS> {
public:
	typedef struct {
		float pInvInfMat[NB_MOTORS][6];
		float descVect[NB_MOTORS];
	} ParamPinv ;
public:

	ModulatorPinv(
			/* Input */
			const ::math::Vector3f& torque_B,
			const ::math::Vector3f& force_B,
			/* Output */
			hw::Pwm::Input& out,
			::math::Vector3f& torqueReal_B,
			::math::Vector3f& forceReal_B,
			/* Param */
			typename ::autom::Modulator<NB_MOTORS>::ParamGen paramGen,
			const ParamPinv& paramPinv);
	virtual ~ModulatorPinv();

	/** @brief Execute the process */
	virtual ::infra::status execute();

protected:
	bool scaleLambda0();
	bool scaleLambda1();
protected:
	/** @brief Parameters */
	const ParamPinv& _paramPinv;
};


template <int8_t NB_MOTORS>
ModulatorPinv<NB_MOTORS>::ModulatorPinv(
		/* Input */
		const ::math::Vector3f& torque_B,
		const ::math::Vector3f& force_B,
		/* Output */
		hw::Pwm::Input& out,
		::math::Vector3f& torqueReal_B,
		::math::Vector3f& forceReal_B,
		/* Param */
		typename ::autom::Modulator<NB_MOTORS>::ParamGen paramGen,
		const ParamPinv& paramPinv)
: ::autom::Modulator<NB_MOTORS>::Modulator(torque_B, force_B, out, torqueReal_B, forceReal_B, paramGen),
  _paramPinv(paramPinv)
{
	// TODO Auto-generated constructor stub
}

template <int8_t NB_MOTORS>
ModulatorPinv<NB_MOTORS>::~ModulatorPinv() {
	// TODO Auto-generated destructor stub
}

/** @brief Execute the process */
template <int8_t NB_MOTORS>
::infra::status ModulatorPinv<NB_MOTORS>::execute()
{
	int8_t iMotor;

	/* 1) Compute the requested motor square of PWM ratio using pseudo inverse */
	for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
	{
		this->_sqRatios[iMotor] =   this->_torque_B.x * _paramPinv.pInvInfMat[iMotor][0]
				           + this->_torque_B.y * _paramPinv.pInvInfMat[iMotor][1]
						   + this->_torque_B.z * _paramPinv.pInvInfMat[iMotor][2]
						   + this->_force_B.x * _paramPinv.pInvInfMat[iMotor][3]
						   + this->_force_B.y * _paramPinv.pInvInfMat[iMotor][4]
						   + this->_force_B.z * _paramPinv.pInvInfMat[iMotor][5];

	}

	/* 2) Scale to have only positive values */
	bool success = scaleLambda0();
	if (success)
	{
		/* 3) Scale to have ratio inferior to 1. */
		success = scaleLambda1();
	}

	for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
	{
		/* 3) Compute the square root for each ratio */
		float ratio = sqrt(this->_sqRatios[iMotor]);

		/* 5) Convert into PWM*/
		this->_out.channels[iMotor] = ratio*(MAX_PULSEWIDTH-MIN_PULSEWIDTH)+MIN_PULSEWIDTH;
	}

	/* Update the realized torsor from upper class */
	return Modulator<NB_MOTORS>::execute();
}


template <int8_t NB_MOTORS>
bool ModulatorPinv<NB_MOTORS>::scaleLambda0()
{
	/* 1) Use descent vector to force square of the ratio to be positive */
	float lbdRes = 0.;
	float lbdPos = 0.;
	float lbdNeg = 0.;
	float lbdMax = 0.;
	bool isLbdMax = false;
	float lbdMin = 0.;
	bool isLbdMin = false;
	bool success = true;
	int8_t iMotor;

	for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
	{
		if (_paramPinv.descVect[iMotor] == 0. && this->_sqRatios[iMotor]<0.)
		{
			/* Not possible to force it to be positive */
			/* Error: bad descent vector definition */
			success = false;
		}
		else if (this->_sqRatios[iMotor] < 0.)
		{
			if (_paramPinv.descVect[iMotor] > 0.)
			{
				/* Compute positive lambda */
				lbdPos = math_max(lbdPos, - this->_sqRatios[iMotor] / _paramPinv.descVect[iMotor]);
			}
			else if (_paramPinv.descVect[iMotor] < 0.)
			{
				/* Compute negative lambda */
				lbdNeg = math_min(lbdNeg, - this->_sqRatios[iMotor] / _paramPinv.descVect[iMotor]);
			}
		}
		else
		{
			if (_paramPinv.descVect[iMotor] > 0.)
			{
				/* Define the smallest acceptable lambda */
				if (isLbdMin)
					lbdMin = math_max(lbdMin,- this->_sqRatios[iMotor] / _paramPinv.descVect[iMotor]);
				else
				{
					lbdMin = - this->_sqRatios[iMotor] / _paramPinv.descVect[iMotor];
					isLbdMin = true;
				}
			}
			else if (_paramPinv.descVect[iMotor] < 0.)
			{
				/* Define the greatest acceptable lambda */
				if (isLbdMax)
					lbdMax = math_min(lbdMax,- this->_sqRatios[iMotor] / _paramPinv.descVect[iMotor]);
				else
				{
					lbdMax = - this->_sqRatios[iMotor] / _paramPinv.descVect[iMotor];
					isLbdMax = true;
				}
			}
		}
	}

	/* Check computed values */
	if (success)
	{
		if (isLbdMin && isLbdMax && (lbdMax < lbdMin))
		{
			/* Error: bad descent vector definition */
			success = false;
		}
		else
		{
			if (lbdPos != 0. && lbdNeg != 0.)
			{
				/* Error: bad descent vector definition */
				success = false;
			}
			else if (lbdPos != 0.)
			{
				if (isLbdMin && (lbdPos < lbdMin))
				{
					/* Force to min */
					lbdRes = lbdMin;
				}
				else if (isLbdMax && (lbdPos > lbdMax))
				{
					/* Force to max */
					lbdRes = lbdMax;
				}
				else
				{
					lbdRes = lbdPos;
				}
			}
			else if (lbdNeg != 0.)
			{
				if (isLbdMin && (lbdNeg < lbdMin))
				{
					/* Force min */
					lbdRes = lbdMin;
				}
				else if (isLbdMax && (lbdNeg > lbdMax))
				{
					/* Force max */
					lbdRes = lbdMax;
				}
				else
				{
					lbdRes = lbdNeg;
				}
			}
		}
	}

	if (success)
	{
		/* There is a lambda */
		for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
		{
			this->_sqRatios[iMotor] += lbdRes * _paramPinv.descVect[iMotor];

			if (this->_sqRatios[iMotor]<0.)
				this->_sqRatios[iMotor] = 0.;

		}
	}
	else
	{
		/* We are in the shit */
		for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
		{
			this->_sqRatios[iMotor] = 0.;
		}
	}

	return success;
}

template <int8_t NB_MOTORS>
bool ModulatorPinv<NB_MOTORS>::scaleLambda1()
{
	/* 1) Use descent vector to force square of the ratio to be positive */
	float lbdRes = 0.;
	float lbdPos = 0.;
	float lbdNeg = 0.;
	float lbdMax = 0.;
	bool isLbdMax = false;
	float lbdMin = 0.;
	bool isLbdMin = false;
	bool success = true;
	int8_t iMotor;
	float maxSqrtRatio = 1.;

	for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
	{
		if (_paramPinv.descVect[iMotor] == 0. && this->_sqRatios[iMotor]>1.)
		{
			/* Not possible to force it to be inferior to 1 using lambda*/
			/* Error: bad descent vector definition */
			success = false;
		}
		else if (this->_sqRatios[iMotor] > 1.)
		{
			if (_paramPinv.descVect[iMotor] > 0.)
			{
				/* Compute negative lambda */
				lbdNeg = math_min(lbdNeg, (1. - this->_sqRatios[iMotor]) / _paramPinv.descVect[iMotor]);
			}
			else if (_paramPinv.descVect[iMotor] < 0.)
			{
				/* Compute positive lambda */
				lbdPos = math_max(lbdPos, (1. - this->_sqRatios[iMotor]) / _paramPinv.descVect[iMotor]);
			}
		}
		else
		{
			if (_paramPinv.descVect[iMotor] > 0.)
			{
				/* Define the smallest acceptable lambda */
				if (isLbdMin)
					lbdMin = math_max(lbdMin,- this->_sqRatios[iMotor] / _paramPinv.descVect[iMotor]);
				else
				{
					lbdMin = - this->_sqRatios[iMotor] / _paramPinv.descVect[iMotor];
					isLbdMin = true;
				}
				/* Define the greatest acceptable lambda */
				if (isLbdMax)
					lbdMax = math_min(lbdMax,(1.- this->_sqRatios[iMotor]) / _paramPinv.descVect[iMotor]);
				else
				{
					lbdMax = (1. - this->_sqRatios[iMotor]) / _paramPinv.descVect[iMotor];
					isLbdMax = true;
				}
			}
			else if (_paramPinv.descVect[iMotor] < 0.)
			{
				/* Define the smallest acceptable lambda */
				if (isLbdMin)
					lbdMin = math_max(lbdMin,(1. - this->_sqRatios[iMotor]) / _paramPinv.descVect[iMotor]);
				else
				{
					lbdMin = (1. - this->_sqRatios[iMotor]) / _paramPinv.descVect[iMotor];
					isLbdMin = true;
				}
				/* Define the greatest acceptable lambda */
				if (isLbdMax)
					lbdMax = math_min(lbdMax,- this->_sqRatios[iMotor] / _paramPinv.descVect[iMotor]);
				else
				{
					lbdMax = - this->_sqRatios[iMotor] / _paramPinv.descVect[iMotor];
					isLbdMax = true;
				}
			}
		}
	}

	/* Check computed values */
	if (success)
	{
		if (isLbdMin && isLbdMax && (lbdMax < lbdMin))
		{
			/* Error: bad descent vector definition */
			success = false;
		}
		else
		{
			if (lbdPos != 0. && lbdNeg != 0.)
			{
				/* Error: bad descent vector definition */
				success = false;
			}
			else if (lbdPos != 0.)
			{
				if (isLbdMin && (lbdPos < lbdMin))
				{
					/* Use the min */
					lbdRes = lbdMin;
				}
				else if (isLbdMax && (lbdPos > lbdMax))
				{
					/* Use the max */
					lbdRes = lbdMax;
				}
				else
				{
					lbdRes = lbdPos;
				}
			}
			else if (lbdNeg != 0.)
			{
				if (isLbdMin && (lbdNeg < lbdMin))
				{
					/* Use max */
					lbdRes = lbdMin;
				}
				else if (isLbdMax && (lbdNeg > lbdMax))
				{
					/* Use max */
					lbdRes = lbdMax;
				}
				else
				{
					lbdRes = lbdNeg;
				}
			}
		}
	}
//	Serial.printf("mod_2: success = %d\n",success);
	if (success)
	{
		/* There is a lambda */
		for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
		{
			this->_sqRatios[iMotor] += lbdRes * _paramPinv.descVect[iMotor];

			if (this->_sqRatios[iMotor]<0)
				this->_sqRatios[iMotor] = 0;

			maxSqrtRatio = math_max(maxSqrtRatio, this->_sqRatios[iMotor]);
		}

		if (maxSqrtRatio > 1.)
		{
			/* Scale down */
			for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
			{
				this->_sqRatios[iMotor] /= maxSqrtRatio;

				if (this->_sqRatios[iMotor]>1.)
					this->_sqRatios[iMotor] = 1.;

			}

		}
	}
	else
	{
		/* We are in the shit */
		for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
		{
			this->_sqRatios[iMotor] = 0.;
		}
	}

	return 0;
}

} /* namespace autom */

#endif /* MODULATORPINV_HPP_ */
