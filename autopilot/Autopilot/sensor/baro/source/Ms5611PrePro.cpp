/*
 * Ms5611PrePro.cpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#include <string.h>
#include "../include/Ms5611PrePro.hpp"

namespace sensor {

Ms5611PrePro::Ms5611PrePro(hw::HalBarometerMs5611OverSpi& hal) :
	_hal(hal)
{
}

Ms5611PrePro::~Ms5611PrePro() {
}

void Ms5611PrePro::preProcess(
		const uint32_t& rawPressure,
		const uint32_t& rawTemperature,
		float& pressure,
		int16_t& temperature)
{
    int32_t dT = 0;
    int32_t TEMP = 0;
    int64_t OFF = 0;
    int64_t SENS = 0;
    int32_t T2 = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;
    int64_t tmp = 0;

    dT =
    		((int64_t)rawTemperature)
    		- ((int64_t)(((uint64_t)(_hal.getCoeff(hw::HalBarometerMs5611OverSpi::T_COEFF_C5)))<<8));
    OFF =
    		(((int64_t)(_hal.getCoeff(hw::HalBarometerMs5611OverSpi::T_COEFF_C2)))<<16)
    		+ ((((int64_t)(_hal.getCoeff(hw::HalBarometerMs5611OverSpi::T_COEFF_C4)))*((int64_t)dT))>>7);
    SENS =
    		(((int64_t)(_hal.getCoeff(hw::HalBarometerMs5611OverSpi::T_COEFF_C1)))<<15)
    		+ ((((int64_t)(_hal.getCoeff(hw::HalBarometerMs5611OverSpi::T_COEFF_C3)))*((int64_t)dT))>>8);

    TEMP = ((int64_t)((dT*((int64_t)(_hal.getCoeff(hw::HalBarometerMs5611OverSpi::T_COEFF_C6))))>>23));

    if (TEMP < 0)
    {
    	T2 = ((int64_t)(((int64_t)dT)*((int64_t)dT)))>>31;
    	OFF2 = (((int64_t)5)*((int64_t)(((int64_t)TEMP)*((int64_t)TEMP))))>>1;
    	SENS2 = OFF2>>1;

    	if (TEMP < -3500)
    	{
    		tmp = TEMP+((int32_t)3500);
    		tmp *= tmp;
    		OFF2 += 7*tmp;
    		SENS2 += 11*(tmp>>1);
    	}
    }

    TEMP += ((int32_t) 2000);
    TEMP -= ((int32_t) T2);
    OFF -= OFF2;
    SENS -= SENS2;

    pressure = (((((int64_t)rawPressure)*((int64_t)SENS)>>21)-OFF)>>15) / 100.;
    temperature = (int16_t) TEMP;
}

} /* namespace sensor */
