/*
 * HalBarometerMs5611OverSpi.cpp
 *
 *  Created on: 26 mars 2013
 *      Author: Aberzen
 */

#include <FreeRTOS.h>
#include <task.h>
#include <infra/include/Task.hpp>
#include <hw/baro/include/HalBarometerMs5611OverSpi.hpp>

#define CMD_MS5611_RESET 0x1E
#define CMD_MS5611_PROM_Setup 0xA0
#define CMD_MS5611_PROM_C1 0xA2
#define CMD_MS5611_PROM_C2 0xA4
#define CMD_MS5611_PROM_C3 0xA6
#define CMD_MS5611_PROM_C4 0xA8
#define CMD_MS5611_PROM_C5 0xAA
#define CMD_MS5611_PROM_C6 0xAC
#define CMD_MS5611_PROM_CRC 0xAE
#define CMD_CONVERT_D1_OSR4096 0x48   // Maximum resolution (oversampling)
#define CMD_CONVERT_D2_OSR4096 0x58   // Maximum resolution (oversampling)

#define MS5611_IDX_C1 0
#define MS5611_IDX_C2 1
#define MS5611_IDX_C3 2
#define MS5611_IDX_C4 3
#define MS5611_IDX_C5 4
#define MS5611_IDX_C6 5

namespace hw {

HalBarometerMs5611OverSpi::HalBarometerMs5611OverSpi(
		/* Inputs */
		SpiSlave& spi,
		/* Outputs */
		Output& out
		/* Parameters */
) :
HalBarometer(out),
_spi(spi),
_rawPressure(0),
_rawTemperature(0)
{
}

HalBarometerMs5611OverSpi::~HalBarometerMs5611OverSpi() {

}

/** @brief Initialize the sensor. */
void HalBarometerMs5611OverSpi::initialize()
{
	int8_t result;

	/* Call super */
	HalBarometer::initialize();

	/* Reset chip */
	reset();

	/* Read ROM */
	readRom();

	/* Reserve device */
	if (this->_spi.select(1))
	{
		/* Program D2 convertion */
		_spi.transfer0(CMD_CONVERT_D2_OSR4096);
		_convState = E_CONVERT_D2;

		/* Release device */
		this->_spi.release();
	}
}


/** @brief Reset the sensor. */
void HalBarometerMs5611OverSpi::reset()
{
	infra::status result;
	/* Call super */
	HalBarometer::reset();

	/* Reserve device */
	if (_spi.select(1))
	{
		/* Reset device */
		_spi.transfer0(CMD_MS5611_RESET);

		/* Release device */
		_spi.release();

		/* Wait at least 2.8ms */
		infra::Task::delay(((3*configTICK_RATE_HZ)/1000)+1);
	}
}

/** @brief Execute the process */
void HalBarometerMs5611OverSpi::execute()
{
	/* Reserve device */
	if (_spi.select(1))
	{
		/* Alternate D1 / D2 conversion */
		switch (_convState)
		{
		case E_CONVERT_D1:

			/* Read _D1 */
			_spi.transfer24(0, _rawPressure);

			/* Next state to D2 */
			_spi.transfer0(CMD_CONVERT_D2_OSR4096);
			_convState = E_CONVERT_D2;
			convertRaw();
			_out.isAvailable =  true;
			break;


		case E_CONVERT_D2:

			/* Read _D1 */
			_spi.transfer24(0, _rawTemperature);

			/* Next state to D1 */
			_spi.transfer0(CMD_CONVERT_D1_OSR4096);
			_convState = E_CONVERT_D1;
			_out.isAvailable =  false;
			break;
		default:
			/* Error unknown state (-2) */
			return;
			break;
		}
		/* Release the spi */
		_spi.release();
	}
}

/** @brief Initialize the sensor. */
void HalBarometerMs5611OverSpi::readRom()
{
	uint16_t crc = 0;

	/* Reserve device */
	if (this->_spi.select(1))
	{

		/* Read ROM Coefficients */
		_spi.transfer16((uint8_t)CMD_MS5611_PROM_C1, _coeffs[T_COEFF_C1], true);
		_spi.transfer16((uint8_t)CMD_MS5611_PROM_C2, _coeffs[T_COEFF_C2], true);
		_spi.transfer16((uint8_t)CMD_MS5611_PROM_C3, _coeffs[T_COEFF_C3], true);
		_spi.transfer16((uint8_t)CMD_MS5611_PROM_C4, _coeffs[T_COEFF_C4], true);
		_spi.transfer16((uint8_t)CMD_MS5611_PROM_C5, _coeffs[T_COEFF_C5], true);
		_spi.transfer16((uint8_t)CMD_MS5611_PROM_C6, _coeffs[T_COEFF_C6], true);

		/* Read ROM CRC */
		_spi.transfer16((uint8_t)CMD_MS5611_PROM_CRC, crc, true);

		// TODO: implement CRC checksum

		this->_spi.release();
	}
}

void HalBarometerMs5611OverSpi::convertRaw()
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
    		((int64_t)_rawTemperature)
    		- ((int64_t)(((uint64_t)(_coeffs[T_COEFF_C5]))<<8));
    OFF =
    		(((int64_t)(_coeffs[T_COEFF_C2]))<<16)
    		+ ((((int64_t)(_coeffs[T_COEFF_C4]))*((int64_t)dT))>>7);
    SENS =
    		(((int64_t)(_coeffs[T_COEFF_C1]))<<15)
    		+ ((((int64_t)(_coeffs[T_COEFF_C3]))*((int64_t)dT))>>8);

    TEMP = ((int64_t)((dT*((int64_t)(_coeffs[T_COEFF_C6])))>>23));

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

    _out.pressure = (((((int64_t)_rawPressure)*((int64_t)SENS)>>21)-OFF)>>15) / 100.;
    _out.temperature = (int16_t) TEMP;
}



} /* namespace hw */
