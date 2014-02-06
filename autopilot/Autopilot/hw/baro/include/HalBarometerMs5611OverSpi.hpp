/*
 * HalBarometerMs5611OverSpi.hpp
 *
 *  Created on: 26 mars 2013
 *      Author: Aberzen
 */

#ifndef BAROMETERMS5611OVERSPI_HPP_
#define BAROMETERMS5611OVERSPI_HPP_

#include <hw/baro/include/HalBarometer.hpp>
#include <hw/spi/include/SpiSlave.hpp>

#define MS5611_ROM_SIZE	(6)

namespace hw {

class HalBarometerMs5611OverSpi : public HalBarometer {
public:
	typedef enum
	{
		T_COEFF_C1 = 0,
		T_COEFF_C2 = 1,
		T_COEFF_C3 = 2,
		T_COEFF_C4 = 3,
		T_COEFF_C5 = 4,
		T_COEFF_C6 = 5
	} T_COEFF;

protected:
	typedef enum
	{
		E_CONVERT_NONE = 0,
		E_CONVERT_D1,
		E_CONVERT_D2
	} T_CONVERT_STATE;

public:
	HalBarometerMs5611OverSpi(
			/* Inputs */
			SpiSlave& spi,
			/* Outputs */
			Output& out
			/* Parameters */
			);
	virtual ~HalBarometerMs5611OverSpi();

	/** @brief Initialize the sensor. */
	virtual void initialize();

	/** @brief Reset the sensor. */
	virtual void reset();

	/** @brief Execute the process */
	virtual void execute();

	/** @brief Get a coefficient */
	inline uint16_t getCoeff(T_COEFF coeffId);

protected:
	/** @brief Convert the raw measurements into nominal measurements */
	void convertRaw();

	/** @brief Initialize the sensor. */
	void readRom();

protected:
	/** Current convertion state */
	T_CONVERT_STATE _convState;

	/** @brief Spi slave gateway to access MS5611 chip interface */
	SpiSlave& _spi;

	/** @brief ROM correction values */
	uint16_t _coeffs[MS5611_ROM_SIZE];

	/** @brief Raw pressure */
	uint32_t _rawPressure;

	/** @brief Raw temperature */
	uint32_t _rawTemperature;
};

/** @brief Get a coefficient */
inline uint16_t HalBarometerMs5611OverSpi::getCoeff(T_COEFF coeffId)
{
	return _coeffs[coeffId];

}

} /* namespace hw */
#endif /* BAROMETERMS5611OVERSPI_HPP_ */
