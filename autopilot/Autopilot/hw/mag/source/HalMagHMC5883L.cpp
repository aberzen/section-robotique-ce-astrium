/*
 * HalMagHMC5883L.cpp
 *
 *  Created on: 6 août 2013
 *      Author: Aberzen
 */

#include <infra/include/Task.hpp>
#include "../include/HalMagHMC5883L.hpp"
#include <config/include/nrd.h>


namespace hw {


#define I2C_DEV_ADD_HMC5883L	0x1E

#define REG_ADDR_CRA			0x00 /**< @brief Config Register A */
#define REG_ADDR_CRB			0x01 /**< @brief Config Register B */
#define REG_ADDR_MR				0x02 /**< @brief Mode Register */
#define REG_ADDR_DORXM			0x03 /**< @brief Data Output Register X MSB */
#define REG_ADDR_DORXL			0x04 /**< @brief Data Output Register X LSB */
#define REG_ADDR_DORYM			0x05 /**< @brief Data Output Register Y MSB */
#define REG_ADDR_DORYL			0x06 /**< @brief Data Output Register Y LSB */
#define REG_ADDR_DORZM			0x07 /**< @brief Data Output Register Z MSB */
#define REG_ADDR_DORZL			0x08 /**< @brief Data Output Register Z LSB */
#define REG_ADDR_SR				0x09 /**< @brief Status Register */
#define REG_ADDR_IRA			0x0A /**< @brief Id Register A */
#define REG_ADDR_IRB			0x0B /**< @brief Id Register B */
#define REG_ADDR_IRC			0x0C /**< @brief Id Register C */

#define REG_VAL_CRA_MA_AV1		(0x00<<5) /**< @brief Average 1 sample */
#define REG_VAL_CRA_MA_AV2		(0x01<<5) /**< @brief Average 2 sample */
#define REG_VAL_CRA_MA_AV4		(0x02<<5) /**< @brief Average 4 sample */
#define REG_VAL_CRA_MA_AV8		(0x03<<5) /**< @brief Average 8 sample */

#define REG_VAL_CRA_DO_0_75HZ	(0x00<<2) /**< @brief Sample at 0.75Hz */
#define REG_VAL_CRA_DO_1_50HZ	(0x01<<2) /**< @brief Sample at 1.5Hz */
#define REG_VAL_CRA_DO_3_00HZ	(0x02<<2) /**< @brief Sample at 3Hz */
#define REG_VAL_CRA_DO_7_50HZ	(0x03<<2) /**< @brief Sample at 7.5Hz */
#define REG_VAL_CRA_DO_15HZ		(0x04<<2) /**< @brief Sample at 15Hz */
#define REG_VAL_CRA_DO_30HZ		(0x05<<2) /**< @brief Sample at 30Hz */
#define REG_VAL_CRA_DO_75HZ		(0x06<<2) /**< @brief Sample at 75Hz */

#define REG_VAL_CRA_MS_NM		(0x00<<0) /**< @brief Normal Mode */
#define REG_VAL_CRA_MS_PB		(0x01<<0) /**< @brief Positive bias */
#define REG_VAL_CRA_MS_NB		(0x02<<0) /**< @brief Negative bias */

#define REG_VAL_CRB_GN_0_88GA	(0x00<<5) /**< @brief Range set to +/- 0.88 Gauss */
#define REG_VAL_CRB_GN_1_3GA	(0x01<<5) /**< @brief Range set to +/- 1.3 Gauss */
#define REG_VAL_CRB_GN_1_9GA	(0x02<<5) /**< @brief Range set to +/- 1.9 Gauss */
#define REG_VAL_CRB_GN_2_5GA	(0x03<<5) /**< @brief Range set to +/- 2.5 Gauss */
#define REG_VAL_CRB_GN_4_0GA	(0x04<<5) /**< @brief Range set to +/- 4.0 Gauss */
#define REG_VAL_CRB_GN_4_7GA	(0x05<<5) /**< @brief Range set to +/- 4.7 Gauss */
#define REG_VAL_CRB_GN_5_6GA	(0x06<<5) /**< @brief Range set to +/- 5.6 Gauss */
#define REG_VAL_CRB_GN_8_1GA	(0x07<<5) /**< @brief Range set to +/- 8.1 Gauss */

#define REG_VAL_MR_MD_CM		(0x00<<0) /**< @brief Continuous mode */
#define REG_VAL_MR_MD_SM		(0x01<<0) /**< @brief Single measurement mode */
#define REG_VAL_MR_MD_IDLE		(0x02<<0) /**< @brief Idle mode */


HalMagHMC5883L::HalMagHMC5883L(I2C& bus) :
	HalMagnetometer(),
	_bus(bus)
{
}

HalMagHMC5883L::~HalMagHMC5883L()
{

}


/** @brief Read a register value */
bool HalMagHMC5883L::read_register(uint8_t address, uint8_t* value)
{
    if (_bus.read((uint8_t)I2C_DEV_ADD_HMC5883L, address, 1, value) != 0) {
//        healthy = false;
        return false;
    }
    return true;
}

/** @brief Write a value in a register */
bool HalMagHMC5883L::write_register(uint8_t address, uint8_t value)
{
    if (_bus.write((uint8_t)I2C_DEV_ADD_HMC5883L, address, value) != 0) {
//        healthy = false;
        return false;
    }
    return true;
}

/** @brief Initialize the HW */
status HalMagHMC5883L::initialize()
{
	if (0>HalMagnetometer::initialize())
		return -1;

    /* Configuration is:
     * - CRA:
     * 		- Sampling average: 1
     * 		- Rate: N/A ==> Single measurement mode
     * 		- Config: normal
     * 	- CRB:
     * 		- gain: 1.3Gauss?
     * 	- MR:
     * 		- mode: Single Measurement ==> performed in commandSample
     */
    if (!write_register(REG_ADDR_CRA, REG_VAL_CRA_MA_AV1 | REG_VAL_CRA_MS_NM))
    {
        return -2;
    }
    if (!write_register(REG_ADDR_CRB, REG_VAL_CRB_GN_1_3GA))
    {
        return -3;
    }

    /* Program next sample */
    if (0>commandSample())
    	return -4;

    return 0;
}

/** @brief Reset the HW */
status HalMagHMC5883L::reset()
{
	if (0>HalMagnetometer::reset())
		return -1;

	initialize();
	return 0;
}

/** @brief Execute the driver */
status HalMagHMC5883L::execute()
{
	/* Read data */
	if (0>readRaw())
	{
		_isAvailable = false;
		return -1;
	}

	/* Scale data */
	scaleRaw();

	/* Command a new read */
	if (0>commandSample())
		return -2;


	return 0;
}

/** @brief Program a measurement */
status HalMagHMC5883L::commandSample()
{
    if (!write_register(REG_ADDR_MR, REG_VAL_MR_MD_SM))
    {
        return -1;
    }
    return 0;
}

/** @brief Read raw measurement */
status HalMagHMC5883L::readRaw()
{
    uint8_t buff[6];
    uint8_t regValue;

    /* Check if data is ready */
    if (!read_register(REG_ADDR_SR, &regValue))
    {
    	return -1;
    }

    /* Check that status is ready */
    if (!(regValue & 0x01))
    {
    	return -2;
    }

    /* Read data */
    if (_bus.read(I2C_DEV_ADD_HMC5883L, REG_ADDR_DORXM, 6, buff) != 0)
	{
        return -3;
    }

    _rawMagFieldMeas.x = (int16_t)(buff[0] << 8) | buff[1];
    _rawMagFieldMeas.y = (int16_t)(buff[2] << 8) | buff[3];
    _rawMagFieldMeas.z = (int16_t)(buff[4] << 8) | buff[5];

    return 0;
}


/** @brief Scale raw measurement into SI */
void HalMagHMC5883L::scaleRaw()
{
	_magFieldMeas.x = _rawMagFieldMeas.x * 1.3/65536.;
	_magFieldMeas.y = _rawMagFieldMeas.y * 1.3/65536.;
	_magFieldMeas.z = _rawMagFieldMeas.z * 1.3/65536.;
}



} /* namespace hw */
