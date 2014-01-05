/*
 * HalImuMpu6000.hpp
 *
 *  Created on: 7 juil. 2013
 *      Author: Aberzen
 */

#ifndef HALIMUMPU6000_HPP_
#define HALIMUMPU6000_HPP_

#include "HalImu.hpp"
#include <hw/spi/include/SpiBus.hpp>
#include <hw/spi/include/SpiSlave.hpp>

namespace hw {

class HalImuMpu6000: public HalImu {
public:
	typedef enum {
		E_GYR_CNF_250DPS = 0x00,
		E_GYR_CNF_500DPS = 0x08,
		E_GYR_CNF_1000DPS = 0x10,
		E_GYR_CNF_2000DPS = 0x18
	} T_GYR_CNF;

	typedef enum {
		E_ACC_CNF_2G = 0,
		E_ACC_CNF_4G = 1,
		E_ACC_CNF_8G = 2,
		E_ACC_CNF_16G = 3
	} T_ACC_CNF;

	typedef enum {
		E_UPT_FREQ_1000HZ = 0x00,
		E_UPT_FREQ_500HZ = 0x01,
		E_UPT_FREQ_250HZ = 0x03,
		E_UPT_FREQ_200HZ = 0x04,
		E_UPT_FREQ_100HZ = 0x09,
		E_UPT_FREQ_50HZ = 0x13
	} T_UPT_FREQ;

	typedef struct {
		T_GYR_CNF gyrCnf;
		T_ACC_CNF accCnf;
		T_UPT_FREQ frequence;
	} Param;
public:
	HalImuMpu6000(
			/* Dependencies */
			SpiSlave& spiSlave,
			/* Inputs */
			/* Outputs */
			HalImu::Output& out,
			HalImu::RawOutput& rawOut,
			/* Parameters */
			const Param& param);
	virtual ~HalImuMpu6000();

	/** @brief Init the process */
	virtual infra::status initialize();

	/** @brief Execute the process */
	virtual infra::status execute();

	/** @brief Reset the process */
	virtual infra::status reset();

protected:
	/** @brief Write the register */
	void register_write(uint8_t reg, uint8_t val);

	/** @brief Read the register */
	uint8_t register_read( uint8_t reg );

protected:
	/** @brief Interrupt count */
	volatile uint8_t _intRdyCnt;

	/** @brief SPI slave */
	SpiSlave _spiSlave;

	/** @brief Parameters */
	const Param& _param;

    /** @brief Configured gyro LSB */
    float _gyrLsb;

	/** @brief Configured accel LSB */
    float _accLsb;

	/** @brief Product ID */
	uint8_t _productId;

protected:
	/** @brief Imu for interrupt handling */
	static HalImuMpu6000* _imu;
	/** @brief Handle data ready interrupt */
	static void handleReadyInterrupt(void);
};

} /* namespace hw */
#endif /* HALIMUMPU6000_HPP_ */
