/*
 * ParameterMgt.hpp
 *
 *  Created on: 5 juin 2013
 *      Author: Aberzen
 */

#ifndef PARAMETERMGT_HPP_
#define PARAMETERMGT_HPP_

#include <stdint.h>
//#include "AP_Param.hpp"
#include <mavlink_types.h>
#include <common/mavlink.h>

namespace mavlink {

class ParameterMgt {
protected:
	static const uint16_t eepromRefRevision;

public:

	typedef char param_name_t[MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN+1];
	typedef union {
		uint8_t UINT8;
		int8_t INT8;
		uint16_t UINT16;
		int16_t INT16;
		uint32_t UINT32;
		int32_t INT32;
	//		uint64_t _uint64_t; ==> unsupported / take too memory
	//		int64_t _int64_t; ==> unsupported / take too memory
		float REAL32;
	//		double _double; ==> unsupported / take too memory
	} Value;

	typedef struct
	{
		enum MAV_PARAM_TYPE type; // AP_PARAM_*
		const param_name_t name;
		void* currentValue;    // pointer to the variable in memory
		Value defaultValue;
	} ParamInfo;

	typedef struct
	{
		enum MAV_PARAM_TYPE type;
		param_name_t name;
		Value value;
		uint16_t idx;
	} Param;
protected:
	typedef struct
	{
		uint16_t revision; // Revision of Flash program for which this eeprom was formated
		uint16_t count; // Number of parameters currently saved in eeprom (to be compared with Flash)
		uint8_t crc; // CRC of data in EEPROM
	} Header;

public:
	ParameterMgt(const ParamInfo* info, uint16_t paramCount);
	virtual ~ParameterMgt();

	bool find(const char* name, uint16_t& idx);
	inline bool getInfo(uint16_t idx, enum MAV_PARAM_TYPE* type, char* name=NULL);

	/** @brief Read parameter value */
	bool read(uint16_t idx, Value& value);

	/** @brief Write parameter value */
	bool write(uint16_t idx, const Value& value);

	/** @brief Get parameter count */
	inline uint16_t getParamCount();

	// Check eeprom header (i.e. check revision, and CRC)
    bool checkEeprom();

    // Reset eeprom (i.e. reset header, values, and recompute the CRC)
    /* return the number of failed updated parameters */
    int16_t resetEeprom();

    // Reset all parameters value stored into eeprom with the default value of the flash
    // return the number of failed updated parameters
    int16_t resetToDefaultValues();

    // Load all values from eeprom
    /* return the number of failed updated parameters */
    int16_t loadAllValues();

    // Save all values to eeprom
    /* return the number of failed updated parameters */
    int16_t saveAllValues();

protected:
	const ParamInfo* infos;
	uint16_t paramCount;
                                                         // parameter's index
protected: /* STATIC METHODS */

	static bool copyValueToMemory(enum MAV_PARAM_TYPE type, const Value& value, void* addr);
	static bool copyValueFromMemory(enum MAV_PARAM_TYPE type, Value& value, const void* addr);

protected: /* REGULAR METHODS */

	bool getFullInfo(uint16_t idx, enum MAV_PARAM_TYPE* type, char* name=NULL, void** currentValue=NULL, Value* eepromValue=NULL);

	/* Save current value into EEPROM */
	bool save(uint16_t paramIdx, bool updateCrc = false);
	/* Load current value from EEPROM */
	bool load(uint16_t paramIdx);
	/* Load current value from program memory */
	bool loadDefault(uint16_t paramIdx);

    // Check flash revision compatibility
    bool checkRevision();

    // Check flash revision compatibility
    bool checkCount();

    // Check eeprom CRC
    bool checkCrc();


    // Reset header (i.e. recopy the resvision information, parameter count, and CRC)
    void resetHeader();

    // Reset all parameters value stored into eeprom with the default value of the flash
    uint8_t computeCrc();

    // Update the CRC corresponding to the replacement of previous parameter value with the new one */
	void updateParamCrc(Value oldEepromValue, Value newEepromValue);
};

inline bool ParameterMgt::getInfo(uint16_t idx, enum MAV_PARAM_TYPE* type, char* name)
{
	return getFullInfo(idx, type, name, NULL, NULL);
}
inline uint16_t ParameterMgt::getParamCount()
{
	return paramCount;
}

} /* namespace mavlink */
#endif /* PARAMETERMGT_HPP_ */
