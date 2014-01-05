/*
 * ParameterMgt.cpp
 *
 *  Created on: 5 juin 2013
 *      Author: Aberzen
 */

#include <math.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include "../include/ParameterMgt.hpp"
#include <gcs/include/Channel.hpp>

namespace mavlink {

ParameterMgt::ParameterMgt(const ParamInfo* infos, uint8_t paramCount) :
		infos(infos),
		paramCount(paramCount)
{
	/* Check Eeprom */
	if (!checkEeprom())
		resetEeprom();
}

ParameterMgt::~ParameterMgt() {

}

bool ParameterMgt::read(uint8_t idx, Value& value)
{
	/* Get type */
	return getFullInfo(idx, NULL, NULL, NULL, &value);
}

bool ParameterMgt::write(uint8_t idx, const Value& value)
{
	enum MAV_PARAM_TYPE type;
	void* currentValue;

	/* Get type and ptr to current value */
	if (!getFullInfo(idx, &type, NULL, &currentValue))
			return false;

	/* Update memory value */
	copyValueToMemory(type, value, currentValue);

	/* Save memory value in EEPROM while updating CRC */
	return save(idx,true);
}


bool ParameterMgt::find(const char* name, uint8_t& idx)
{
	for(idx=0 ; idx<paramCount ; idx++)
	{
		if (0==strcmp_P(name,infos[idx].name))
		{
			/* Found */
			return true;
			break;
		}
	}
	idx = 0;
	return false;
}

bool ParameterMgt::getFullInfo(uint8_t idx, enum MAV_PARAM_TYPE* type, char* name, void** currentValue, Value* eepromValue)
{
	if (idx<paramCount)
	{
		if (name != NULL)
			/* Copy type from progmem */
			strcpy_P(&name[0],&infos[idx].name[0]);

		if (currentValue != NULL)
			/* Copy pointer from progmem */
			memcpy_P(currentValue,&infos[idx].currentValue,sizeof(void *));

		if (eepromValue != NULL)
			/* Copy pointer from progmem */
			eeprom_read_block(eepromValue,(void*)(sizeof(Header)+idx*sizeof(Value)),sizeof(Value));

		if (type != NULL)
			/* Copy type from progmem */
			memcpy_P(type,&infos[idx].type,sizeof(enum MAV_PARAM_TYPE));

		return true;
	}
	return false;
}

bool ParameterMgt::save(uint8_t paramIdx, bool updateCrc)
{
	const ParamInfo* info;
	Value newEepromValue;
	Value oldEepromValue;
	enum MAV_PARAM_TYPE type;
	void* value;
	if (paramIdx < paramCount)
	{
		info = &infos[paramIdx];
		/* Read type from program memory*/
		memcpy_P(&type, &info->type, sizeof(enum MAV_PARAM_TYPE));
		/* Read pointer from program memory*/
		memcpy_P(&value, &info->currentValue, sizeof(void*));
		/* Get value from SRAM */
		if (!ParameterMgt::copyValueFromMemory(type, newEepromValue, value))
			return false;
		if (updateCrc)
		{
			/* Read previous value from program memory */
			eeprom_read_block(&oldEepromValue, (void*)(sizeof(Header)+paramIdx*sizeof(Value)), sizeof(Value));
			/* Then update CRC */
			updateParamCrc(oldEepromValue, newEepromValue);
		}
		/* Write value into eeprom */
		eeprom_write_block(&newEepromValue, (void*)(sizeof(Header)+paramIdx*sizeof(Value)), sizeof(Value));
		return true;
	}
	return false;
}

bool ParameterMgt::load(uint8_t paramIdx)
{
	const ParamInfo* info;
	Value eepromValue;
	enum MAV_PARAM_TYPE type;
	void* value;
	if (paramIdx < paramCount)
	{
		info = &infos[paramIdx];
		/* Read default value from program memory */
		eeprom_read_block(&eepromValue, (void*)(sizeof(Header)+paramIdx*sizeof(Value)), sizeof(Value));
		/* Read type from program memory*/
		memcpy_P(&type, &info->type, sizeof(enum MAV_PARAM_TYPE));
		/* Read pointer from program memory*/
		memcpy_P(&value, &info->currentValue, sizeof(void*));
		/* Update */
		return ParameterMgt::copyValueToMemory(type, eepromValue, value);
	}
	return false;
}

bool ParameterMgt::loadDefault(uint8_t paramIdx)
{
	const ParamInfo* info;
	Value defaultValue;
	enum MAV_PARAM_TYPE type;
	void* value;
	if (paramIdx < paramCount)
	{
		info = &infos[paramIdx];
		/* Read default value from program memory */
		memcpy_P(&defaultValue, &(info->defaultValue), sizeof(Value));
		/* Read type from program memory*/
		memcpy_P(&type, &(info->type), sizeof(enum MAV_PARAM_TYPE));
		/* Read pointer from program memory*/
		memcpy_P(&value, &(info->currentValue), sizeof(void*));
		/* Update */
		return ParameterMgt::copyValueToMemory(type, defaultValue, value);
	}
	return false;
}



// Check eeprom header (i.e. check revision, and CRC)
bool ParameterMgt::checkEeprom()
{
	// Check revision
	if (!checkRevision())
		return false;

	// Check parameter count
	if (!checkCount())
		return false;

	// Check CRC
	if (!checkCrc())
		return false;

	// else all is Ok
	return true;
}


// Check flash revision compatibility
bool ParameterMgt::checkRevision()
{
	Header header;
	/* Read header */
	eeprom_read_block(&header, 0, sizeof(Header));
	/* Compare to reference revision */
	return (header.revision == eepromRefRevision );
}

// Check flash revision compatibility
bool ParameterMgt::checkCount()
{
	Header header;
	/* Read header */
	eeprom_read_block(&header, 0, sizeof(Header));
	/* Compare to reference revision */
	return (header.count == paramCount);
}


//// Check eeprom CRC
bool ParameterMgt::checkCrc()
{
	Header header;
	uint16_t crc = 0;
	/* Read header */
	eeprom_read_block(&header, 0, sizeof(Header));
	/* Compute CRC */
	crc = computeCrc();
	/* Compare CRC */
	return (header.crc == crc);
}


// Reset all parameters value stored into eeprom with the default value of the flash
int16_t ParameterMgt::resetToDefaultValues()
{
	uint8_t paramIdx;
	uint8_t succesCount=0;
	for (paramIdx=0 ; paramIdx < paramCount ; paramIdx++)
	{
		/* Load default value */
		if (loadDefault(paramIdx))
			/* Save into EEPROM */
			if (save(paramIdx))
				succesCount++;
	}
	return succesCount-paramCount;
}


// Reset eeprom (i.e. reset header, values, and recompute the CRC)
int16_t ParameterMgt::resetEeprom()
{
	int16_t status;
	/* Reset values to default values */
	status = resetToDefaultValues();
	/* First reset header (but not the CRC) */
	resetHeader();
	/* return the status */
	return status;
}


// Reset header (i.e. recopy the revision information, parameter count, and CRC)
void ParameterMgt::resetHeader()
{
	Header header;
	header.count = paramCount;
	header.revision = ParameterMgt::eepromRefRevision;
	header.crc = computeCrc();

	/* Read default value from program memory */
	eeprom_write_block(&header, (void*)(NULL), sizeof(Header));
}

// Load all values from eeprom
int16_t ParameterMgt::loadAllValues()
{
	uint8_t paramIdx;
	uint8_t succesCount=0;
	/* For each Parameter, load the value */
	for (paramIdx=0 ; paramIdx < paramCount ; paramIdx++)
	{
		if (load(paramIdx))
			succesCount++;
	}
	return succesCount-paramCount;
}

// Save all values to eeprom
int16_t ParameterMgt::saveAllValues()
{
	uint8_t paramIdx;
	uint8_t succesCount=0;
	/* For each Parameter, load the value */
	for (paramIdx=0 ; paramIdx < paramCount ; paramIdx++)
	{
		if (save(paramIdx))
			succesCount++;
	}
	return succesCount-paramCount;
}

bool ParameterMgt::copyValueToMemory(enum MAV_PARAM_TYPE type, const Value& value, void* addr)
{
	/* Write default value into RAM */
	switch (type)
	{
	case MAV_PARAM_TYPE_UINT8:
		(*((uint8_t*) addr)) = value.UINT8;
		break;
	case MAV_PARAM_TYPE_INT8:
		(*((int8_t*) addr)) = value.INT8;
		break;
	case MAV_PARAM_TYPE_UINT16:
		(*((uint16_t*) addr)) = value.UINT16;
		break;
	case MAV_PARAM_TYPE_INT16:
		(*((int16_t*) addr)) = value.INT16;
		break;
	case MAV_PARAM_TYPE_UINT32:
		(*((uint32_t*) addr)) = value.UINT32;
		break;
	case MAV_PARAM_TYPE_INT32:
		(*((int32_t*) addr)) = value.INT32;
		break;
	case MAV_PARAM_TYPE_REAL32:
		(*((float*) addr)) = value.REAL32;
		break;
	case MAV_PARAM_TYPE_UINT64:
	case MAV_PARAM_TYPE_INT64:
	case MAV_PARAM_TYPE_REAL64:
	default:
		/* Not supported */
		return false;
		break;
	}
	return true;
}

bool ParameterMgt::copyValueFromMemory(enum MAV_PARAM_TYPE type, Value& value, const void* addr)
{
	/* Write default value into RAM */
	switch (type)
	{
	case MAV_PARAM_TYPE_UINT8:
		value.UINT8 = (*((uint8_t*) addr));
		break;
	case MAV_PARAM_TYPE_INT8:
		value.INT8 = (*((int8_t*) addr));
		break;
	case MAV_PARAM_TYPE_UINT16:
		value.UINT16 = (*((uint16_t*) addr));
		break;
	case MAV_PARAM_TYPE_INT16:
		value.INT16 = (*((int16_t*) addr));
		break;
	case MAV_PARAM_TYPE_UINT32:
		value.UINT32 = (*((uint32_t*) addr));
		break;
	case MAV_PARAM_TYPE_INT32:
		value.INT32 = (*((int32_t*) addr));
		break;
	case MAV_PARAM_TYPE_REAL32:
		value.REAL32 = (*((float*) addr));
		break;
	case MAV_PARAM_TYPE_UINT64:
	case MAV_PARAM_TYPE_INT64:
	case MAV_PARAM_TYPE_REAL64:
	default:
		/* Not supported */
		memset(&value, 0, sizeof(Value));
		return false;
		break;
	}
	return true;
}

// Update the CRC corresponding to the replacement of previous parameter value with the new one */
void ParameterMgt::updateParamCrc(Value oldEepromValue, Value newEepromValue)
{
	Header header;
	uint8_t* oldVal;
	uint8_t* newVal;
	uint8_t count;

	/* Read header */
	eeprom_read_block(&header, 0, sizeof(Header));

	count = sizeof(Value);
	oldVal = (uint8_t*)&oldEepromValue;
	newVal = (uint8_t*)&newEepromValue;
	/* Update the CRC by substracting CRC of previous value and adding CRC of new value */
	while(count--)
		header.crc += *(newVal++) - *(oldVal++);

	/* Write header */
	eeprom_write_block(&header, 0, sizeof(Header));
}

// Reset all parameters value stored into eeprom with the default value of the flash
uint8_t ParameterMgt::computeCrc()
{
	uint8_t crc = 0;
	uint8_t* addr = (uint8_t*) sizeof(Header);
	uint16_t count = paramCount*sizeof(Value);
	/* Compute the CRC as the sum of byte of all parameters values */
	while(count--)
	{
		crc += eeprom_read_byte(addr++);
	}

	return crc;
}



} /* namespace mavlink */
