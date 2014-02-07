/*
 * Led.h
 *
 *  Created on: 9 janv. 2013
 *      Author: Aberzen
 */

#ifndef LED_H_
#define LED_H_

#include <stdint.h>

namespace hw {

typedef enum ELedState{
	E_LED_ON=0,
	E_LED_OFF
} LedState;

class Led {
public:
	Led(uint8_t pin);
	virtual ~Led();

	/** @brief Switch on the led */
	void switchOn(void);

	/** @brief Switch off the led */
	void switchOff(void);

	/** @brief Blink the led */
	void blink(void);

	/** @brief Get led state
	 * @return The led state
	 */
	inline LedState getState(void);

protected:
	uint8_t _pin;
	LedState _state;

private:
	/* @bried switch on the led w/o state update */
	void _switchOn(void);

	/* @bried switch off the led w/o state update */
	void _switchOff(void);
};

/** @brief Get led state */
inline LedState Led::getState(void) {
	return this->_state;
}

} /* namespace hw */
#endif /* LED_H_ */
