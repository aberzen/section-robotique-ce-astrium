/*
 * Led.h
 *
 *  Created on: 9 janv. 2013
 *      Author: Aberzen
 */

#ifndef LED_H_
#define LED_H_

#include <Arduino.h>
#include <stdint.h>

namespace arducopter {

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
	inline void _switchOn(void);

	/* @bried switch off the led w/o state update */
	inline void _switchOff(void);
};

/** @brief Get led state */
inline LedState Led::getState(void) {
	return this->_state;
}

/* @bried switch on the led w/o state update */
inline void Led::_switchOn(void) {
	/* Switch off the led (HIGH is the voltage level) */
	digitalWrite(this->_pin, (uint8_t)HIGH);
}

/* @bried switch off the led w/o state update */
inline void Led::_switchOff(void) {
	/* Switch off the led (LOW is the voltage level) */
	digitalWrite(this->_pin, (uint8_t)LOW);
}

} /* namespace arducopter */
#endif /* LED_H_ */
