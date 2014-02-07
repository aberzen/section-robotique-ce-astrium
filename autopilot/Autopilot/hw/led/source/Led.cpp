/*
 * Led.cpp
 *
 *  Created on: 9 janv. 2013
 *      Author: Aberzen
 */

#include <Arduino.h>
#include "../include/Led.hpp"


namespace arducopter {

Led::Led(uint8_t pin) :
	_pin(pin),
	_state(E_LED_OFF) {

	/* Initialize pin mode to OUTPUT */
	pinMode(this->_pin, (uint8_t) OUTPUT);

	/* Initialize the led to OFF */
	this->switchOff();
}

Led::~Led() {

}

/** @brief Switch on the led */
void Led::switchOn(void) {
	/* Switch on the led */
	this->_switchOn();

	/* Update the state */
	this->_state = E_LED_ON;
}

/** @brief Switch off the led */
void Led::switchOff(void) {
	/* Switch off the led */
	this->_switchOff();

	/* Update the state */
	this->_state = E_LED_OFF;
}

/** @brief Blink the led */
void Led::blink(void){
	/* Inverse led state */
	switch (this->_state) {
	case E_LED_ON:
		/* Switch off */
		this->_switchOff();
		/* Update the state */
		this->_state = E_LED_OFF;
		break;
	case E_LED_OFF:
	default:
		/* Switch on */
		this->_switchOn();
		/* Update the state */
		this->_state = E_LED_ON;
		break;
	}
}

/* @bried switch on the led w/o state update */
void Led::_switchOn(void) {
	/* Switch off the led (HIGH is the voltage level) */
	digitalWrite(this->_pin, (uint8_t)HIGH);
}

/* @bried switch off the led w/o state update */
void Led::_switchOff(void) {
	/* Switch off the led (LOW is the voltage level) */
	digitalWrite(this->_pin, (uint8_t)LOW);
}

} /* namespace arducopter */
