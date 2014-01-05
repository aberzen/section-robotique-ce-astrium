/*
 * Application.h
 *
 *  Created on: 8 janv. 2013
 *      Author: Aberzen
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_

#include <arch/msgBus/include/MessageListener.hpp>

namespace arducopter {

class Application : public arducopter::MessageListener {
public:
	Application();
	virtual ~Application() {
		// TODO Auto-generated destructor stub
	}
};

} /* namespace arducopter */
#endif /* APPLICATION_H_ */
