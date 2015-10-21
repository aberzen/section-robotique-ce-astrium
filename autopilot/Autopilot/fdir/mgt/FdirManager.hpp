/*
 * FdirManager.hpp
 *
 *  Created on: 18 sept. 2015
 *      Author: AdministrateurLocal
 */

#ifndef FDIR_MGT_FDIRMANAGER_HPP_
#define FDIR_MGT_FDIRMANAGER_HPP_

#include <stdint.h>

namespace fdir {

class FdirManager {
public:
	FdirManager();
	virtual ~FdirManager();

	/** @brief Signal missing cycle */
	void signalMissingCycles(uint8_t missedCount);
};

} /* namespace fdir */

#endif /* FDIR_MGT_FDIRMANAGER_HPP_ */
