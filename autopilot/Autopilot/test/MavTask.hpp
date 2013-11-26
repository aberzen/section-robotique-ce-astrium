/*
 * MavTask.hpp
 *
 *  Created on: 26 juin 2013
 *      Author: Aberzen
 */

#ifndef MAVTASK_HPP_
#define MAVTASK_HPP_

#include <infra/include/Task.hpp>
#include <gcs/include/MavServiceManager.hpp>

namespace test {

extern mavlink::HouseKeepingMgt* hkMgt;

class MavTask : public infra::Task{
public:
	MavTask(const signed char * pcName,
            unsigned portSHORT usStackDepth,
            unsigned portBASE_TYPE uxPriority);
	virtual ~MavTask();

	/** @brief Non returning function executed as the task body function. */
	virtual void init(void);

	/** @brief Non returning function executed as the task body function. */
	virtual void run(void);

protected:
//	mavlink::ModeMgt _modeMgt;
//	mavlink::ParameterMgt _paramMgt;
//	mavlink::WaypointMgt _wpMgt;
//	mavlink::HouseKeepingMgt _hkMgt;
//	mavlink::CommandMgt _cmdMgt;
//	mavlink::MountMgt _mountMgt;
//	mavlink::FenceMgt _fenceMgt;
//	mavlink::DigivcamMgt _digivcamMgt;
//	mavlink::MavServiceManager _mavSvcMgt;

};

} /* namespace test */
#endif /* MAVTASK_HPP_ */
