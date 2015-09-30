/*
 * AttitudeManager.cpp
 *
 *  Created on: 31 août 2015
 *      Author: AdministrateurLocal
 */

//#include <system/system/System.hpp>
#include <att/mgt/AttitudeManager.hpp>

namespace attitude {

AttitudeManager::AttitudeManager(const AttitudeManager::Parameter& param)
: _guidMgt(),
  _ctrl(param.ctrl)
{
}

AttitudeManager::~AttitudeManager()
{
}


/** @brief periodic execution */
void AttitudeManager::execute()
{
	/* Execute the guidance */
	_guidMgt.execute();

	/* Execute the controller */
	_ctrl.execute();

//	{
//		char message[100];
//		sprintf(message, "ctrlAttAngErrB={%.3e,{%.3e,%.3e,%.3e}}\n",
//				system::system.dataPool.ctrlAttAngErrB.scalar,
//				system::system.dataPool.ctrlAttAngErrB.vector.x,
//				system::system.dataPool.ctrlAttAngErrB.vector.y,
//				system::system.dataPool.ctrlAttAngErrB.vector.z);
//		mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_INFO, message);
//		sprintf(message, "ctrlErrAttB={%d,%d,%d}\n",
//				ctrlErrAttB.x,
//				ctrlErrAttB.y,
//				ctrlErrAttB.z);
//		mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_INFO, message);
//		sprintf(message, "crlAttRateErrB={%d,%d,%d}\n",
//				crlAttRateErrB.x,
//				crlAttRateErrB.y,
//				crlAttRateErrB.z);
//		mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_INFO, message);
//		sprintf(message, "ctrlTrqDemB={%d,%d,%d}\n",
//				dataPool.ctrlTrqDemB.x,
//				dataPool.ctrlTrqDemB.y,
//				dataPool.ctrlTrqDemB.z);
//		mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_INFO, message);
//	}

}

} /* namespace attitude */
