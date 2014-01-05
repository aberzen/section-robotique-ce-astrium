/*
 * System.hpp
 *
 *  Created on: 24 juil. 2013
 *      Author: Aberzen
 */

#ifndef SYSTEM_HPP_
#define SYSTEM_HPP_

#include <arch/app/include/Process.hpp>
#include <board/gen/include/Board.hpp>
#include <gcs/include/MavServiceManager.hpp>
//#include <autom/est/include/AhrsEstimator.hpp>
#include <autom/mgt/include/Ancs.hpp>

namespace system {

class System : public infra::Process {
public:
	static System& system;

public:
	System(board::Board& board);
	virtual ~System();

	/** @brief Init the process */
	virtual infra::status initialize();

	/** @brief Execute the process */
	virtual infra::status execute();

	/** @brief Get house keeping */
	inline mavlink::ParameterMgt& getParameterMgt();

	/** @brief Get house keeping */
	inline mavlink::HouseKeepingMgt& getMavHouseKeeping();

	/** @brief Get GCS */
	inline mavlink::MavServiceManager& getMavSvcMgr();

	/** @brief Estimator */
//	virtual inline autom::Estimator& getEstimator();

	/** @brief Get board */
	inline board::Board& getBoard();

	autom::Ancs _mgt;

protected:
	/** @brief Board */
	board::Board& _board;


	/** @brief Mode management */
	mavlink::ModeMgt _modeMgt;

	/** @brief Parameter management */
	mavlink::ParameterMgt _paramMgt;

	/** @brief Waypoint management */
	mavlink::WaypointMgt _wpMgt;

	/** @brief House keeping management */
	mavlink::HouseKeepingMgt _hkMgt;

	/** @brief Command management */
	mavlink::CommandMgt _cmdMgt;

	/** @brief Mount management */
	mavlink::MountMgt _mountMgt;

	/** @brief Fence management */
	mavlink::FenceMgt _fenceMgt;

	/** @brief Digicam Management */
	mavlink::DigivcamMgt _digivcamMgt;

	/** @brief Mavlink services manager */
	mavlink::MavServiceManager _mavSvcMgt;

	/** @brief Estimator */
//	autom::AhrsEstimator _est;


};

/** @brief Get board */
inline board::Board& System::getBoard()
{
	return _board;
}

/** @brief Get house keeping */
inline mavlink::HouseKeepingMgt& System::getMavHouseKeeping()
{
	return _hkMgt;
}

/** @brief Get GCS */
inline mavlink::MavServiceManager& System::getMavSvcMgr()
{
	return _mavSvcMgt;
}

///** @brief Estimator */
//inline autom::Estimator& System::getEstimator()
//{
//	return _est;
//}

/** @brief Get house keeping */
inline mavlink::ParameterMgt& System::getParameterMgt()
{
	return _paramMgt;
}

} /* namespace system */
#endif /* SYSTEM_HPP_ */
