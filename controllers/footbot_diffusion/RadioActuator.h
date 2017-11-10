/*
 * RadioActuator.h
 *
 *  Created on: 27 Jan 2015
 *      Author: richard
 */

#ifndef CONTROLLERS_FOOTBOT_EXPLORING_RADIOACTUATOR_H_
#define CONTROLLERS_FOOTBOT_EXPLORING_RADIOACTUATOR_H_

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/datatypes/byte_array.h>
#include <argos3/core/simulator/entity/positional_entity.h>
#include <argos3/core/simulator/actuator.h>
#include <argos3/core/simulator/entity/composable_entity.h>

#include <limits>
#include <queue>

using namespace argos;

class RadioActuator : public CCI_Actuator, public CSimulatedActuator
{
public:
	constexpr static Real infinity = std::numeric_limits<Real>::infinity();

	RadioActuator() : posEnt(nullptr), transmitRange(infinity) {};
	~RadioActuator() {};

	virtual void SetPositionalEntity(CPositionalEntity& ent) { posEnt = &ent; }
	virtual void SetRobot(CComposableEntity& cEnt);
	virtual void Update();

	virtual void BroadcastData(CByteArray& transmit);

	virtual void Init(TConfigurationNode& node);

	virtual void CreateLuaState(lua_State* s) {}

	virtual CVector3 GetPosition();
	virtual Real GetRange() { return transmitRange; }
private:
	CPositionalEntity* posEnt;
	Real transmitRange;

	std::queue<CByteArray> transmitBuffer;
};

#endif /* CONTROLLERS_FOOTBOT_EXPLORING_RADIOACTUATOR_H_ */
