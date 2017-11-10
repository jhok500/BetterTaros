/*
 * RadioSensor.h
 *
 *  Created on: 27 Jan 2015
 *      Author: richard
 */

#ifndef CONTROLLERS_FOOTBOT_EXPLORING_RADIOSENSOR_H_
#define CONTROLLERS_FOOTBOT_EXPLORING_RADIOSENSOR_H_

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/config.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/datatypes/byte_array.h>
#include <argos3/core/utility/configuration/base_configurable_resource.h>
#include <argos3/core/simulator/entity/positional_entity.h>
#include <argos3/core/simulator/sensor.h>
#include <argos3/core/simulator/entity/composable_entity.h>

#include <queue>
#include <limits>

using namespace argos;

class RadioSensor : public CSimulatedSensor, public CCI_Sensor
{
public:
	constexpr static Real infinity = std::numeric_limits<Real>::infinity();

	RadioSensor();
	~RadioSensor();

	virtual void SetRobot(CComposableEntity& cEntity);
	virtual void SetPositionalEntity(CPositionalEntity& pEnt) { posEntity = &pEnt; }
	virtual void Init(TConfigurationNode& t_node);
	virtual void Reset();
	virtual void Destroy();
	virtual void Update();

	virtual int GetAddress();
	virtual void RecvData(CByteArray& recv){ mBuffer.push(recv); }
	virtual CVector3 GetLocation();

	virtual CByteArray GetNextMessage();

	bool operator<(RadioSensor& s){ return address < s.address; }
	bool operator>(RadioSensor& s){ return address > s.address; }
	bool operator==(RadioSensor& s){ return address == s.address; }

	virtual void CreateLuaState(lua_State* s){}
	virtual void ReadingsToLuaState(lua_State* s){}

private:
	int address = -1;
	CPositionalEntity* posEntity;
	std::queue<CByteArray> messages;
	std::queue<CByteArray> mBuffer;
};

#endif /* CONTROLLERS_FOOTBOT_EXPLORING_RADIOSENSOR_H_ */
