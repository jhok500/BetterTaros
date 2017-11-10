/*
 * RadioFieldSingleton.h
 *
 *  Created on: 27 Jan 2015
 *      Author: richard
 */

#ifndef CONTROLLERS_FOOTBOT_EXPLORING_RADIOFIELDSINGLETON_H_
#define CONTROLLERS_FOOTBOT_EXPLORING_RADIOFIELDSINGLETON_H_

#include <set>
#include <mutex>
#include <argos3/core/utility/datatypes/byte_array.h>

#include "RadioActuator.h"
#include "RadioSensor.h"

using namespace argos;

class RadioFieldSingleton
{
public:
	static RadioFieldSingleton& GetFieldInstance();

	void RegisterSensor(RadioSensor& sensor);
	void DeregisterSensor(RadioSensor& sensor);
	int SensorsInRange(RadioActuator& of);
	int TransmitData(RadioActuator& from, CByteArray& toTransmit);
	int GetNextFreeAddress();

private:
	RadioFieldSingleton();
	virtual ~RadioFieldSingleton();

	int nextAddress = 0;

	std::set<RadioSensor*> sensors;
	std::mutex instanceMutex;
};

#endif /* CONTROLLERS_FOOTBOT_EXPLORING_RADIOFIELDSINGLETON_H_ */
