#ifndef ARGOS3_EXAMPLES_LOGGING_LOOP_FUNCTIONS_H
#define ARGOS3_EXAMPLES_LOGGING_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/utility/math/angles.h>
#include <boost/circular_buffer.hpp>
#include <numeric>
#include <assert.h>
using namespace argos;
class LoggingLoopFunctions : public CLoopFunctions {

public:

    LoggingLoopFunctions();

    virtual ~LoggingLoopFunctions() {}


    virtual void Init(TConfigurationNode& t_tree);
    virtual void Reset();
    virtual void Destroy();
    virtual void PreStep();
    virtual void PostStep();
    Real AgentBearing_Sensor;

    std::vector<Real> Agent0;
    //std::vector<Real> Agent0Ex;
    std::vector<Real> Agent1;
    //std::vector<Real> Agent1Ex;
    std::vector<Real> Agent2;
    //std::vector<Real> Agent2Ex;

    std::vector<Real> Agent;
    std::vector<Real> AgentExt;
    std::vector<Real> Indices;
    std::vector<Real> Indices1;
    std::vector<Real> Indices2;
    std::vector<Real> Indices3;
    std::vector<Real> Indices4;


    boost::circular_buffer<Real>* IntCoord;
    boost::circular_buffer<Real>* ExtCoord;
    boost::circular_buffer<Real>* YawSensor;
    boost::circular_buffer<Real>* WheelDataCurrent;
    boost::circular_buffer<Real>* RABDataCurrent;
    boost::circular_buffer<Real>* IntYawCoord;
    boost::circular_buffer<Real>* YawHolder;
    boost::circular_buffer<Real>* ExtYawCoord;
    boost::circular_buffer<Real>* YawHolder1;
    boost::circular_buffer<int>* MidCurrent;
    boost::circular_buffer<int>* CloseCurrent;
    boost::circular_buffer<int>* ExMidCurrent;
    boost::circular_buffer<int>* ExCloseCurrent;


































    boost::circular_buffer<Real>* IntCoordCurrent0;
    boost::circular_buffer<Real>* ExtCoordCurrent01;
    boost::circular_buffer<Real>* ExtCoordCurrent02;
    boost::circular_buffer<Real>* WheelDataCurrent0;
    boost::circular_buffer<Real>* RABDataCurrent0;
    boost::circular_buffer<Real>* IntYawCoord0;
    boost::circular_buffer<Real>* ExtYawCoord01;
    boost::circular_buffer<Real>* ExtYawCoord02;
    boost::circular_buffer<int>* MidCurrent0;
    boost::circular_buffer<int>* CloseCurrent0;
    boost::circular_buffer<int>* ExMidCurrent01;
    boost::circular_buffer<int>* ExMidCurrent02;
    boost::circular_buffer<int>* ExCloseCurrent01;
    boost::circular_buffer<int>* ExCloseCurrent02;

    boost::circular_buffer<Real>* IntCoordCurrent1;
    boost::circular_buffer<Real>* ExtCoordCurrent10;
    boost::circular_buffer<Real>* ExtCoordCurrent12;
    boost::circular_buffer<Real>* WheelDataCurrent1;
    boost::circular_buffer<Real>* RABDataCurrent1;
    boost::circular_buffer<Real>* IntYawCoord1;
    boost::circular_buffer<Real>* ExtYawCoord10;
    boost::circular_buffer<Real>* ExtYawCoord12;
    boost::circular_buffer<int>* MidCurrent1;
    boost::circular_buffer<int>* CloseCurrent1;
    boost::circular_buffer<int>* ExMidCurrent10;
    boost::circular_buffer<int>* ExMidCurrent12;
    boost::circular_buffer<int>* ExCloseCurrent10;
    boost::circular_buffer<int>* ExCloseCurrent12;

    boost::circular_buffer<Real>* IntCoordCurrent2;
    boost::circular_buffer<Real>* ExtCoordCurrent20;
    boost::circular_buffer<Real>* ExtCoordCurrent21;
    boost::circular_buffer<Real>* WheelDataCurrent2;
    boost::circular_buffer<Real>* RABDataCurrent2;
    boost::circular_buffer<Real>* IntYawCoord2;
    boost::circular_buffer<Real>* ExtYawCoord20;
    boost::circular_buffer<Real>* ExtYawCoord21;
    boost::circular_buffer<int>* MidCurrent2;
    boost::circular_buffer<int>* CloseCurrent2;
    boost::circular_buffer<int>* ExMidCurrent20;
    boost::circular_buffer<int>* ExMidCurrent21;
    boost::circular_buffer<int>* ExCloseCurrent20;
    boost::circular_buffer<int>* ExCloseCurrent21;







private:


};

#endif //ARGOS3_EXAMPLES_LOGGING_LOOP_FUNCTIONS_H
