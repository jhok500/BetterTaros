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
    Real AgentX;
    Real AgentY;
    int hang;
    int hangF1;
    int hangF2;
    int hangF3;
    int hangF4;
    int hangF5;

    std::vector<Real> Agent0;
    //std::vector<Real> Agent0Ex;
    std::vector<Real> Agent1;
    //std::vector<Real> Agent1Ex;
    std::vector<Real> Agent2;
    //std::vector<Real> Agent2Ex;

    std::vector<Real> Agent;
    std::vector<Real> AgentCoord;
    std::vector<Real> AgentExt;

    std::vector<Real> Indices;
    std::vector<Real> Indices1;
    std::vector<Real> Indices2;
    std::vector<Real> Indices3;
    std::vector<Real> Indices4;
    std::ofstream Discrep3;
    std::ofstream FVout;
    std::ofstream FVout0;
    std::ofstream FVout1;
    std::ofstream FVout2;
    std::ofstream FVout3;
    std::ofstream FVout4;
    std::ofstream FVCoordout;
    std::ofstream FVCoordout0;
    std::ofstream FVCoordout1;
    std::ofstream FVCoordout2;
    std::ofstream FVCoordout3;
    std::ofstream FVCoordout4;
    int Snapshot = 0;
    boost::circular_buffer<Real>* IntCoord;
    boost::circular_buffer<Real>* ExtCoord;
    boost::circular_buffer<Real>* YawSensor;
    boost::circular_buffer<Real>* YawHolder;
    boost::circular_buffer<Real>* YawHolder1;
    boost::circular_buffer<int>* FeatureVectors3;
    boost::circular_buffer<int>* MemoryLog;
    Real CoordDistance;

    inline Real GetCoordDistanceFeature() const {

        for (int i = 0; i < Agent.size(); i++ ) {
            if (Agent[i] == -83) {
                return Agent[i+5];
            }
        }

    }











































private:


};

#endif //ARGOS3_EXAMPLES_LOGGING_LOOP_FUNCTIONS_H
