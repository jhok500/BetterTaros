#ifndef ARGOS3_EXAMPLES_LOGGING_LOOP_FUNCTIONS_H
#define ARGOS3_EXAMPLES_LOGGING_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
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
    int counter = 1;

private:


CBoxEntity* m_pcBox;











































private:


};

#endif //ARGOS3_EXAMPLES_LOGGING_LOOP_FUNCTIONS_H
