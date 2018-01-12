#include "logging_loop_functions.h"

#include <controllers/footbot_diffusion/footbot_diffusion.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include <boost/utility/binary.hpp>
#include <iostream>
#include <fstream>
#include <sstream>

//using namespace std;
/****************************************/
/****************************************/


CVector2 cPosInternalA;


LoggingLoopFunctions::LoggingLoopFunctions() :
m_pcBox(NULL)
{
}

/****************************************/
/****************************************/

void LoggingLoopFunctions::Init(TConfigurationNode& t_node) {
    try {

    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
    }
}

/****************************************/
/****************************************/

void LoggingLoopFunctions::Reset() {

}

/****************************************/
/****************************************/

void LoggingLoopFunctions::Destroy() {

}



/****************************************/
/****************************************/

void LoggingLoopFunctions::PreStep() {

    //bool 0Eyes1 = false;
    //std::cout << "Rolling" << std::endl;
    CSpace::TMapPerType &m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    CVector2 NeighbourDistance;
    Real Counter = -1;



    for (CSpace::TMapPerType::iterator it = m_cFootbots.begin();

         it != m_cFootbots.end();
         ++it) {
        /* Get handle to foot-bot entity and controller */


        CFootBotEntity &cFootBot = *any_cast<CFootBotEntity *>(it->second);
        CFootBotDiffusion &cController = dynamic_cast<CFootBotDiffusion &>(cFootBot.GetControllableEntity().GetController());
        //CFootBotDiffusion &cWheels = dynamic_cast<CFootBotDiffusion &>(cFootBot.GetControllableEntity().GetRootEntity());
        /* Get the position of the foot-bot on the ground as a CVector2 */
        //if (cFootBot.GetRootEntity().GetId().c_str() == std::string("fb0")) {

        CRadians yaw, pitch, roll;
        CQuaternion orientation_quaternion = cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation;
        orientation_quaternion.ToEulerAngles(yaw, pitch, roll);
        CCI_DifferentialSteeringSensor::SReading WheelData = cController.GetWheelVelocity();
        Real Velocity_Wheels =
                0.5 * (cController.GetSpoofLeftWheelVelocity() + cController.GetSpoofRightWheelVelocity());
        Real Difference_Wheels = cController.GetSpoofLeftWheelVelocity() - cController.GetSpoofRightWheelVelocity();
        CCI_RangeAndBearingSensor::TReadings packets = cController.GetRABSensorReadings();


        std::string OriginID = cFootBot.GetRootEntity().GetId().c_str();
        int RealID = OriginID.at(2) + 32;
        CVector3 RandPos = {1,1,0};
        CQuaternion Ori = {1,0,0,0};
        CVector3 Size = {0.3,0.3,0.5};
        //m_pcBox = new CBoxEntity("box", RandPos, Ori, false, Size, 10000.0f);

        //std::cout << CSimulator::GetInstance().GetSpace().GetEntitiesByType("light").size() << std::endl;

        /*if (GetSpace().GetSimulationClock() > 500 && CSimulator::GetInstance().GetSpace().GetEntitiesByType("box").size() == 4) {
            CallEntityOperation<CSpaceOperationAddEntity, CSpace, void>(CSimulator::GetInstance().GetSpace(), *m_pcBox);

            AddEntity(*m_pcBox);
        }*/

        /*if (CSimulator::GetInstance().GetSpace().GetEntitiesByType("light").size() == 1) {
            CAny maybeLight = CSimulator::GetInstance().GetSpace().GetEntitiesByType("light").begin()->second;
            auto light = any_cast<CLightEntity*>(maybeLight);
            if(GetSpace().GetSimulationClock() > 500) {
                light->MoveTo({2, 2, 0.1}, light->GetOrientation());
            }
        }*/


        for (CCI_RangeAndBearingSensor::SPacket packet : packets) {


        }
    }





}



void LoggingLoopFunctions::PostStep() {





}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(LoggingLoopFunctions, "logging_loop_functions")

