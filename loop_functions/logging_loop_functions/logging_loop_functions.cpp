#include "logging_loop_functions.h"

#include <controllers/footbot_diffusion/footbot_diffusion.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include <boost/utility/binary.hpp>
#include <iostream>
#include <fstream>
#include <sstream>

//using namespace std;
/****************************************/
/****************************************/


CVector2 cPosInternalA;
int AgentNumber = 10;
LoggingLoopFunctions::LoggingLoopFunctions()
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
        Counter = Counter + 1;

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


        for (CCI_RangeAndBearingSensor::SPacket packet : packets) {


        }
    }


    //std::cout << "AGENTs: " << std::endl;
    /*for (int i = 0; i < Agent.size(); i++ ) {
        if (Agent[i] == -83 && GetSpace().GetSimulationClock() > 10) {
            std::cout << "Agent 83" << std::endl;
            std::cout << Agent[i+1] << std::endl;
            std::cout << AgentCoord[i+1] << std::endl;
            std::cout << Agent[i+2] << std::endl;
            std::cout << AgentCoord[i+2] << std::endl;
            std::cout << Agent[i+3] << std::endl;
            std::cout << AgentCoord[i+3] << std::endl;
            std::cout << Agent[i+4] << std::endl;
            std::cout << AgentCoord[i+4] << std::endl;
            std::cout << Agent[i+5] << std::endl;
            std::cout << AgentCoord[i+5] << std::endl;
            FeatureVectors3->push_back(GetSpace().GetSimulationClock());
            FeatureVectors3->push_back(Agent[i+1]);
            FeatureVectors3->push_back(AgentCoord[i+1]);
            FeatureVectors3->push_back(Agent[i+2]);
            FeatureVectors3->push_back(AgentCoord[i+2]);
            FeatureVectors3->push_back(Agent[i+3]);
            FeatureVectors3->push_back(AgentCoord[i+3]);
            FeatureVectors3->push_back(Agent[i+4]);
            FeatureVectors3->push_back(AgentCoord[i+4]);
            FeatureVectors3->push_back(Agent[i+5]);
            FeatureVectors3->push_back(AgentCoord[i+5]);

        }
        //std::cout << Agent[i] << std::endl;
        //std::cout << AgentCoord[i] << std::endl;

    }*/
    if (FeatureVectors3->size() == FeatureVectors3->capacity()) {
        for (int i = 0; i < FeatureVectors3->size(); i++ ) {

            //std::cout << FeatureVectors3->at(i) << std::endl;
        }
    }



    /*if (Agent.size() == 61 && AgentCoord.size() == 61 && GetSpace().GetSimulationClock() < 3000 && GetSpace().GetSimulationClock() > 504) {
        //FVout0.open ("FeatureVectors0.csv", std::ios_base::app);
        //FVout1.open ("FeatureVectors1.csv", std::ios_base::app);
        //FVout2.open ("FeatureVectors2.csv", std::ios_base::app);
        //FVout3.open("FeatureVectors3.csv", std::ios_base::app);
        //FVout4.open ("FeatureVectors4.csv", std::ios_base::app);
        //FVout << Agent[0] << ", ";
        for (int i = 0; i < Agent.size(); i++) {


            if (Agent[i] == -83) {
                // SOFTWARE HANG //
                *//*if (GetSpace().GetSimulationClock() > 501) {
                    if (hang == 0) {
                        hangF1 = Agent[i + 1];
                        hangF2 = Agent[i + 2];
                        hangF3 = Agent[i + 3];
                        hangF4 = Agent[i + 4];
                        hangF5 = Agent[i + 5];
                        hang = 1;
                    }
                    else {
                        Agent[i + 1] = hangF1;
                        Agent[i + 2] = hangF2;
                        Agent[i + 3] = hangF3;
                        Agent[i + 4] = hangF4;
                        Agent[i + 5] = hangF5;

                    }
                }*//*
                // Power Failure //
                *//*if (GetSpace().GetSimulationClock() > 501) {
                    Agent[i+1] = 0;
                    Agent[i+2] = 0;
                    Agent[i+3] = 0;
                    Agent[i+4] = 0;
                    Agent[i+5] = 0;


                }*//*
                // Complete Sensor Failure //
                *//*if (GetSpace().GetSimulationClock() > 501) {
                    Agent[i+1] = 0;
                    Agent[i+2] = 0;

                }*//*
                //FVout << Agent[i+1];
                //FVout << Agent[i+2];
                //FVout << Agent[i+3];
                //FVout << Agent[i+4];
                //FVout << Agent[i+5];

                *//*FVout3 << Agent[i + 1] << ", ";
                FVout3 << AgentCoord[i + 1] << ", ";
                FVout3 << Agent[i + 2] << ", ";
                FVout3 << AgentCoord[i + 2] << ", ";
                FVout3 << Agent[i + 3] << ", ";
                FVout3 << AgentCoord[i + 3] << ", ";
                FVout3 << Agent[i + 4] << ", ";
                FVout3 << AgentCoord[i + 4] << ", ";
                FVout3 << Agent[i + 5] << ", ";
                FVout3 << AgentCoord[i + 5] << ", ";
                FVout3 << std::endl;*//*


            }


        }
        FVout3.close();

    }*/















}



void LoggingLoopFunctions::PostStep() {





}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(LoggingLoopFunctions, "logging_loop_functions")

