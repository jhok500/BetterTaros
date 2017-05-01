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


    IntCoord = new boost::circular_buffer<double>(3*AgentNumber*2);
    ExtCoord = new boost::circular_buffer<double>(4*AgentNumber*(AgentNumber-1)*2);
    YawSensor = new boost::circular_buffer<double>(2*AgentNumber*2);
    YawHolder = new boost::circular_buffer<double>(2*AgentNumber*2);
    YawHolder1 = new boost::circular_buffer<double>(3*AgentNumber*(AgentNumber-1)*2);
    hang = 0;
    FeatureVectors3 = new boost::circular_buffer<int>((1+(5*2))*50);
    MemoryLog = new boost::circular_buffer<int>(10);






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
    Agent.clear();
    AgentCoord.clear();
    AgentExt.clear();
    //bool 0Eyes1 = false;
    //std::cout << "Rolling" << std::endl;
    CSpace::TMapPerType &m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    CVector2 NeighbourDistance;
    Real Counter = -1;
    Agent.push_back(GetSpace().GetSimulationClock());
    AgentCoord.push_back(GetSpace().GetSimulationClock());
    AgentExt.push_back(GetSpace().GetSimulationClock());


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
        Real Velocity_Wheels = 0.5*(cController.GetSpoofLeftWheelVelocity() + cController.GetSpoofRightWheelVelocity());
        Real Difference_Wheels = cController.GetSpoofLeftWheelVelocity() - cController.GetSpoofRightWheelVelocity();
        CCI_RangeAndBearingSensor::TReadings packets = cController.GetRABSensorReadings();



        std::string OriginID = cFootBot.GetRootEntity().GetId().c_str();
        int RealID = OriginID.at(2) + 32;

        int MidProxCoord = 0;
        int CloseProxCoord = 0;
        int MidProx = 0;
        int CloseProx = 0;
        Indices.clear();
        Indices1.clear();
        Indices4.clear();
        unsigned seed = GetSpace().GetSimulationClock();
        std::default_random_engine generator(seed);
        std::normal_distribution<double> YawNoise(0,1);
        std::normal_distribution<double> XNoise(0,0.00001);
        std::normal_distribution<double> YNoise(0,0.00001);
        std::normal_distribution<double> RABNoise(0,1);
        //std::cout << "Rnd Noise = " << YawNoise(generator) << std::endl;
        AgentBearing_Sensor = (yaw.GetValue()*180/ARGOS_PI) + YawNoise(generator);
        AgentX = cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX() + XNoise(generator);
        AgentY = cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY() + YNoise(generator);
        //std::cout << AgentX - cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX() << std::endl;

        cPosInternalA.Set(AgentX,AgentY);
        Agent.push_back(-RealID);
        AgentCoord.push_back(-RealID);

        YawSensor->push_back(RealID);
        YawSensor->push_back(AgentBearing_Sensor);

        //////////////// EXT

        for(CCI_RangeAndBearingSensor::SPacket packet : packets)
        {

            double bearing = ToDegrees(packet.HorizontalBearing).GetValue();
            if (GetSpace().GetSimulationClock() >501 && RealID == 83 && abs(bearing) > 945) {
                //std::cout << "NOPE" << std::endl;
            }
            else {
                int MidProxExt = 0;
                int CloseProxExt = 0;
                Indices2.clear();
                Indices3.clear();
                std::string TargetID = "fb" + std::to_string(packet.Data[0]);
                int NeighbourID = packet.Data[0] + 80;
                AgentExt.push_back(-RealID);
                AgentExt.push_back(-NeighbourID);
                ExtCoord->push_back(RealID);
                ExtCoord->push_back(NeighbourID);

                MidProx = MidProx + 1;
                MidProxCoord = MidProxCoord + 1;
                MidProxExt = MidProxExt + 1;
                //AgentExt.push_back(MidProxExt);
                if (MidProxExt > 0) {
                    AgentExt.push_back(1);
                }
                else {
                    AgentExt.push_back(0);
                }


                if (packet.Range < 30) {
                    CloseProx = CloseProx + 1;
                    CloseProxExt = CloseProxExt + 1;
                }
                //AgentExt.push_back(CloseProxExt);
                if (CloseProxExt > 0) {
                    AgentExt.push_back(1);
                }
                else {
                    AgentExt.push_back(0);
                }

                Real TrueBearing;
                if (bearing + AgentBearing_Sensor < -180) {
                    TrueBearing = (360 - (sqrt(pow(bearing + AgentBearing_Sensor, 2))));
                }
                else if (bearing + AgentBearing_Sensor > 180) {
                    TrueBearing = -(360 - (sqrt(pow(bearing + AgentBearing_Sensor, 2))));
                }
                else {
                    TrueBearing = bearing + AgentBearing_Sensor;
                }
                double range = packet.Range + RABNoise(generator);
                Real TrueRange = (range / 100);
                //std::cout << RealID << " Range & angle" << TrueRange << ", " << bearing << ", " << TrueBearing << " " << NeighbourID << std::endl;
                NeighbourDistance.Set(TrueRange * cos(ARGOS_PI * TrueBearing / 180),
                                      TrueRange * sin(ARGOS_PI * TrueBearing / 180));
                if (sqrt(pow(NeighbourDistance.GetX(), 2) + pow(NeighbourDistance.GetY(), 2)) < 0.3) {
                    CloseProxCoord = CloseProxCoord + 1;
                }

                ExtCoord->push_back(cPosInternalA.GetX() + NeighbourDistance.GetX());
                ExtCoord->push_back(cPosInternalA.GetY() + NeighbourDistance.GetY());

                for (int i = 0; i < ExtCoord->size(); i++) {
                    //std::cout << ExtCoord->at(i) << std::endl;
                    if (ExtCoord->at(i) == RealID && ExtCoord->at(i + 1) == NeighbourID) {
                        Indices2.push_back(i);

                    }
                }
                //std::cout << Indices2.size() << std::endl;
                if (Indices2.size() >= 2) {

                    Real NeighbourDistanceTravelled = sqrt(
                            pow(((ExtCoord->at(Indices2.front() + 2)) - (ExtCoord->at(Indices2[1] + 2))), 2)
                            + pow(((ExtCoord->at(Indices2.front() + 3)) - (ExtCoord->at(Indices2[1] + 3))), 2));
                    Real NeighbourSpeed = NeighbourDistanceTravelled * 10;
                    //AgentExt.push_back(NeighbourDistanceTravelled);
                    if (NeighbourDistanceTravelled > 0.0037) {
                        AgentExt.push_back(1);
                    }
                    else {
                        AgentExt.push_back(0);
                    }
                    //AgentExt.push_back(NeighbourSpeed);
                    if (NeighbourSpeed > 0.01) {
                        AgentExt.push_back(1);
                    }
                    else {
                        AgentExt.push_back(0);
                    }
                    //std::cout << RealID << " thinks that " << NeighbourID << " is travelling " << NeighbourDistanceTravelled << std::endl;
                    YawHolder1->push_back(RealID);
                    YawHolder1->push_back(NeighbourID);
                    YawHolder1->push_back(atan2((ExtCoord->at(Indices2.front() + 2) - (ExtCoord->at(Indices2[1] + 2))),
                                                (ExtCoord->at(Indices2.front() + 3) -
                                                 (ExtCoord->at(Indices2[1] + 3)))));
                    for (int i = 0; i < YawHolder1->size(); i++) {
                        if (YawHolder1->at(i) == RealID && YawHolder1->at(i + 1) == NeighbourID) {
                            Indices3.push_back(i);
                            //std::cout << "CORRECT " << RealID << " YAW AT " << i << std::endl;
                        }
                    }
                    if (Indices3.size() >= 2) {
                        Real NeighbourYawCoord =
                                (YawHolder1->at(Indices3.front() + 2) - YawHolder1->at(Indices3[1] + 2)) * 180 /
                                ARGOS_PI;
                        if (sqrt(pow(NeighbourYawCoord, 2)) < 0.001) {
                            NeighbourYawCoord = 0;
                        }
                        //AgentExt.push_back(NeighbourYawCoord);
                        if (NeighbourYawCoord > 0.2) {
                            AgentExt.push_back(1);
                        }
                        else {
                            AgentExt.push_back(0);
                        }
                        //std::cout << RealID << " thinks that " << NeighbourID << " is turning " << NeighbourYawCoord << std::endl;
                    }
                }

                //std::cout << RealID << "  " << packet.Range << std::endl;
            }
        }

        //////////////// EXT

        //Agent.push_back(MidProxCoord);


        if (MidProxCoord > 0) {
            AgentCoord.push_back(1);
        }
        else {
            AgentCoord.push_back(0);
        }
        //Agent.push_back(MidProx);
        if (MidProx > 0) {
            Agent.push_back(1);
        }
        else {
            Agent.push_back(0);
        }
        //Agent.push_back(CloseProxCoord);
        if (CloseProxCoord > 0) {
            AgentCoord.push_back(1);
        }
        else {
            AgentCoord.push_back(0);
        }
        //Agent.push_back(CloseProx);
        if (CloseProx > 0) {
            Agent.push_back(1);
        }
        else {
            Agent.push_back(0);
        }

        if (IntCoord->size() == IntCoord->capacity()) {
            for (int i = 0; i < IntCoord->size(); i++) {
                //std::cout << IntCoord->at(i) << std::endl;
                if (IntCoord->at(i) == RealID) {
                    Indices.push_back(i);
                }
            }
            Real CoordDistance = sqrt(pow(((IntCoord->at(Indices.front() + 1)) - (IntCoord->at(Indices.back() + 1))), 2)
                                 + pow(((IntCoord->at(Indices.front() + 2)) - (IntCoord->at(Indices.back() + 2))), 2));
            Real CoordSpeed = CoordDistance * 10;

            //std::cout << "ID & distance & speed = " << RealID << ", " << CoordDistance << ", " << CoordSpeed << std::endl;
            //std::cout << "Time, X, Y = " << GetSpace().GetSimulationClock() << ", " << XNoise(generator) << ", " << YNoise(generator) << std::endl;

            //Agent.push_back(CoordDistance);
            if (CoordDistance > 0.0045) {
                AgentCoord.push_back(1);
            }
            else {
                AgentCoord.push_back(0);
            }
            //Agent.push_back((Velocity_Wheels/10)/100);
            if ((Velocity_Wheels/10)/100 > 0.0045) {
                Agent.push_back(1);
            }
            else {
                Agent.push_back(0);
            }
            //Agent.push_back(CoordSpeed);
            if (CoordSpeed > 0.01) {
                AgentCoord.push_back(1);
            }
            else {
                AgentCoord.push_back(0);
            }
            //Agent.push_back(Velocity_Wheels/100);
            if (Velocity_Wheels/100 > 0.01) {
                Agent.push_back(1);
            }
            else {
                Agent.push_back(0);
            }
            YawHolder->push_back(RealID);
            YawHolder->push_back(atan2(((IntCoord->at(Indices.front() + 1)) - (IntCoord->at(Indices.front() + 1 + (3*AgentNumber)))),
                                       ((IntCoord->at(Indices.front() + 2)) -
                                        (IntCoord->at(Indices.front() + 2 + (3*AgentNumber))))));
            if (YawHolder->size() == YawHolder->capacity()) {
                for (int i = 0; i < YawHolder->size(); i++) {
                    if (YawHolder->at(i) == RealID) {
                        Indices1.push_back(i);
                        //std::cout << "CORRECT " << RealID << " YAW AT " << i << std::endl;
                    }
                }
                /*IntYawCoord->push_back(RealID);
                IntYawCoord->push_back(YawHolder->at(Indices1.front()+1) - YawHolder->at(Indices1.back()+1));*/
                Real IntYawCoord = YawHolder->at(Indices1.front() + 1) - YawHolder->at(Indices1.back() + 1);
                //AgentCoord.push_back(IntYawCoord*180/ARGOS_PI);
                if (RealID == 83) {
                    //std::cout << "distance: " << CoordDistance << std::endl;
                    //std::cout << "YAW: " << fabs(IntYawCoord * 180 / ARGOS_PI) << std::endl;
                }
                if (fabs(IntYawCoord*180/ARGOS_PI) > 0.9) {
                    AgentCoord.push_back(1);
                }
                else {
                    AgentCoord.push_back(0);
                }
            }
            if (YawSensor->size() == YawSensor->capacity()) {
                for (int i = 0; i < YawSensor->size(); i++) {
                    if (YawSensor->at(i) == RealID) {
                        Indices4.push_back(i);
                    }
                }
                /*Real AngularVelocity = (YawSensor->at(Indices4.front()+1) - YawSensor->at(Indices4.back()+1));
                if (abs(AngularVelocity) < 0.001) {
                    AngularVelocity = 0;
                }
                Agent.push_back(AngularVelocity);*/
                if (Difference_Wheels == 0) {
                    Agent.push_back(0);

                }
                else {
                    Agent.push_back(1);
                }
            }
        }


        IntCoord->push_back(RealID);
        IntCoord->push_back(cPosInternalA.GetX());
        IntCoord->push_back(cPosInternalA.GetY());



    }

    //std::cout << "AGENTs: " << std::endl;
    for (int i = 0; i < Agent.size(); i++ ) {
        if (Agent[i] == -83 && GetSpace().GetSimulationClock() > 10) {
            /*std::cout << "Agent 83" << std::endl;
            std::cout << Agent[i+1] << std::endl;
            std::cout << AgentCoord[i+1] << std::endl;
            std::cout << Agent[i+2] << std::endl;
            std::cout << AgentCoord[i+2] << std::endl;
            std::cout << Agent[i+3] << std::endl;
            std::cout << AgentCoord[i+3] << std::endl;
            std::cout << Agent[i+4] << std::endl;
            std::cout << AgentCoord[i+4] << std::endl;
            std::cout << Agent[i+5] << std::endl;
            std::cout << AgentCoord[i+5] << std::endl;*/
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

    }
    if (FeatureVectors3->size() == FeatureVectors3->capacity()) {
        for (int i = 0; i < FeatureVectors3->size(); i++ ) {

            //std::cout << FeatureVectors3->at(i) << std::endl;
        }
    }

    //std::cout << Agent.size() << std::endl;
    /*std::cout << "AGENTCoords: " << std::endl;
    for (int i = 0; i < AgentCoord.size(); i++ ) {
        std::cout << AgentCoord[i] << std::endl;
    }*/
    /*std::cout << "AGENTs (external): " << std::endl;
    for (int i = 0; i < AgentExt.size(); i++ ) {
        std::cout << AgentExt[i] << std::endl;
    }*/

    if (Agent.size() == 61 && AgentCoord.size() == 61 && GetSpace().GetSimulationClock() < 3000 && GetSpace().GetSimulationClock() > 504) {
        //FVout0.open ("FeatureVectors0.csv", std::ios_base::app);
        //FVout1.open ("FeatureVectors1.csv", std::ios_base::app);
        //FVout2.open ("FeatureVectors2.csv", std::ios_base::app);
        FVout3.open("FeatureVectors3.csv", std::ios_base::app);
        //FVout4.open ("FeatureVectors4.csv", std::ios_base::app);
        //FVout << Agent[0] << ", ";
        for (int i = 0; i < Agent.size(); i++) {


            if (Agent[i] == -83) {
                // SOFTWARE HANG //
                /*if (GetSpace().GetSimulationClock() > 501) {
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
                }*/
                // Power Failure //
                /*if (GetSpace().GetSimulationClock() > 501) {
                    Agent[i+1] = 0;
                    Agent[i+2] = 0;
                    Agent[i+3] = 0;
                    Agent[i+4] = 0;
                    Agent[i+5] = 0;


                }*/
                // Complete Sensor Failure //
                /*if (GetSpace().GetSimulationClock() > 501) {
                    Agent[i+1] = 0;
                    Agent[i+2] = 0;

                }*/
                //FVout << Agent[i+1];
                //FVout << Agent[i+2];
                //FVout << Agent[i+3];
                //FVout << Agent[i+4];
                //FVout << Agent[i+5];

                /*FVout3 << Agent[i + 1] << ", ";
                FVout3 << AgentCoord[i + 1] << ", ";
                FVout3 << Agent[i + 2] << ", ";
                FVout3 << AgentCoord[i + 2] << ", ";
                FVout3 << Agent[i + 3] << ", ";
                FVout3 << AgentCoord[i + 3] << ", ";
                FVout3 << Agent[i + 4] << ", ";
                FVout3 << AgentCoord[i + 4] << ", ";
                FVout3 << Agent[i + 5] << ", ";
                FVout3 << AgentCoord[i + 5] << ", ";
                FVout3 << std::endl;*/


            }


        }
        FVout3.close();

    }















}



void LoggingLoopFunctions::PostStep() {





}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(LoggingLoopFunctions, "logging_loop_functions")

