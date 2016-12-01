#include "logging_loop_functions.h"

#include <controllers/footbot_diffusion/footbot_diffusion.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include <iostream>
#include <fstream>

//using namespace std;
/****************************************/
/****************************************/


CVector2 cPosInternalA;
Real Yaw_Coordinates_Internal;
Real Yaw_Coordinates_0a_Internal;
Real Yaw_Coordinates_1a_Internal;
Real Yaw_Coordinates_2a_Internal;
Real Yaw_Coordinates_0b_Internal = 0;
Real Yaw_Coordinates_1b_Internal = 0;
Real Yaw_Coordinates_2b_Internal = 0;
Real Yaw_Coordinates_0a_From_1;
Real Yaw_Coordinates_0a_From_2;
Real Yaw_Coordinates_1a_From_0;
Real Yaw_Coordinates_1a_From_2;
Real Yaw_Coordinates_2a_From_0;
Real Yaw_Coordinates_2a_From_1;
Real Yaw_Coordinates_0b_From_1 = 0;
Real Yaw_Coordinates_0b_From_2 = 0;
Real Yaw_Coordinates_1b_From_0 = 0;
Real Yaw_Coordinates_1b_From_2 = 0;
Real Yaw_Coordinates_2b_From_0 = 0;
Real Yaw_Coordinates_2b_From_1 = 0;
int MidNeighbours_0;
int MidEx_01;
int MidEx_02;
int MidNeighbours_1;
int MidEx_10;
int MidEx_12;
int MidNeighbours_2;
int MidEx_20;
int MidEx_21;
int CloseNeighbours_0;
int CloseEx_01;
int CloseEx_02;
int CloseNeighbours_1;
int CloseEx_10;
int CloseEx_12;
int CloseNeighbours_2;
int CloseEx_20;
int CloseEx_21;
std::ofstream agent0("agent0.txt");
std::ofstream agent1("agent1.txt");
std::ofstream agent2("agent2.txt");

LoggingLoopFunctions::LoggingLoopFunctions()
{

    IntCoordCurrent0 = new boost::circular_buffer<double>(22);
    ExtCoordCurrent01 = new boost::circular_buffer<double>(22);
    ExtCoordCurrent02 = new boost::circular_buffer<double>(22);
    WheelDataCurrent0 = new boost::circular_buffer<double>(10);
    RABDataCurrent0 = new boost::circular_buffer<double>(10);
    IntYawCoord0 = new boost::circular_buffer<double>(10);
    ExtYawCoord01 = new boost::circular_buffer<double>(10);
    ExtYawCoord02 = new boost::circular_buffer<double>(10);
    MidCurrent0 = new boost::circular_buffer<int>(10);
    CloseCurrent0 = new boost::circular_buffer<int>(10);
    ExMidCurrent01 = new boost::circular_buffer<int>(10);
    ExMidCurrent02 = new boost::circular_buffer<int>(10);
    ExCloseCurrent01 = new boost::circular_buffer<int>(10);
    ExCloseCurrent02 = new boost::circular_buffer<int>(10);


    IntCoordCurrent1 = new boost::circular_buffer<double>(22);
    ExtCoordCurrent10 = new boost::circular_buffer<double>(22);
    ExtCoordCurrent12 = new boost::circular_buffer<double>(22);
    WheelDataCurrent1 = new boost::circular_buffer<double>(10);
    RABDataCurrent1 = new boost::circular_buffer<double>(10);
    IntYawCoord1 = new boost::circular_buffer<double>(10);
    ExtYawCoord10 = new boost::circular_buffer<double>(10);
    ExtYawCoord12 = new boost::circular_buffer<double>(10);
    MidCurrent1 = new boost::circular_buffer<int>(10);
    CloseCurrent1 = new boost::circular_buffer<int>(10);
    ExMidCurrent10 = new boost::circular_buffer<int>(10);
    ExMidCurrent12 = new boost::circular_buffer<int>(10);
    ExCloseCurrent10 = new boost::circular_buffer<int>(10);
    ExCloseCurrent12 = new boost::circular_buffer<int>(10);

    IntCoordCurrent2 = new boost::circular_buffer<double>(22);
    ExtCoordCurrent20 = new boost::circular_buffer<double>(22);
    ExtCoordCurrent21 = new boost::circular_buffer<double>(22);
    WheelDataCurrent2 = new boost::circular_buffer<double>(10);
    RABDataCurrent2 = new boost::circular_buffer<double>(10);
    IntYawCoord2 = new boost::circular_buffer<double>(10);
    ExtYawCoord20 = new boost::circular_buffer<double>(10);
    ExtYawCoord21 = new boost::circular_buffer<double>(10);
    MidCurrent2 = new boost::circular_buffer<int>(10);
    CloseCurrent2 = new boost::circular_buffer<int>(10);
    ExMidCurrent20 = new boost::circular_buffer<int>(10);
    ExMidCurrent21 = new boost::circular_buffer<int>(10);
    ExCloseCurrent20 = new boost::circular_buffer<int>(10);
    ExCloseCurrent21 = new boost::circular_buffer<int>(10);


    IntCoordCurrent = new boost::circular_buffer<double>(11*3);
    ExtCoordCurrent = new boost::circular_buffer<double>(22*3);
    WheelDataCurrent = new boost::circular_buffer<double>(10*3);
    RABDataCurrent = new boost::circular_buffer<double>(10*3);
    YawHolder = new boost::circular_buffer<double>(2*3);
    IntYawCoord = new boost::circular_buffer<double>(10*3);
    ExtYawCoord = new boost::circular_buffer<double>(10*3);
    MidCurrent = new boost::circular_buffer<int>(10*3);
    CloseCurrent = new boost::circular_buffer<int>(10*3);
    ExMidCurrent = new boost::circular_buffer<int>(10*3);
    ExCloseCurrent = new boost::circular_buffer<int>(10*3);



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
    CVector2 NeighbourPos;
    Real Counter = -1;
    Agent.clear();
    Agent0.clear();
    Agent1.clear();
    Agent2.clear();
    int ZeroOne = 0;
    int ZeroTwo = 0;
    int OneZero = 0;
    int OneTwo = 0;
    int TwoZero = 0;
    int TwoOne = 0;
    MidEx_01 = 0;
    MidEx_02 = 0;
    MidEx_10 = 0;
    MidEx_12 = 0;
    MidEx_20 = 0;
    MidEx_21 = 0;
    CloseEx_01 = 0;
    CloseEx_02 = 0;
    CloseEx_10 = 0;
    CloseEx_12 = 0;
    CloseEx_20 = 0;
    CloseEx_21 = 0;
    std::vector<int> indices;

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
        CCI_RangeAndBearingSensor::TReadings packets = cController.GetRABSensorReadings();
        AgentBearing_Sensor = yaw.GetValue()*180/ARGOS_PI;

        std::string OriginID = cFootBot.GetRootEntity().GetId().c_str();
        int RealID = OriginID.at(2) + 32;
        cPosInternalA.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        int MidProxNeighbours = 0;
        int CloseProxNeighbours = 0;
        if (OriginID == std::string("fb0")) {
            RABDataCurrent0->push_back(AgentBearing_Sensor);
            WheelDataCurrent0->push_back(Velocity_Wheels/100);
            IntCoordCurrent0->push_back(cPosInternalA.GetX());
            IntCoordCurrent0->push_back(cPosInternalA.GetY());
        }
        if (OriginID == std::string("fb1")) {
            RABDataCurrent1->push_back(AgentBearing_Sensor);
            WheelDataCurrent1->push_back(Velocity_Wheels/100);
            IntCoordCurrent1->push_back(cPosInternalA.GetX());
            IntCoordCurrent1->push_back(cPosInternalA.GetY());
        }
        if (OriginID == std::string("fb2")) {
            RABDataCurrent2->push_back(AgentBearing_Sensor);
            WheelDataCurrent2->push_back(Velocity_Wheels/100);
            IntCoordCurrent2->push_back(cPosInternalA.GetX());
            IntCoordCurrent2->push_back(cPosInternalA.GetY());
        }



        for(CCI_RangeAndBearingSensor::SPacket packet : packets)
        {
            if (packet.Range < 60) {
                MidProxNeighbours = MidProxNeighbours + 1;
            }
            if(packet.Range < 27) {
                CloseProxNeighbours = CloseProxNeighbours+1;
            }

            double bearing = ToDegrees(packet.HorizontalBearing).GetValue();
            Real TrueBearing;
            if (bearing + AgentBearing_Sensor < -180) {
                TrueBearing = (360-(sqrt(pow(bearing + AgentBearing_Sensor,2))));
            }
            else if (bearing + AgentBearing_Sensor > 180) {
                TrueBearing = -(360-(sqrt(pow(bearing + AgentBearing_Sensor,2))));
            }
            else {
                TrueBearing = bearing+AgentBearing_Sensor;
            }
            std::string TargetID = "fb" + std::to_string(packet.Data[0]);



            Real TrueRange;
            TrueRange = packet.Range/100;
            NeighbourPos.Set(TrueRange*cos(ARGOS_PI*TrueBearing/180),TrueRange*sin(ARGOS_PI*TrueBearing/180));



            if (OriginID == std::string("fb0")) {
                MidNeighbours_0 = MidProxNeighbours;
                CloseNeighbours_0 = CloseProxNeighbours;
                if (TargetID == "fb1") {
                    ZeroOne = 1;
                    ExtCoordCurrent01->push_back(cPosInternalA.GetX() + NeighbourPos.GetX());
                    ExtCoordCurrent01->push_back(cPosInternalA.GetY() + NeighbourPos.GetY());
                    if (sqrt(pow(cPosInternalA.GetX() - (cPosInternalA.GetX() + NeighbourPos.GetX()), 2) +
                             pow(cPosInternalA.GetY() - (cPosInternalA.GetY() + NeighbourPos.GetY()), 2)) < 0.6) {
                        MidEx_01 = MidEx_01 + 1;
                        if (sqrt(pow(cPosInternalA.GetX() - (cPosInternalA.GetX() + NeighbourPos.GetX()), 2) +
                                 pow(cPosInternalA.GetY() - (cPosInternalA.GetY() + NeighbourPos.GetY()), 2)) < 0.27) {
                            //std::cout << "0 TURN" << std::endl;
                            CloseEx_01 = CloseEx_01 + 1;
                        }
                    }
                }
                if (TargetID == "fb2") {
                    ZeroTwo = 1;
                    ExtCoordCurrent02->push_back(cPosInternalA.GetX() + NeighbourPos.GetX());
                    ExtCoordCurrent02->push_back(cPosInternalA.GetY() + NeighbourPos.GetY());
                    if (sqrt(pow(cPosInternalA.GetX() - (cPosInternalA.GetX() + NeighbourPos.GetX()), 2) +
                             pow(cPosInternalA.GetY() - (cPosInternalA.GetY() + NeighbourPos.GetY()), 2)) < 0.6) {
                        MidEx_02 = MidEx_02 + 1;
                        if (sqrt(pow(cPosInternalA.GetX() - (cPosInternalA.GetX() + NeighbourPos.GetX()), 2) +
                                 pow(cPosInternalA.GetY() - (cPosInternalA.GetY() + NeighbourPos.GetY()), 2)) < 0.27) {
                            //std::cout << "0 TURN" << std::endl;
                            CloseEx_02 = CloseEx_02 + 1;
                        }
                    }
                }
            }
            if (OriginID == std::string("fb1")) {
                MidNeighbours_1 = MidProxNeighbours;
                CloseNeighbours_1 = CloseProxNeighbours;
                if (TargetID == "fb0") {
                    OneZero = 1;
                    ExtCoordCurrent10->push_back(cPosInternalA.GetX() + NeighbourPos.GetX());
                    ExtCoordCurrent10->push_back(cPosInternalA.GetY() + NeighbourPos.GetY());
                    if (sqrt(pow(cPosInternalA.GetX() - (cPosInternalA.GetX() + NeighbourPos.GetX()), 2) +
                             pow(cPosInternalA.GetY() - (cPosInternalA.GetY() + NeighbourPos.GetY()), 2)) < 0.6) {
                        MidEx_10 = MidEx_10 + 1;

                        if (sqrt(pow(cPosInternalA.GetX() - (cPosInternalA.GetX() + NeighbourPos.GetX()), 2) +
                                 pow(cPosInternalA.GetY() - (cPosInternalA.GetY() + NeighbourPos.GetY()), 2)) < 0.27) {
                            //std::cout << "1 TURN" << std::endl;
                            CloseEx_10 = CloseEx_10 + 1;
                        }
                    }
                }
                if (TargetID == "fb2") {
                    OneTwo = 1;
                    ExtCoordCurrent12->push_back(cPosInternalA.GetX() + NeighbourPos.GetX());
                    ExtCoordCurrent12->push_back(cPosInternalA.GetY() + NeighbourPos.GetY());
                    if (sqrt(pow(cPosInternalA.GetX() - (cPosInternalA.GetX() + NeighbourPos.GetX()), 2) +
                             pow(cPosInternalA.GetY() - (cPosInternalA.GetY() + NeighbourPos.GetY()), 2)) < 0.6) {
                        MidEx_12 = MidEx_12 + 1;

                        if (sqrt(pow(cPosInternalA.GetX() - (cPosInternalA.GetX() + NeighbourPos.GetX()), 2) +
                                 pow(cPosInternalA.GetY() - (cPosInternalA.GetY() + NeighbourPos.GetY()), 2)) < 0.27) {
                            //std::cout << "1 TURN" << std::endl;
                            CloseEx_12 = CloseEx_12 + 1;
                        }
                    }
                }
            }
            if (OriginID == std::string("fb2")) {
                MidNeighbours_2 = MidProxNeighbours;
                CloseNeighbours_2 = CloseProxNeighbours;
                if (TargetID == "fb0") {
                    TwoZero = 1;
                    ExtCoordCurrent20->push_back(cPosInternalA.GetX() + NeighbourPos.GetX());
                    ExtCoordCurrent20->push_back(cPosInternalA.GetY() + NeighbourPos.GetY());
                    if (sqrt(pow(cPosInternalA.GetX() - (cPosInternalA.GetX() + NeighbourPos.GetX()), 2) +
                             pow(cPosInternalA.GetY() - (cPosInternalA.GetY() + NeighbourPos.GetY()), 2)) < 0.6) {
                        MidEx_20 = MidEx_20 + 1;

                        if (sqrt(pow(cPosInternalA.GetX() - (cPosInternalA.GetX() + NeighbourPos.GetX()), 2) +
                                 pow(cPosInternalA.GetY() - (cPosInternalA.GetY() + NeighbourPos.GetY()), 2)) < 0.27) {
                            //std::cout << "1 TURN" << std::endl;
                            CloseEx_20 = CloseEx_20 + 1;
                        }
                    }
                }
                if (TargetID == "fb1") {
                    TwoOne = 1;
                    ExtCoordCurrent21->push_back(cPosInternalA.GetX() + NeighbourPos.GetX());
                    ExtCoordCurrent21->push_back(cPosInternalA.GetY() + NeighbourPos.GetY());
                    if (sqrt(pow(cPosInternalA.GetX() - (cPosInternalA.GetX() + NeighbourPos.GetX()), 2) +
                             pow(cPosInternalA.GetY() - (cPosInternalA.GetY() + NeighbourPos.GetY()), 2)) < 0.6) {
                        MidEx_21 = MidEx_21 + 1;

                        if (sqrt(pow(cPosInternalA.GetX() - (cPosInternalA.GetX() + NeighbourPos.GetX()), 2) +
                                 pow(cPosInternalA.GetY() - (cPosInternalA.GetY() + NeighbourPos.GetY()), 2)) < 0.27) {
                            //std::cout << "1 TURN" << std::endl;
                            CloseEx_21 = CloseEx_21 + 1;
                        }
                    }
                }
            }





        }





    }
    MidCurrent0->push_back(MidNeighbours_0);
    CloseCurrent0->push_back(CloseNeighbours_0);
    if (ZeroOne > 0) {
        ExMidCurrent01->push_back(MidEx_01);
        ExCloseCurrent01->push_back(CloseEx_01);
    }
    else {
        ExMidCurrent01->push_back(-1);
        ExCloseCurrent01->push_back(-1);

    }
    if (ZeroTwo > 0) {
        ExMidCurrent02->push_back(MidEx_02);
        ExCloseCurrent02->push_back(CloseEx_01);
    }
    else {
        ExMidCurrent02->push_back(-1);
        ExCloseCurrent02->push_back(-1);
    }
    MidCurrent1->push_back(MidNeighbours_1);
    CloseCurrent1->push_back(CloseNeighbours_1);
    if (OneZero > 0) {
        ExMidCurrent10->push_back(MidEx_10);
        ExCloseCurrent10->push_back(CloseEx_10);
    }
    else {
        ExMidCurrent10->push_back(-1);
        ExCloseCurrent10->push_back(-1);
    }
    if (OneTwo > 0) {
        ExMidCurrent12->push_back(MidEx_12);
        ExCloseCurrent12->push_back(CloseEx_12);
    }
    else {
        ExMidCurrent12->push_back(-1);
        ExCloseCurrent12->push_back(-1);
    }
    MidCurrent2->push_back(MidNeighbours_2);
    CloseCurrent2->push_back(CloseNeighbours_2);
    if (TwoZero > 0) {
        ExMidCurrent20->push_back(MidEx_20);
        ExCloseCurrent20->push_back(CloseEx_20);
    }
    else{
        ExMidCurrent20->push_back(-1);
        ExCloseCurrent20->push_back(-1);
    }
    if (TwoOne > 0) {
        ExMidCurrent21->push_back(MidEx_21);
        ExCloseCurrent21->push_back(CloseEx_21);
    }
    else {
        ExMidCurrent21->push_back(-1);
        ExCloseCurrent21->push_back(-1);
    }
    if (GetSpace().GetSimulationClock() > 22) {
        if (IntCoordCurrent0->size() == 22 && IntCoordCurrent1->size() == 22) {

            Yaw_Coordinates_0a_Internal = atan2(IntCoordCurrent0->at(21) - IntCoordCurrent0->at(19),
                                                IntCoordCurrent0->at(20) - IntCoordCurrent0->at(18)) * 180 / ARGOS_PI;
            if (ExtCoordCurrent10->size() == 22 && OneZero > 0) {
                Yaw_Coordinates_0a_From_1 = atan2(ExtCoordCurrent10->at(21) - ExtCoordCurrent10->at(19),
                                                  ExtCoordCurrent10->at(20) - ExtCoordCurrent10->at(18)) * 180 /
                                            ARGOS_PI;
            }
            if (ExtCoordCurrent20->size() == 22 && TwoZero > 0) {
                Yaw_Coordinates_0a_From_2 = atan2(ExtCoordCurrent20->at(21) - ExtCoordCurrent20->at(19),
                                                  ExtCoordCurrent20->at(20) - ExtCoordCurrent20->at(18)) * 180 /
                                            ARGOS_PI;
            }
            Yaw_Coordinates_1a_Internal = atan2(IntCoordCurrent1->at(21) - IntCoordCurrent1->at(19),
                                                IntCoordCurrent1->at(20) - IntCoordCurrent1->at(18)) * 180 / ARGOS_PI;
            if (ExtCoordCurrent01->size() == 22 && OneZero > 0) {
                Yaw_Coordinates_1a_From_0 = atan2(ExtCoordCurrent01->at(21) - ExtCoordCurrent01->at(19),
                                                  ExtCoordCurrent01->at(20) - ExtCoordCurrent01->at(18)) * 180 /
                                            ARGOS_PI;
            }
            if (ExtCoordCurrent21->size() == 22 && OneTwo > 0) {
                Yaw_Coordinates_1a_From_2 = atan2(ExtCoordCurrent21->at(21) - ExtCoordCurrent21->at(19),
                                                  ExtCoordCurrent21->at(20) - ExtCoordCurrent21->at(18)) * 180 /
                                            ARGOS_PI;
            }
            Yaw_Coordinates_2a_Internal = atan2(IntCoordCurrent2->at(21) - IntCoordCurrent2->at(19),
                                                IntCoordCurrent2->at(20) - IntCoordCurrent2->at(18)) * 180 / ARGOS_PI;
            if (ExtCoordCurrent02->size() == 22 && TwoZero > 0) {
                Yaw_Coordinates_2a_From_0 = atan2(ExtCoordCurrent02->at(21) - ExtCoordCurrent02->at(19),
                                                  ExtCoordCurrent02->at(20) - ExtCoordCurrent02->at(18)) * 180 /
                                            ARGOS_PI;
            }
            if (ExtCoordCurrent12->size() == 22 && TwoOne > 0) {
                Yaw_Coordinates_2a_From_1 = atan2(ExtCoordCurrent12->at(21) - ExtCoordCurrent12->at(19),
                                                  ExtCoordCurrent12->at(20) - ExtCoordCurrent12->at(18)) * 180 /
                                            ARGOS_PI;
            }
        }
    }
    if (GetSpace().GetSimulationClock() > 32){


        if (std::accumulate(MidCurrent0->begin(), MidCurrent0->end(), 0.0) > 5) {
            Agent0.push_back(1);
            //AGENT 0 #MID-PROX NEIGHBOURS ACCORDING TO SELF
        }
        else {
            Agent0.push_back(0);
        }
        if (sqrt(pow(IntCoordCurrent0->at(20) - IntCoordCurrent1->at(0),2)+pow(IntCoordCurrent0->at(21) - IntCoordCurrent1->at(1),2)) < 0.6 ||
            sqrt(pow(IntCoordCurrent0->at(20) - IntCoordCurrent2->at(0),2)+pow(IntCoordCurrent0->at(21) - IntCoordCurrent2->at(1),2)) < 0.6) {
            Agent0.push_back(1);
        }
        else {
            Agent0.push_back(0);
        }

        if (std::accumulate(ExMidCurrent10->begin(),ExMidCurrent10->end(),0.0) > -1) {
            if (std::accumulate(ExMidCurrent10->begin(), ExMidCurrent10->end(), 0.0) > 5) {
                Agent0.push_back(1);
                //AGENT 0 #MID-PROX NEIGHBOURS ACCORDING TO NEIGHBOUR 1
            }
            else {
                Agent0.push_back(0);
            }
        }
        else {
            Agent0.push_back(8);
        }

        if (std::accumulate(ExMidCurrent20->begin(),ExMidCurrent20->end(),0.0) > -1) {
            if (std::accumulate(ExMidCurrent20->begin(), ExMidCurrent20->end(), 0.0) > 5) {
                Agent0.push_back(1);
                //AGENT 0 #MID-PROX NEIGHBOURS ACCORDING TO NEIGHBOUR 2
            }
            else {
                Agent0.push_back(0);
            }
        }
        else {
            Agent0.push_back(8);
        }

        if (std::accumulate(CloseCurrent0->begin(), CloseCurrent0->end(), 0.0) > 5) {
            Agent0.push_back(1);
            //AGENT 0 #CLOSE-PROX NEIGHBOURS ACCORDING TO SELF
        }
        else {
            Agent0.push_back(0);
        }
        if (sqrt(pow(IntCoordCurrent0->at(20) - IntCoordCurrent1->at(0),2)+pow(IntCoordCurrent0->at(21) - IntCoordCurrent1->at(1),2)) < 0.27 ||
            sqrt(pow(IntCoordCurrent0->at(20) - IntCoordCurrent2->at(0),2)+pow(IntCoordCurrent0->at(21) - IntCoordCurrent2->at(1),2)) < 0.27) {
            Agent0.push_back(1);
        }
        else {
            Agent0.push_back(0);
        }
        if (std::accumulate(ExCloseCurrent10->begin(),ExCloseCurrent10->end(),0.0) > -1) {
            if (std::accumulate(ExCloseCurrent10->begin(), ExCloseCurrent10->end(), 0.0) > 5) {
                Agent0.push_back(1);
                //AGENT 0 #CLOSE-PROX NEIGHBOURS ACCORDING TO NEIGHBOUR 1
            }
            else {
                Agent0.push_back(0);
            }
        }
        else {
            Agent0.push_back(8);
        }
        if (std::accumulate(ExCloseCurrent20->begin(),ExCloseCurrent20->end(),0.0) >= 0) {
            if (std::accumulate(ExCloseCurrent20->begin(), ExCloseCurrent20->end(), 0.0) > 5) {
                Agent0.push_back(1);
                //AGENT 0 #CLOSE-PROX NEIGHBOURS ACCORDING TO NEIGHBOUR 2
            }
            else {
                Agent0.push_back(0);
            }
        }
        else {
            Agent0.push_back(8);
        }
        if (sqrt(pow(IntCoordCurrent0->at(20) - IntCoordCurrent0->at(0),2)+pow(IntCoordCurrent0->at(21) - IntCoordCurrent0->at(1),2)) > 0.037) {
            Agent0.push_back(1);
            //AGENT 0 DISTANCE TRAVELLED FROM INTERNAL COORDINATES
        }
        else {
            Agent0.push_back(0);
        }

        if (std::accumulate(WheelDataCurrent0->begin(),WheelDataCurrent0->end(),0.0) > 0.37) {
            Agent0.push_back(1);
            //AGENT 0 DISTANCE TRAVELLED ACCORDING TO INTERNAL WHEEL SPEED
        }
        else {
            Agent0.push_back(0);
        }
        if (ExtCoordCurrent10->size() == 22 && OneZero > 0) {
            if (sqrt(pow(ExtCoordCurrent10->at(20) - ExtCoordCurrent10->at(0), 2) +
                     pow(ExtCoordCurrent10->at(21) - ExtCoordCurrent10->at(1), 2)) > 0.037) {
                Agent0.push_back(1);
                //AGENT 0 DISTANCE TRAVELLED ACCORDING TO AGENT 1
            }
            else {
                Agent0.push_back(0);
            }
        }
        else {
            Agent0.push_back(8);
        }
        if (ExtCoordCurrent20->size() == 22 && TwoZero > 0) {
            if (sqrt(pow(ExtCoordCurrent20->at(20) - ExtCoordCurrent20->at(0), 2) +
                     pow(ExtCoordCurrent20->at(21) - ExtCoordCurrent20->at(1), 2)) > 0.037) {
                Agent0.push_back(1);
                //AGENT 0 DISTANCE TRAVELLED ACCORDING TO AGENT 2
            }
            else {
                Agent0.push_back(0);
            }
        }
        else {
            Agent0.push_back(8);
        }
        if (sqrt(pow(std::accumulate(IntYawCoord0->begin(),IntYawCoord0->end(),0.0),2)) > 0.05) {
            Agent0.push_back(1);
            //AGENT 0 ANGULAR VELOCITY ACCORDING TO INTERNAL COORDINATES
        }
        else {
            Agent0.push_back(0);
        }
        if (sqrt(pow(RABDataCurrent0->at(9) - RABDataCurrent0->at(0),2)) > 5) {
            Agent0.push_back(1);
            //AGENT 0 ANGULAR VELOCITY ACCORDING TO INTERNAL BEARING SENSOR
        }
        else {
            Agent0.push_back(0);
        }
        if (ExtYawCoord10->size() == 10 && OneZero > 0) {
            if (sqrt(pow(std::accumulate(ExtYawCoord10->begin(), ExtYawCoord10->end(), 0.0), 2)) > 0.05) {
                Agent0.push_back(1);
                //AGENT 0 ANGULAR VELOCITY ACCORDING TO AGENT 1
            }
            else {
                Agent0.push_back(0);
            }
        }
        else {
            Agent0.push_back(8);
        }
        if (ExtYawCoord20->size() == 10 && TwoZero > 0) {
            if (sqrt(pow(std::accumulate(ExtYawCoord20->begin(), ExtYawCoord20->end(), 0.0), 2)) > 0.05) {
                Agent0.push_back(1);
                //AGENT 0 ANGULAR VELOCITY ACCORDING TO AGENT 2
            }
            else {
                Agent0.push_back(0);
            }
        }
        else {
            Agent0.push_back(8);
        }
        if (sqrt(pow(IntCoordCurrent0->at(20) - IntCoordCurrent0->at(0),2)+pow(IntCoordCurrent0->at(21) - IntCoordCurrent0->at(1),2)) > 0.02) {
            Agent0.push_back(1);
            //AGENT 0 SPEED ACCORDING TO INTERNAL COORDINATES
        }
        else {
            Agent0.push_back(0);
        }
        if (std::accumulate(WheelDataCurrent0->begin(),WheelDataCurrent0->end(),0.0) > 0.2) {
            Agent0.push_back(1);
            //AGENT 0 SPEED ACCORDING INTERNAL WHEEL SENSOR
        }
        else {
            Agent0.push_back(0);
        }
        if (ExtCoordCurrent10->size() == 22 && OneZero > 0) {
            if (sqrt(pow(ExtCoordCurrent10->at(20) - ExtCoordCurrent10->at(0), 2) +
                     pow(ExtCoordCurrent10->at(21) - ExtCoordCurrent10->at(1), 2)) > 0.02) {
                Agent0.push_back(1);
                //AGENT 0 SPEED ACCORDING TO AGENT 1
            }
            else {
                Agent0.push_back(0);
            }
        }
        else {
            Agent0.push_back(8);
        }
        if (ExtCoordCurrent20->size() == 22 && TwoZero > 0) {
            if (sqrt(pow(ExtCoordCurrent20->at(20) - ExtCoordCurrent20->at(0), 2) +
                     pow(ExtCoordCurrent20->at(21) - ExtCoordCurrent20->at(1), 2)) > 0.02) {
                Agent0.push_back(1);
                //AGENT 0 SPEED ACCORDING TO AGENT 2
            }
            else {
                Agent0.push_back(0);
            }
        }
        else {
            Agent0.push_back(8);
        }

        /// AGENT 1 ///

        if (std::accumulate(MidCurrent1->begin(), MidCurrent1->end(), 0.0) > 5) {
            Agent1.push_back(1);
            //AGENT 1 #MID-PROX NEIGHBOURS ACCORDING TO SELF
        }
        else {
            Agent1.push_back(0);
        }


        if (std::accumulate(ExMidCurrent01->begin(),ExMidCurrent01->end(),0.0) > -1) {
            if (std::accumulate(ExMidCurrent01->begin(), ExMidCurrent01->end(), 0.0) > 5) {
                Agent1.push_back(1);
                //AGENT 1 #MID-PROX NEIGHBOURS ACCORDING TO NEIGHBOUR 0
            }
            else {
                Agent1.push_back(0);
            }
        }
        else {
            Agent1.push_back(8);
        }
        if (sqrt(pow(IntCoordCurrent1->at(20) - IntCoordCurrent0->at(0),2)+pow(IntCoordCurrent1->at(21) - IntCoordCurrent0->at(1),2)) < 0.6 ||
            sqrt(pow(IntCoordCurrent1->at(20) - IntCoordCurrent2->at(0),2)+pow(IntCoordCurrent1->at(21) - IntCoordCurrent2->at(1),2)) < 0.6) {
            Agent1.push_back(1);
        }
        else {
            Agent1.push_back(0);
        }
        if (std::accumulate(ExMidCurrent21->begin(),ExMidCurrent21->end(),0.0) > -1) {
            if (std::accumulate(ExMidCurrent21->begin(), ExMidCurrent21->end(), 0.0) > 5) {
                Agent1.push_back(1);
                //AGENT 1 #MID-PROX NEIGHBOURS ACCORDING TO NEIGHBOUR 2
            }
            else {
                Agent1.push_back(0);
            }
        }
        else {
            Agent1.push_back(8);
        }

        if (std::accumulate(CloseCurrent1->begin(), CloseCurrent1->end(), 0.0) > 5) {
            Agent1.push_back(1);
            //AGENT 1 #CLOSE-PROX NEIGHBOURS ACCORDING TO SELF
        }
        else {
            Agent1.push_back(0);
        }
        if (sqrt(pow(IntCoordCurrent1->at(20) - IntCoordCurrent0->at(0),2)+pow(IntCoordCurrent1->at(21) - IntCoordCurrent0->at(1),2)) < 0.27 ||
            sqrt(pow(IntCoordCurrent1->at(20) - IntCoordCurrent2->at(0),2)+pow(IntCoordCurrent1->at(21) - IntCoordCurrent2->at(1),2)) < 0.27) {
            Agent1.push_back(1);
        }
        else {
            Agent1.push_back(0);
        }
        if (std::accumulate(ExCloseCurrent01->begin(),ExCloseCurrent01->end(),0.0) > -1) {
            if (std::accumulate(ExCloseCurrent01->begin(), ExCloseCurrent01->end(), 0.0) > 5) {
                Agent1.push_back(1);
                //AGENT 1 #CLOSE-PROX NEIGHBOURS ACCORDING TO NEIGHBOUR 0
            }
            else {
                Agent1.push_back(0);
            }
        }
        else {
            Agent1.push_back(8);
        }
        if (std::accumulate(ExCloseCurrent21->begin(),ExCloseCurrent21->end(),0.0) >= 0) {
            if (std::accumulate(ExCloseCurrent21->begin(), ExCloseCurrent21->end(), 0.0) > 5) {
                Agent1.push_back(1);
                //AGENT 1 #CLOSE-PROX NEIGHBOURS ACCORDING TO NEIGHBOUR 2
            }
            else {
                Agent1.push_back(0);
            }
        }
        else {
            Agent1.push_back(8);
        }
        if (sqrt(pow(IntCoordCurrent1->at(20) - IntCoordCurrent1->at(0),2)+pow(IntCoordCurrent1->at(21) - IntCoordCurrent1->at(1),2)) > 0.037) {
            Agent1.push_back(1);
            //AGENT 1 DISTANCE TRAVELLED FROM INTERNAL COORDINATES
        }
        else {
            Agent1.push_back(0);
        }

        if (std::accumulate(WheelDataCurrent1->begin(),WheelDataCurrent1->end(),0.0) > 0.37) {
            Agent1.push_back(1);
            //AGENT 1 DISTANCE TRAVELLED ACCORDING TO INTERNAL WHEEL SPEED
        }
        else {
            Agent1.push_back(0);
        }
        if (ExtCoordCurrent01->size() == 22 && ZeroOne > 0) {
            if (sqrt(pow(ExtCoordCurrent01->at(20) - ExtCoordCurrent01->at(0), 2) +
                     pow(ExtCoordCurrent01->at(21) - ExtCoordCurrent01->at(1), 2)) > 0.037) {
                Agent1.push_back(1);
                //AGENT 1 DISTANCE TRAVELLED ACCORDING TO AGENT 0
            }
            else {
                Agent1.push_back(0);
            }
        }
        else {
            Agent1.push_back(8);
        }
        if (ExtCoordCurrent21->size() == 22 && TwoOne > 0) {
            if (sqrt(pow(ExtCoordCurrent21->at(20) - ExtCoordCurrent21->at(0), 2) +
                     pow(ExtCoordCurrent21->at(21) - ExtCoordCurrent21->at(1), 2)) > 0.037) {
                Agent1.push_back(1);
                //AGENT 1 DISTANCE TRAVELLED ACCORDING TO AGENT 2
            }
            else {
                Agent1.push_back(0);
            }
        }
        else {
            Agent1.push_back(8);
        }
        if (sqrt(pow(std::accumulate(IntYawCoord1->begin(),IntYawCoord1->end(),0.0),2)) > 0.05) {
            Agent1.push_back(1);
            //AGENT 1 ANGULAR VELOCITY ACCORDING TO INTERNAL COORDINATES
        }
        else {
            Agent1.push_back(0);
        }
        if (sqrt(pow(RABDataCurrent1->at(9) - RABDataCurrent1->at(0),2)) > 5) {
            Agent1.push_back(1);
            //AGENT 1 ANGULAR VELOCITY ACCORDING TO INTERNAL BEARING SENSOR
        }
        else {
            Agent1.push_back(0);
        }
        if (ExtYawCoord01->size() == 10 && ZeroOne > 0) {
            if (sqrt(pow(std::accumulate(ExtYawCoord01->begin(), ExtYawCoord01->end(), 0.0), 2)) > 0.05) {
                Agent1.push_back(1);
                //AGENT 1 ANGULAR VELOCITY ACCORDING TO AGENT 0
            }
            else {
                Agent1.push_back(0);
            }
        }
        else {
            Agent1.push_back(8);
        }
        if (ExtYawCoord21->size() == 10 && TwoOne > 0) {
            if (sqrt(pow(std::accumulate(ExtYawCoord21->begin(), ExtYawCoord21->end(), 0.0), 2)) > 0.05) {
                Agent1.push_back(1);
                //AGENT 1 ANGULAR VELOCITY ACCORDING TO AGENT 2
            }
            else {
                Agent1.push_back(0);
            }
        }
        else {
            Agent1.push_back(8);
        }
        if (sqrt(pow(IntCoordCurrent1->at(20) - IntCoordCurrent1->at(0),2)+pow(IntCoordCurrent1->at(21) - IntCoordCurrent1->at(1),2)) > 0.02) {
            Agent1.push_back(1);
            //AGENT 1 SPEED ACCORDING TO INTERNAL COORDINATES
        }
        else {
            Agent1.push_back(0);
        }
        if (std::accumulate(WheelDataCurrent1->begin(),WheelDataCurrent1->end(),0.0) > 0.2) {
            Agent1.push_back(1);
            //AGENT 1 SPEED ACCORDING INTERNAL WHEEL SENSOR
        }
        else {
            Agent1.push_back(0);
        }
        if (ExtCoordCurrent01->size() == 22 && ZeroOne > 0) {
            if (sqrt(pow(ExtCoordCurrent01->at(20) - ExtCoordCurrent01->at(0), 2) +
                     pow(ExtCoordCurrent01->at(21) - ExtCoordCurrent01->at(1), 2)) > 0.02) {
                Agent1.push_back(1);
                //AGENT 1 SPEED ACCORDING TO AGENT 0
            }
            else {
                Agent1.push_back(0);
            }
        }
        else {
            Agent1.push_back(8);
        }
        if (ExtCoordCurrent21->size() == 22 && TwoOne > 0) {
            if (sqrt(pow(ExtCoordCurrent21->at(20) - ExtCoordCurrent21->at(0), 2) +
                     pow(ExtCoordCurrent21->at(21) - ExtCoordCurrent21->at(1), 2)) > 0.02) {
                Agent1.push_back(1);
                //AGENT 1 SPEED ACCORDING TO AGENT 2
            }
            else {
                Agent1.push_back(0);
            }
        }
        else {
            Agent1.push_back(8);
        }

        /// AGENT 2 ///

        if (std::accumulate(MidCurrent2->begin(), MidCurrent2->end(), 0.0) > 5) {
            Agent2.push_back(1);
            //AGENT 2 #MID-PROX NEIGHBOURS ACCORDING TO SELF
        }
        else {
            Agent2.push_back(0);
        }
        if (sqrt(pow(IntCoordCurrent2->at(20) - IntCoordCurrent0->at(0),2)+pow(IntCoordCurrent2->at(21) - IntCoordCurrent0->at(1),2)) < 0.6 ||
            sqrt(pow(IntCoordCurrent2->at(20) - IntCoordCurrent1->at(0),2)+pow(IntCoordCurrent2->at(21) - IntCoordCurrent1->at(1),2)) < 0.6) {
            Agent2.push_back(1);
        }
        else {
            Agent2.push_back(0);
        }
        if (std::accumulate(ExMidCurrent02->begin(),ExMidCurrent02->end(),0.0) > -1) {
            if (std::accumulate(ExMidCurrent02->begin(), ExMidCurrent02->end(), 0.0) > 5) {
                Agent2.push_back(1);
                //AGENT 2 #MID-PROX NEIGHBOURS ACCORDING TO NEIGHBOUR 0
            }
            else {
                Agent2.push_back(0);
            }
        }
        else {
            Agent2.push_back(8);
        }
        if (std::accumulate(ExMidCurrent12->begin(),ExMidCurrent12->end(),0.0) > -1) {
            if (std::accumulate(ExMidCurrent12->begin(), ExMidCurrent12->end(), 0.0) > 5) {
                Agent2.push_back(1);
                //AGENT 2 #MID-PROX NEIGHBOURS ACCORDING TO NEIGHBOUR 1
            }
            else {
                Agent2.push_back(0);
            }
        }
        else {
            Agent2.push_back(8);
        }

        if (std::accumulate(CloseCurrent2->begin(), CloseCurrent2->end(), 0.0) > 5) {
            Agent2.push_back(1);
            //AGENT 2 #CLOSE-PROX NEIGHBOURS ACCORDING TO SELF
        }
        else {
            Agent2.push_back(0);
        }
        if (sqrt(pow(IntCoordCurrent2->at(20) - IntCoordCurrent0->at(0),2)+pow(IntCoordCurrent2->at(21) - IntCoordCurrent0->at(1),2)) < 0.27 ||
            sqrt(pow(IntCoordCurrent2->at(20) - IntCoordCurrent1->at(0),2)+pow(IntCoordCurrent2->at(21) - IntCoordCurrent1->at(1),2)) < 0.27) {
            Agent2.push_back(1);
        }
        else {
            Agent2.push_back(0);
        }
        if (std::accumulate(ExCloseCurrent02->begin(),ExCloseCurrent02->end(),0.0) > -1) {
            if (std::accumulate(ExCloseCurrent02->begin(), ExCloseCurrent02->end(), 0.0) > 5) {
                Agent2.push_back(1);
                //AGENT 2 #CLOSE-PROX NEIGHBOURS ACCORDING TO NEIGHBOUR 0
            }
            else {
                Agent2.push_back(0);
            }
        }
        else {
            Agent2.push_back(8);
        }
        if (std::accumulate(ExCloseCurrent12->begin(),ExCloseCurrent12->end(),0.0) >= 0) {
            if (std::accumulate(ExCloseCurrent12->begin(), ExCloseCurrent12->end(), 0.0) > 5) {
                Agent2.push_back(1);
                //AGENT 2 #CLOSE-PROX NEIGHBOURS ACCORDING TO NEIGHBOUR 1
            }
            else {
                Agent2.push_back(0);
            }
        }
        else {
            Agent2.push_back(8);
        }
        if (sqrt(pow(IntCoordCurrent2->at(20) - IntCoordCurrent2->at(0),2)+pow(IntCoordCurrent2->at(21) - IntCoordCurrent2->at(1),2)) > 0.037) {
            Agent2.push_back(1);
            //AGENT 2 DISTANCE TRAVELLED FROM INTERNAL COORDINATES
        }
        else {
            Agent2.push_back(0);
        }

        if (std::accumulate(WheelDataCurrent2->begin(),WheelDataCurrent2->end(),0.0) > 0.37) {
            Agent2.push_back(1);
            //AGENT 2 DISTANCE TRAVELLED ACCORDING TO INTERNAL WHEEL SPEED
        }
        else {
            Agent2.push_back(0);
        }
        if (ExtCoordCurrent02->size() == 22 && ZeroTwo > 0) {
            if (sqrt(pow(ExtCoordCurrent02->at(20) - ExtCoordCurrent02->at(0), 2) +
                     pow(ExtCoordCurrent02->at(21) - ExtCoordCurrent02->at(1), 2)) > 0.037) {
                Agent2.push_back(1);
                //AGENT 2 DISTANCE TRAVELLED ACCORDING TO AGENT 0
            }
            else {
                Agent2.push_back(0);
            }
        }
        else {
            Agent2.push_back(8);
        }
        if (ExtCoordCurrent12->size() == 22 && OneTwo > 0) {
            if (sqrt(pow(ExtCoordCurrent12->at(20) - ExtCoordCurrent12->at(0), 2) +
                     pow(ExtCoordCurrent12->at(21) - ExtCoordCurrent12->at(1), 2)) > 0.037) {
                Agent2.push_back(1);
                //AGENT 2 DISTANCE TRAVELLED ACCORDING TO AGENT 1
            }
            else {
                Agent2.push_back(0);
            }
        }
        else {
            Agent2.push_back(8);
        }
        if (sqrt(pow(std::accumulate(IntYawCoord2->begin(),IntYawCoord2->end(),0.0),2)) > 0.05) {
            Agent2.push_back(1);
            //AGENT 2 ANGULAR VELOCITY ACCORDING TO INTERNAL COORDINATES
        }
        else {
            Agent2.push_back(0);
        }
        if (sqrt(pow(RABDataCurrent2->at(9) - RABDataCurrent2->at(0),2)) > 5) {
            Agent2.push_back(1);
            //AGENT 2 ANGULAR VELOCITY ACCORDING TO INTERNAL BEARING SENSOR
        }
        else {
            Agent2.push_back(0);
        }
        if (ExtYawCoord02->size() == 10 && ZeroTwo > 0) {
            if (sqrt(pow(std::accumulate(ExtYawCoord02->begin(), ExtYawCoord02->end(), 0.0), 2)) > 0.05) {
                Agent2.push_back(1);
                //AGENT 2 ANGULAR VELOCITY ACCORDING TO AGENT 0
            }
            else {
                Agent2.push_back(0);
            }
        }
        else {
            Agent2.push_back(8);
        }
        if (ExtYawCoord12->size() == 10 && OneTwo > 0) {
            if (sqrt(pow(std::accumulate(ExtYawCoord12->begin(), ExtYawCoord12->end(), 0.0), 2)) > 0.05) {
                Agent2.push_back(1);
                //AGENT 2 ANGULAR VELOCITY ACCORDING TO AGENT 1
            }
            else {
                Agent2.push_back(0);
            }
        }
        else {
            Agent2.push_back(8);
        }
        if (sqrt(pow(IntCoordCurrent2->at(20) - IntCoordCurrent2->at(0),2)+pow(IntCoordCurrent2->at(21) - IntCoordCurrent2->at(1),2)) > 0.02) {
            Agent2.push_back(1);
            //AGENT 2 SPEED ACCORDING TO INTERNAL COORDINATES
        }
        else {
            Agent2.push_back(0);
        }
        if (std::accumulate(WheelDataCurrent2->begin(),WheelDataCurrent2->end(),0.0) > 0.2) {
            Agent2.push_back(1);
            //AGENT 2 SPEED ACCORDING INTERNAL WHEEL SENSOR
        }
        else {
            Agent2.push_back(0);
        }
        if (ExtCoordCurrent02->size() == 22 && ZeroTwo > 0) {
            if (sqrt(pow(ExtCoordCurrent02->at(20) - ExtCoordCurrent02->at(0), 2) +
                     pow(ExtCoordCurrent02->at(21) - ExtCoordCurrent02->at(1), 2)) > 0.02) {
                Agent2.push_back(1);
                //AGENT 2 SPEED ACCORDING TO AGENT 0
            }
            else {
                Agent2.push_back(0);
            }
        }
        else {
            Agent2.push_back(8);
        }
        if (ExtCoordCurrent12->size() == 22 && OneTwo > 0) {
            if (sqrt(pow(ExtCoordCurrent12->at(20) - ExtCoordCurrent12->at(0), 2) +
                     pow(ExtCoordCurrent12->at(21) - ExtCoordCurrent12->at(1), 2)) > 0.02) {
                Agent2.push_back(1);
                //AGENT 2 SPEED ACCORDING TO AGENT 1
            }
            else {
                Agent2.push_back(0);
            }
        }
        else {
            Agent2.push_back(8);
        }


        std::cout << "AGENT0 FV: " << std::endl;
        for (int i = 0; i < Agent0.size(); i++ ) {
            std::cout << Agent0[i] << std::endl;
            //agent0 << Agent0[i] << " \n";
        }
        std::cout << "AGENT1 FV: " << std::endl;
        for (int i = 0; i < Agent1.size(); i++ ) {
            std::cout << Agent1[i] << std::endl;
            //agent1 << Agent1[i] << "\n";
        }
        std::cout << "AGENT2 FV: " << std::endl;
        for (int i = 0; i < Agent2.size(); i++ ) {
            std::cout << Agent2[i] << std::endl;
            //agent1 << Agent2[i] << "\n";
        }
        /*std::cout << "AGENTs: " << std::endl;
        for (int i = 0; i < Agent.size(); i++ ) {
            std::cout << Agent[i] << std::endl;
            //agent1 << Agent2[i] << "\n";
        }*/


        /*for (int i = 0; i < IntYawCoord1->size(); i++ ) {
            std::cout << IntYawCoord1->at(i) << std::endl;
            //out << Agent1[i] << "\n";
        }*/
        //std::cout << "TESTING BUFFER SUM = " << std::accumulate(WheelDataCurrent1->begin(),WheelDataCurrent1->end(),0.0) << std::endl;
        //out << Agent0 << Agent1;
        /*if (GetSpace().GetSimulationClock() > 2032) {
            agent0.close();
            agent1.close();
        }*/

    }
    /*for (int i = 0; i < IntCoordCurrent->size(); i++ ) {
        std::cout << i << std::endl;
        std::cout << IntCoordCurrent->at(i) << std::endl;

    }*/
    if (YawHolder->size() == 2) {
        IntYawCoord->push_back(YawHolder->at(1) - YawHolder->at(0));


        IntYawCoord0->push_back(Yaw_Coordinates_0a_Internal - Yaw_Coordinates_0b_Internal);
        if (OneZero > 0) {
            ExtYawCoord10->push_back(Yaw_Coordinates_0a_From_1 - Yaw_Coordinates_0b_From_1);
        }
        if (TwoZero > 0) {
            ExtYawCoord20->push_back(Yaw_Coordinates_0a_From_2 - Yaw_Coordinates_0b_From_2);
        }
        IntYawCoord1->push_back(Yaw_Coordinates_1a_Internal - Yaw_Coordinates_1b_Internal);
        if (ZeroOne > 0) {
            ExtYawCoord01->push_back(Yaw_Coordinates_1a_From_0 - Yaw_Coordinates_1b_From_0);
        }
        if (TwoOne > 0) {
            ExtYawCoord21->push_back(Yaw_Coordinates_1a_From_2 - Yaw_Coordinates_1b_From_2);
        }
        IntYawCoord2->push_back(Yaw_Coordinates_2a_Internal - Yaw_Coordinates_2b_Internal);
        if (ZeroTwo > 0) {
            ExtYawCoord02->push_back(Yaw_Coordinates_2a_From_0 - Yaw_Coordinates_2b_From_0);
        }
        if (OneTwo > 0) {
            ExtYawCoord12->push_back(Yaw_Coordinates_2a_From_1 - Yaw_Coordinates_2b_From_1);
        }
    }
    Yaw_Coordinates_0b_Internal = Yaw_Coordinates_0a_Internal;
    Yaw_Coordinates_1b_Internal = Yaw_Coordinates_1a_Internal;
    Yaw_Coordinates_2b_Internal = Yaw_Coordinates_2a_Internal;
    if (OneZero > 0) {
        Yaw_Coordinates_0b_From_1 = Yaw_Coordinates_0a_From_1;
    }
    if (TwoZero > 0) {
        Yaw_Coordinates_0b_From_2 = Yaw_Coordinates_0a_From_2;
    }
    if (ZeroOne > 0) {
        Yaw_Coordinates_1b_From_0 = Yaw_Coordinates_1a_From_0;
    }
    if (TwoOne > 0) {
        Yaw_Coordinates_1b_From_2 = Yaw_Coordinates_1a_From_2;
    }
    if (ZeroTwo > 0) {
        Yaw_Coordinates_2b_From_0 = Yaw_Coordinates_2a_From_0;
    }
    if (OneTwo > 0) {
        Yaw_Coordinates_2b_From_1 = Yaw_Coordinates_2a_From_1;
    }



}



void LoggingLoopFunctions::PostStep() {





}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(LoggingLoopFunctions, "logging_loop_functions")

