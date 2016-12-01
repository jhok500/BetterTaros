#include "logging_loop_functions.h"

#include <controllers/footbot_diffusion/footbot_diffusion.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include <iostream>
#include <fstream>

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
    IntYawCoord = new boost::circular_buffer<double>(10*3);

    WheelDataCurrent = new boost::circular_buffer<double>(10*3);
    RABDataCurrent = new boost::circular_buffer<double>(10*3);
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
    Agent.clear();
    AgentExt.clear();
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
        Real Velocity_Wheels = 0.5*(cController.GetSpoofLeftWheelVelocity() + cController.GetSpoofRightWheelVelocity());
        CCI_RangeAndBearingSensor::TReadings packets = cController.GetRABSensorReadings();



        std::string OriginID = cFootBot.GetRootEntity().GetId().c_str();
        int RealID = OriginID.at(2) + 32;
        cPosInternalA.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        int MidProxCoord = 0;
        int CloseProxCoord = 0;
        int MidProx = 0;
        int CloseProx = 0;
        Indices.clear();
        Indices1.clear();
        Indices4.clear();

        AgentBearing_Sensor = yaw.GetValue()*180/ARGOS_PI;
        Agent.push_back(RealID);

        YawSensor->push_back(RealID);
        YawSensor->push_back(AgentBearing_Sensor);

        //////////////// EXT

        for(CCI_RangeAndBearingSensor::SPacket packet : packets)
        {


            int MidProxExt = 0;
            int CloseProxExt = 0;
            Indices2.clear();
            Indices3.clear();
            std::string TargetID = "fb" + std::to_string(packet.Data[0]);
            int NeighbourID = packet.Data[0] + 80;
            AgentExt.push_back(RealID);
            AgentExt.push_back(NeighbourID);
            ExtCoord->push_back(RealID);
            ExtCoord->push_back(NeighbourID);

            MidProx = MidProx + 1;
            MidProxCoord = MidProxCoord + 1;
            MidProxExt = MidProxExt + 1;
            AgentExt.push_back(MidProxExt);

            if(packet.Range < 29) {
                CloseProx = CloseProx+1;
                CloseProxExt = CloseProxExt+1;
            }
            AgentExt.push_back(CloseProxExt);
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

            Real TrueRange = packet.Range/100;
            //std::cout << RealID << " Range & angle" << TrueRange << ", " << bearing << ", " << TrueBearing << " " << NeighbourID << std::endl;
            NeighbourDistance.Set(TrueRange*cos(ARGOS_PI*TrueBearing/180),TrueRange*sin(ARGOS_PI*TrueBearing/180));
            if (sqrt(pow(NeighbourDistance.GetX(),2)+pow(NeighbourDistance.GetY(),2)) < 0.29) {
                CloseProxCoord = CloseProxCoord+1;
            }

            ExtCoord->push_back(cPosInternalA.GetX() + NeighbourDistance.GetX());
            ExtCoord->push_back(cPosInternalA.GetY() + NeighbourDistance.GetY());

            for (int i = 0; i < ExtCoord->size(); i++) {
                //std::cout << ExtCoord->at(i) << std::endl;
                if (ExtCoord->at(i) == RealID && ExtCoord->at(i+1) == NeighbourID) {
                    Indices2.push_back(i);

                }
            }
            //std::cout << Indices2.size() << std::endl;
            if (Indices2.size() >= 2) {

                Real NeighbourDistanceTravelled = sqrt(pow(((ExtCoord->at(Indices2.front() + 2)) - (ExtCoord->at(Indices2[1] + 2))), 2)
                                                       + pow(((ExtCoord->at(Indices2.front() + 3)) - (ExtCoord->at(Indices2[1] + 3))), 2));
                Real NeighbourSpeed = NeighbourDistanceTravelled * 10;
                AgentExt.push_back(NeighbourDistanceTravelled);
                AgentExt.push_back(NeighbourSpeed);
                //std::cout << RealID << " thinks that " << NeighbourID << " is travelling " << NeighbourDistanceTravelled << std::endl;
                YawHolder1->push_back(RealID);
                YawHolder1->push_back(NeighbourID);
                YawHolder1->push_back(atan2((ExtCoord->at(Indices2.front()+2)-(ExtCoord->at(Indices2[1]+2))),
                                            (ExtCoord->at(Indices2.front() + 3) - (ExtCoord->at(Indices2[1] + 3)))));
                for (int i = 0; i < YawHolder1->size(); i++) {
                    if (YawHolder1->at(i) == RealID && YawHolder1->at(i+1) == NeighbourID) {
                        Indices3.push_back(i);
                        //std::cout << "CORRECT " << RealID << " YAW AT " << i << std::endl;
                    }
                }
                if (Indices3.size() >= 2) {
                    Real NeighbourYawCoord = (YawHolder1->at(Indices3.front()+2) - YawHolder1->at(Indices3[1] + 2))*180/ARGOS_PI;
                    if (sqrt(pow(NeighbourYawCoord,2)) < 0.001) {
                        NeighbourYawCoord = 0;
                    }
                    AgentExt.push_back(NeighbourYawCoord);
                    //std::cout << RealID << " thinks that " << NeighbourID << " is turning " << NeighbourYawCoord << std::endl;
                }
            }

            //std::cout << RealID << "  " << packet.Range << std::endl;
        }

        //////////////// EXT

        Agent.push_back(CloseProxCoord);
        Agent.push_back(CloseProx);
        Agent.push_back(MidProxCoord);
        Agent.push_back(MidProx);


        if (IntCoord->size() == IntCoord->capacity()) {
            for (int i = 0; i < IntCoord->size(); i++) {
                //std::cout << IntCoord->at(i) << std::endl;
                if (IntCoord->at(i) == RealID) {
                    Indices.push_back(i);
                }
            }
            Real CoordDistance = sqrt(pow(((IntCoord->at(Indices.front() + 1)) - (IntCoord->at(Indices.back() + 1))), 2)
                                 + pow(((IntCoord->at(Indices.front() + 2)) - (IntCoord->at(Indices.back() + 2))), 2));
            Real CoordSpeed = CoordDistance * 10 / (Indices.size() - 1);
            Agent.push_back(CoordDistance);
            Agent.push_back((Velocity_Wheels/10)/100);
            Agent.push_back(CoordSpeed);
            Agent.push_back(Velocity_Wheels/100);
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
                Agent.push_back(IntYawCoord*180/ARGOS_PI);
            }
            if (YawSensor->size() == YawSensor->capacity()) {
                for (int i = 0; i < YawSensor->size(); i++) {
                    if (YawSensor->at(i) == RealID) {
                        Indices4.push_back(i);
                    }
                }
                Real AngularVelocity = (YawSensor->at(Indices4.front()+1) - YawSensor->at(Indices4.back()+1));
                Agent.push_back(AngularVelocity);
            }
        }


        IntCoord->push_back(RealID);
        IntCoord->push_back(cPosInternalA.GetX());
        IntCoord->push_back(cPosInternalA.GetY());












    }

    /*std::cout << "AGENTs: " << std::endl;
    for (int i = 0; i < Agent.size(); i++ ) {
        std::cout << Agent[i] << std::endl;
    }*/
    /*std::cout << "AGENTs (external): " << std::endl;
    for (int i = 0; i < AgentExt.size(); i++ ) {
        std::cout << AgentExt[i] << std::endl;
    }*/












}



void LoggingLoopFunctions::PostStep() {





}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(LoggingLoopFunctions, "logging_loop_functions")

