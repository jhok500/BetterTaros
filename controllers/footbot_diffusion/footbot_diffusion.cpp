/* Include the controller definition */
#include "footbot_diffusion.h"

#include "loop_functions/logging_loop_functions/logging_loop_functions.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
int Time = 0;


CFootBotDiffusion::CFootBotDiffusion() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}


/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   wheel_encoders = GetSensor  <CCI_DifferentialSteeringSensor      >("differential_steering"    );
   range_and_bearing_actuator = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
   range_and_bearing_sensor = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
   std::string robot_name = GetId();
   robot_name = robot_name.substr(2, robot_name.size());
   int id = atoi(robot_name.c_str());
   range_and_bearing_actuator->SetData(0, id);





   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
    /****************************************/
    FlockData0 = new boost::circular_buffer<double>(1);
    FlockData1 = new boost::circular_buffer<double>(1);
    FlockData2 = new boost::circular_buffer<double>(1);
    FlockCoordData0 = new boost::circular_buffer<double>(2);
    FlockCoordData1 = new boost::circular_buffer<double>(2);
    FlockCoordData2 = new boost::circular_buffer<double>(2);



}

/****************************************/
/****************************************/
/*void CFootBotDiffusion::FindYaw(const LoggingLoopFunctions &a) {
    y = a.AgentBearing_Sensor;
    std::cout << "TESTING CONF : " << y << std::endl;
    return;

}*/

void CFootBotDiffusion::ControlStep() {
    Time = Time+1;
    //std::cout << "time is " << Time << std::endl;
    /* Get readings from proximity sensor */
    const CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
    /* Sum them together */
    CVector2 cAccumulator;
    for (size_t i = 0; i < tProxReads.size(); ++i) {
        cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }
    cAccumulator /= tProxReads.size();
    /* If the angle of the vector is small enough and the closest obstacle
     * is far enough, continue going straight, otherwise curve a little
     */
    //CRadians cAngle = cAccumulator.Angle();
    const CCI_RangeAndBearingSensor::TReadings &packets = range_and_bearing_sensor->GetReadings();

    cAngle = cAccumulator.Angle();

    CFootBotEntity &entity = dynamic_cast<CFootBotEntity &>(CSimulator::GetInstance().GetSpace().GetEntity(GetId()));

    CRadians yaw, pitch, roll;
    //CFootBotEntity entity;
    entity.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(yaw, pitch, roll);
    Real Heading = yaw.GetValue() * 180 / ARGOS_PI;
    Real X = entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
    Real Y = entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
    //std::cout << "TEST YAW: " << Heading << std::endl;

    //std::cout << entity.GetEmbodiedEntity().GetOriginAnchor().Orientation << std::endl;



    //FindYaw();
    //std::cout << "TESTING CONF : " << LoggingLoopFunctions::ExportYaw << std::endl;




    /*if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
       cAccumulator.Length() < m_fDelta ) {
        //Go straight
        Prox = 0;
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    }
    else {
        //Turn, depending on the sign of the angle
        Prox = 1;
        if(cAngle.GetValue() > 0.0f) {
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
        }
        else {
            m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
        }
    }*/

    //DIFFUSION


    int Move = 0;
    NoiseLeft = 0.0f;
    NoiseRight = 0.0f;

    /*if ( GetId() == "fb3") {
        if (Time > 1000) {
            NoiseLeft = -m_fWheelVelocity;
            NoiseRight = -m_fWheelVelocity;
        }
        else {
            NoiseLeft = 0.0f;
            NoiseRight = 0.0f;
        }
    }
    else {
        NoiseLeft = 0.0f;
        NoiseRight = 0.0f;
    }*/
    if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
        cAccumulator.Length() < m_fDelta) {


        for (CCI_RangeAndBearingSensor::SPacket packet : packets) {

            double bearing = ToDegrees(packet.HorizontalBearing).GetValue();
            double range = packet.Range;
            if (range > 30) {
                LeftWheel = m_fWheelVelocity;
                RightWheel = m_fWheelVelocity;
                m_pcWheels->SetLinearVelocity(m_fWheelVelocity+NoiseLeft, m_fWheelVelocity+NoiseRight);
            }
            else {
                //Turn, depending on the sign of the angle
                if (bearing > 0.0f) {
                    LeftWheel = m_fWheelVelocity;
                    RightWheel = 0;
                    m_pcWheels->SetLinearVelocity(m_fWheelVelocity+NoiseLeft, 0.0f);
                    //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity);
                }
                else {
                    LeftWheel = 0;
                    RightWheel = m_fWheelVelocity;
                    m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity+NoiseRight);
                    //m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, m_fWheelVelocity);
                }
                Move = 1;
            }
        }
    }

        else {
            //Turn, depending on the sign of the angle
            if (cAngle.GetValue() > 0.0f) {
                LeftWheel = m_fWheelVelocity;
                RightWheel = 0;
                m_pcWheels->SetLinearVelocity(m_fWheelVelocity+NoiseLeft, 0.0f);
            }
            else {
                LeftWheel = 0;
                RightWheel = m_fWheelVelocity;
                m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity+NoiseRight);
            }
            Move = 1;
        }

        if (Move == 0) {
            LeftWheel = m_fWheelVelocity;
            RightWheel = m_fWheelVelocity;
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity+NoiseLeft, m_fWheelVelocity+NoiseRight);
        }

    }

/*    //FLOCKING
    if (GetId() == "fb0") {
        FlockData0->push_back(Heading);
        FlockCoordData0->push_back(X);
        FlockCoordData0->push_back(Y);
    }
    else if (GetId() == "fb1") {
        FlockData1->push_back(Heading);
        FlockCoordData1->push_back(X);
        FlockCoordData1->push_back(Y);
    }
    else {
        FlockData2->push_back(Heading);
        FlockCoordData2->push_back(X);
        FlockCoordData2->push_back(Y);
    }

    int Move = 0;
    for (CCI_RangeAndBearingSensor::SPacket packet : packets) {
        double bearing = ToDegrees(packet.HorizontalBearing).GetValue();
        double range = packet.Range;
        Move = 1;
        if (FlockData0->size() > 0 && FlockData1->size() > 0 && FlockData2->size() > 0 && FlockCoordData0->size() > 1 &&
            FlockCoordData1->size() > 1 && FlockCoordData2->size() > 1) {
            //std::cout << "ROLLING" << std::endl;
            if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
                cAccumulator.Length() < m_fDelta) {
                //Go straight
                Prox = 0;

                if (range > 50) {
                    if (sqrt(pow(bearing, 2)) < 15) {
                        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
                    }
                    else {
                        if (bearing < 0.0f) {
                            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
                            //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity);
                        }
                        else {
                            m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
                            //m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, m_fWheelVelocity);
                        }
                    }

                }
                else {

                    if (Heading < ((FlockData0->at(0) + FlockData1->at(0) + FlockData2->at(0))/3) + 5 &&
                        Heading > ((FlockData0->at(0) + FlockData1->at(0) + FlockData2->at(0))/3) - 5) {
                        //if (range > 40) {
                        //Go straight
                        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
                    }
                    else {
                        //Turn, depending on the sign of the angle
                        if (Heading > (FlockData0->at(0) + FlockData1->at(0) + FlockData2->at(0))/3) {
                            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
                            //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity);
                        }
                        else {
                            m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
                            //m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, m_fWheelVelocity);
                        }
                    }
                }


            }
            else {
                //Turn, depending on the sign of the angle
                if (cAngle.GetValue() > 0.0f) {
                    m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
                }
                else {
                    m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
                }
            }
        }
    }
    if (Move == 0) {
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    }
}*/


    //

    //AGGREGATION
    /*int Move = 0;
    for (CCI_RangeAndBearingSensor::SPacket packet : packets) {
        double bearing = ToDegrees(packet.HorizontalBearing).GetValue();
        double range = packet.Range;
        //std::cout << "BEARING IS " << bearing << std::endl;
        Move = 1;
        std::cout << "I AM " << GetId() << " SENSING " << packet.Data[0] << std::endl;
        //if (GetId() == "fb0") {
        if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
            cAccumulator.Length() < m_fDelta) {
            //Go straight
            if (range > 30) {
                if (sqrt(pow(bearing, 2)) < 15) {
                    //if (range > 40) {
                    //Go straight
                    m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
                }
                else {
                    //Turn, depending on the sign of the angle
                    if (bearing < 0.0f) {
                        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
                        //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity);
                    }
                    else {
                        m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
                        //m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, m_fWheelVelocity);
                    }
                }
            }
            else {
                if (bearing > 0.0f) {
                    m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
                    //std::cout << GetId() << "  IS TURNING RIGHT BECAUSE BEARING IS  " << bearing << std::endl;
                    //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity);
                }
                else {
                    m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
                    //std::cout << GetId() << "  IS TURNING LEFT BECAUSE BEARING IS  " << bearing << std::endl;
                    //m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, m_fWheelVelocity);
                }
            }
        }
        else {
            //Turn, depending on the sign of the angle
            Prox = 1;
            if (cAngle.GetValue() > 0.0f) {
                m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
            }
            else {
                m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
            }
        }
    }
    if (Move == 0) {
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    }
}*/




CCI_RangeAndBearingSensor::TReadings CFootBotDiffusion::GetRABSensorReadings()
{
   // Get RAB packets from other robots within range
   const CCI_RangeAndBearingSensor::TReadings& packets = range_and_bearing_sensor->GetReadings();

   return packets;
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")
