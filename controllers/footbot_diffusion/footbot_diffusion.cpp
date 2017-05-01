/* Include the controller definition */
#include "footbot_diffusion.h"

#include "loop_functions/logging_loop_functions/logging_loop_functions.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
int Time = 0;
int RobotNumber = 10;


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
    FlockData = new boost::circular_buffer<double>(2*RobotNumber);
    FlockCoordData = new boost::circular_buffer<double>(3*RobotNumber);
    PingWait = new boost::circular_buffer<int>(10);
    StopWait = new boost::circular_buffer<int>(10);
    RABWait = new boost::circular_buffer<int>(10);
    MotorWait = new boost::circular_buffer<int>(10);
    StraightWait = new boost::circular_buffer<int>(10);
    LapWait = new boost::circular_buffer<int>(10);






}

/****************************************/
/****************************************/
/*void CFootBotDiffusion::FindYaw(const LoggingLoopFunctions &a) {
    y = a.AgentBearing_Sensor;
    std::cout << "TESTING CONF : " << y << std::endl;
    return;

}*/

void CFootBotDiffusion::ControlStep() {
    if (timeweight == 0) {
        Time = Time + 1;
    }

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
    std::string IDraw = GetId();
    int ID = IDraw.at(2) + 32;
    entity.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(yaw, pitch, roll);
    unsigned seed = Time;
    std::default_random_engine generator(seed);
    std::normal_distribution<double> YawNoise(0,1);
    std::normal_distribution<double> XNoise(0,0.00001);
    std::normal_distribution<double> YNoise(0,0.00001);
    std::normal_distribution<double> RABNoise(0,1);

    Real Heading = (yaw.GetValue() * 180 / ARGOS_PI) + YawNoise(generator);
    if (Heading > 180) {
        Heading = -180 + (Heading-180);
    }
    if (Heading < -180) {
        Heading = 180 - (abs(Heading)-180);
    }
    //std::cout << "Heading = " << ID << ", " << Heading << std::endl;
    Real X = entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX() + XNoise(generator);
    Real Y = entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY() + YNoise(generator);

    //std::cout << "Time, X, Y = " << Time << ", " << XNoise(generator) << ", " << YNoise(generator) << std::endl;
    int Move = 0;
    NoiseLeft = 0.0f;
    NoiseRight = 0.0f;
    // W-ALGORITHM

    //std::cout << "ROLLING" << ID << std::endl;

    FlockData->push_back(ID);
    FlockData->push_back(Heading);
    FlockCoordData->push_back(ID);
    FlockCoordData->push_back(X);
    FlockCoordData->push_back(Y);
    FlockRange.clear();
    FlockBearing.clear();
    FlockID.clear();
    FlockHeadings.clear();
    FlockCoords.clear();
    AggX.clear();
    AggY.clear();
    int Danger = 0;
    int Alone = 1;
    int Left = 0;
    int Right = 0;
    int DistantNeighbours = 0;
    int CloseNeighbours = 0;

    /*for (int i = 0; i < FlockData->size(); i++ ) {
        std::cout << FlockData->at(i) << std::endl;
    }*/

    // AGGREGATION

    /*if (FlockData->size() == FlockData->capacity() && FlockCoordData->size() == FlockCoordData->capacity()) {
        //std::cout << FlockCoordData->capacity() << ", " << FlockCoordData->size() << std::endl;
        if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
            cAccumulator.Length() < m_fDelta) {

            for (CCI_RangeAndBearingSensor::SPacket packet : packets) {
                Alone = 0;
                double bearing = ToDegrees(packet.HorizontalBearing).GetValue();
                // Partial Sensor Failure //
                if (ID == 83 && Time > 500 && abs(bearing) > 45 && Fault == 6) {
                    Alone = 1;
                    //std::cout << "DANGER" << std::endl;
                }
                else {
                    FlockBearing.push_back(bearing);
                    double range = packet.Range + RABNoise(generator);
                    FlockRange.push_back(range);
                    FlockID.push_back(packet.Data[0] + 80);
                    //std::cout << ID << ", " << bearing << ", " << range << std::endl;
                }

            }
            for (int i = 0; i < FlockRange.size(); i++) {
                if (FlockRange[i] < 30) {
                    Danger = 1;
                    if (FlockBearing[i] > 0.0f) {
                        //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
                        Left = 1;
                        Right = 0;
                    }
                    else {
                        //m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
                        Left = 0;
                        Right = 1;
                    }
                }
            }

            if (Danger == 0) {

                for (int j = 0; j < FlockCoordData->size(); j++) {
                    for (int k = 0; k < FlockID.size(); k++) {
                        if (FlockCoordData->at(j) == FlockID.at(k)) {
                            AggX.push_back(FlockCoordData->at(j + 1));
                            AggY.push_back(FlockCoordData->at(j + 2));
                        }
                    }
                }

                CVector2 GoalCoord;
                Real GoalBearing;
                GoalCoord.Set((std::accumulate(AggX.begin(), AggX.end(), 0.0) / AggX.size()),
                              (std::accumulate(AggY.begin(), AggY.end(), 0.0) / AggY.size()));
                GoalBearing = atan2(GoalCoord.GetY() - Y, GoalCoord.GetX() - X) * 180 / ARGOS_PI;

                if (abs(Heading) < abs(GoalBearing) + 5 && abs(Heading) > abs(GoalBearing) - 5) {
                    //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
                    Left = 1;
                    Right = 1;
                }
                else {
                    if (Heading < GoalBearing) {
                        //m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
                        Left = 0;
                        Right = 1;
                    }
                    else {
                        //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
                        Left = 1;
                        Right = 0;
                    }
                }
            }


            if (Alone == 1) {
                //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
                Left = 1;
                Right = 1;
            }
        }

        else {
            if (cAngle.GetValue() > 0.0f) {
                //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
                Left = 1;
                Right = 0;
            }
            else {
                //m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
                Left = 0;
                Right = 1;
            }
        }
    }*/




    // FLOCKING//
    /*if (FlockData->size() == FlockData->capacity() && FlockCoordData->size() == FlockCoordData->capacity()) {
        if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
            cAccumulator.Length() < m_fDelta) {

            for (CCI_RangeAndBearingSensor::SPacket packet : packets) {
                Alone = 0;
                double bearing = ToDegrees(packet.HorizontalBearing).GetValue();
                // Partial Sensor Failure //
                if (ID == 83 && Time > 500 && abs(bearing) > 45 && Fault == 6) {
                    Alone = 1;
                    //std::cout << "DANGER" << std::endl;
                }
                else {
                    FlockBearing.push_back(bearing);
                    double range = packet.Range + RABNoise(generator);
                    FlockRange.push_back(range);
                    FlockID.push_back(packet.Data[0] + 80);
                    //std::cout << ID << ", " << bearing << ", " << range << std::endl;
                }

            }
            for (int i = 0; i < FlockRange.size(); i++) {
                if (FlockRange[i] < 30) {
                    Danger = 1;
                    if (FlockBearing[i] > 0.0f) {
                        //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
                        Left = 1;
                        Right = 0;
                    }
                    else {
                        //m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
                        Left = 0;
                        Right = 1;
                    }
                }
            }

            if (Danger == 0) {

                for (int j = 0; j < FlockData->size(); j++) {
                    for (int k = 0; k < FlockID.size(); k++) {
                        if (FlockData->at(j) == FlockID.at(k)) {
                            FlockHeadings.push_back(FlockData->at(j+1));
                        }
                    }
                }
                Real Goal;
                Real GoalHeading = std::accumulate(FlockHeadings.begin(), FlockHeadings.end(),0.0)/FlockHeadings.size();
                Real GoalBearing = std::accumulate(FlockBearing.begin(), FlockBearing.end(), 0.0)/FlockBearing.size();
                if (std::accumulate(FlockRange.begin(), FlockRange.end(), 0.0)/FlockRange.size() > (30 + (15*FlockRange.size()))) {
                    Goal = GoalBearing;
                    //std::cout << ID << " Too far" << std::endl;
                    if (abs(Goal) < 15) {
                        Left = 1;
                        Right = 1;
                    }
                    else {
                        if (Goal < 0) {
                            Left = 1;
                            Right = 0;
                        }
                        else {
                            Left = 0;
                            Right = 1;
                        }
                    }
                }
                else {
                    Goal = GoalHeading;
                    //std::cout << ID << " Close Enough" << std::endl;
                    if (abs(Heading) < abs(Goal) + 15 && abs(Heading) > abs(Goal) - 15) {
                        //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
                        Left = 1;
                        Right = 1;
                    }
                    else {
                        if (Heading < Goal) {
                            //m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
                            Left = 0;
                            Right = 1;
                        }
                        else {
                            //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
                            Left = 1;
                            Right = 0;
                        }
                    }
                }

                //std::cout << ID << ", " << CloseNeighbours << ", " << Heading << ", " <<  Goal << std::endl;

            }
            if (Alone == 1) {
                //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
                Left = 1;
                Right = 1;
            }
        }

        else {
            if (cAngle.GetValue() > 0.0f) {
                //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
                Left = 1;
                Right = 0;
            }
            else {
                //m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
                Left = 0;
                Right = 1;
            }
        }
    }*/

    // OBSTACLE AVOIDANCE
    int Quarantine = 0;
    if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
        cAccumulator.Length() < m_fDelta) {
        Ambulance.clear();
        for (CCI_RangeAndBearingSensor::SPacket packet : packets) {
            Alone = 0;
            double bearing = ToDegrees(packet.HorizontalBearing).GetValue();
            // Partial Sensor Failure //


            if (ID == 83 && Time > 500 && Fault == 6) {
                if (Time > 550) {
                    FaultID = ID;
                }
                if (abs(bearing) > 45) {
                    Alone = 1;
                }
            }

            FlockBearing.push_back(bearing);
            double range = packet.Range + RABNoise(generator);
            FlockRange.push_back(range);
            FlockID.push_back(packet.Data[0] + 80);
            if (packet.Data[0] + 80 == FaultID) {
                //std::cout << ID << " is " << range << " away from faulty " << FaultID << std::endl;
                DrRobo.push_back(ID);
                DrDistance.push_back(range);
            }
            if (packet.Data[0] + 80 == DrID && range < 100 && ID != FaultID) {

                //std::cout << ID << " can see DR " << DrID << " at bearing " << bearing << std::endl;
                if (bearing > 0 && bearing < 90) {
                    Left = 1;
                    Right = 0;
                    Quarantine = 1;
                }
                else if (bearing < 0 && bearing > -90) {
                    Left = 0;
                    Right = 1;
                    Quarantine = 1;
                }
            }
            if (ID == DrID && packet.Data[0] + 80 == FaultID) {
                Stop = 1;
                Ambulance.push_back(bearing);
                Ambulance.push_back(range);
            }
            //std::cout << ID << ", " << bearing << ", " << range << std::endl;



        }

        // DECIDE ON A DR//


        if (Quarantine == 0 && ID != DrID) {
            for (int i = 0; i < FlockRange.size(); i++) {
                if (FlockRange[i] < 30) {
                    if (FlockBearing[i] > 0.0f) {
                        //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
                        Left = 1;
                        Right = 0;
                    }
                    else {
                        //m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
                        Left = 0;
                        Right = 1;
                    }
                }
                else {
                    Left = 1;
                    Right = 1;
                }
            }
            if (Alone == 1) {
                //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
                Left = 1;
                Right = 1;
            }


        }
        if (ID == DrID && Ambulance.size() > 1) {
            //std::cout << "Hello World!" << std::cout;
            //std::cout << Ambulance.at(0) << ", " << Ambulance.at(1) << std::endl;
            if (Ambulance.at(1) < 50) {
                Left = 0;
                Right = 0;
                BeginDiagnostic = 1;
            }
            else {
                if (abs(Ambulance.at(0)) < 20) {
                    Left = 1;
                    Right = 1;
                }
                else {
                    if (Ambulance.at(0) < 0) {
                        Left = 1;
                        Right = 0;
                    }
                    else {
                        Left = 0;
                        Right = 1;
                    }
                }
            }
        }
    }
    else {
        if (cAngle.GetValue() > 0.0f) {
            //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
            Left = 1;
            Right = 0;
        }
        else {
            //m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
            Left = 0;
            Right = 1;
        }
    }

    if (BeginDiagnostic > 0 && ID == DrID) {
        //std::cout << "time to start fixing 2" << std::endl;
        //PING
        if (returnping == 0) {
            ping = 1;
            PingWait->push_back(1);

        }
        else if (returnping == 1) {
            //std::cout << "RESPONSIVE" << std::endl;
            PingWait->push_back(0);

            // STOP FAULTY ID
            Stop = 1;
            if (Stopping == 1) {
                StopWait->push_back(0);
                Stop = 0;
                //std::cout << "RESPONSIVE" << std::endl;
                //COMPARE RAB
                RABCompare = 1;
                if (RABReturn == 1) {
                    RABCompare = 0;
                    //std::cout << "Sensor works for this direction" << std::endl;
                    if (ConfirmRM == 0) {
                        TestRM = 1;

                    }

                    if (ConfirmRM == 1) {
                        TestRM = 0;

                        if (ConfirmLM == 0) {
                            TestLM = 1;

                        }
                        if (ConfirmLM == 1) {
                            TestLM = 0;
                        }
                    }
                    if (std::accumulate(MotorWait->begin(), MotorWait->end(), 0.0) == MotorWait->capacity()) {
                        std::cout << "CMF" << std::endl;
                        Diagnosis = 4;
                    }
                    if (ConfirmLM == 1 && ConfirmRM == 1) {
                        if (ConfirmStraight == 0) {
                            TestStraight = 1;
                        }
                        if (ConfirmStraight == 1) {
                            TestStraight = 0;
                            if (ConfirmLap == 0) {
                                TestLap = 1;
                            }
                            if (ConfirmLap == 1) {
                                TestLap = 0;
                            }
                            if (std::accumulate(LapWait->begin(), LapWait->end(), 0.0) == LapWait->capacity()) {
                                std::cout << "PSF" << std::endl;
                                Diagnosis = 6;
                            }
                        }
                        if (std::accumulate(StraightWait->begin(), StraightWait->end(), 0.0) == StraightWait->capacity()) {
                            std::cout << "PMF" << std::endl;
                            Diagnosis = 5;
                        }


                        // LAP & COMPARE
                    }
                }
                else if (std::accumulate(RABWait->begin(), RABWait->end(), 0.0) == RABWait->capacity()) {
                    std::cout << "CSF DIAGNOSED" << std::endl;
                    Diagnosis = 3;
                }

            }
            else if (Stop == 1 && Stopping == 0) {
                StopWait->push_back(1);
                if (std::accumulate(StopWait->begin(), StopWait->end(), 0.0) == StopWait->capacity()) {
                    std::cout << "STOP UNRESPONSIVE, KILL FAULTY ID" << std::endl;
                }
            }


        }
        if (std::accumulate(PingWait->begin(), PingWait->end(), 0.0) == PingWait->capacity()) {

            std::cout << "PING UNRESPONSIVE, KILL FAULTY ID" << std::endl;
        }

    }
    if (BeginDiagnostic > 0 && ID == FaultID && hang == 0 && power == 0) {
        returnping = 1;
        ping = 0;
        if (Stop = 1) {
            Left = 0;
            Right = 0;
            Stopping = 1;
            if (RABCompare == 1) {
                if (Fault == 3) {
                    RABWait->push_back(1);
                }
                else {
                    RABReturn = 1;
                    RABWait->push_back(0);
                }
            }
            else if (TestRM == 1) {
                Right = 1;
                ConfirmRM = 1;
                MotorWait->push_back(0);


            }
            else if (TestLM == 1) {
                if (Fault == 4) {
                    MotorWait->push_back(1);
                }
                else {
                    Left = 1;
                    ConfirmLM = 1;
                    MotorWait->push_back(0);
                }
            }
            else if (TestStraight == 1) {
                if (Fault == 5) {
                    StraightWait->push_back(1);
                }
                else {
                    Left = 1;
                    Right = 1;
                    ConfirmStraight = 1;
                    StraightWait->push_back(0);
                }
            }
            else if (TestLap == 1) {
                if (Fault == 6) {
                    LapWait->push_back(1);
                }
                else {
                    Left = 1;
                    ConfirmLap;
                    LapWait->push_back(0);
                }
            }
        }
    }


    RightWheel = (m_fWheelVelocity*Right);
    LeftWheel = (m_fWheelVelocity*Left);
    NoiseLeft = 0;
    NoiseRight = 0;
    if (ID == DrID ) {
        //NoiseLeft = -LeftWheel;
        //NoiseRight = -RightWheel;
    }
    if (ID == FaultID && Stopping == 1) {

        RightWheel = 0;
        LeftWheel = 0;


    }
    // MOTOR FAULTS //
    if (ID == 83 && Time > 500 && Fault == 4) {
        NoiseLeft = -LeftWheel;
        if (Time > 550) {
            FaultID = ID;
        }
        //hang = 1;
        //std::cout << FaultID << std::endl;
        //NoiseRight = -RightWheel;
    }
    if (ID == 83 && Time > 500 && Fault == 5) {
        NoiseLeft = -0.5*LeftWheel;
        if (Time > 550) {
            FaultID = ID;
        }
        //hang = 1;
        //std::cout << FaultID << std::endl;
        //NoiseRight = -RightWheel;
    }
    // COMPLETE SENSOR FAULT //
    if (ID == 83 && Time > 500 && Fault == 3) {
        if (Time > 550) {
            FaultID = ID;
        }
        RightWheel = m_fWheelVelocity;
        LeftWheel = m_fWheelVelocity;
    }
    // POWER FAILURE //
    if (ID == 83 && Time > 500 && Fault == 2) {
        FaultID = ID;
        power = 1;
        RightWheel = 0;
        LeftWheel = 0;
    }


    // SOFTWARE HANG //
    if (ID == 83 && Time > 500 && Fault == 1) {
        if (Time > 550) {
            FaultID = ID;
        }
        if (hang == 0) {
            hangRight = RightWheel;
            hangLeft = LeftWheel;
            hang = 1;
        }
        else {
            RightWheel = hangRight;
            LeftWheel = hangLeft;
            //std::cout << "HUNG" << std::endl;
        }
        //RightWheel = m_fWheelVelocity;
        //LeftWheel = m_fWheelVelocity;
    }
    m_pcWheels->SetLinearVelocity(LeftWheel + NoiseLeft, RightWheel + NoiseRight);
    //std::cout << ID << ", " << Left << ", " << Right << std::endl;


    if (ID == 83) {

    }
    timeweight++;

    if (Fault == Diagnosis && Fault !=0) {
        std::cout << "SUCCESSFUL DIAGNOSIS" << std::endl;
        FaultID = 0;
        DrID = 0;
        Stop = 0;
        Stopping = 0;
        BeginDiagnostic = 0;
        ping = 0;
        returnping = 0;
        RABCompare = 0;
        Fault = 0;
        Diagnosis = 0;
        DrRobo.clear();
        Ambulance.clear();
        DrDistance.clear();
    }



    if (timeweight == RobotNumber) {
        timeweight = 0;
        if (DrDistance.size()>0 && DrID == 0) {
            //std::cout << *min_element(DrDistance.begin(), DrDistance.end()) << std::endl;
            for (int i = 0; i < DrDistance.size(); i++ ) {
                if (DrDistance.at(i) == *min_element(DrDistance.begin(), DrDistance.end())) {
                    //std::cout << "CLOSEST ROBO IS " << DrRobo.at(i) << std::endl;
                    DrID = DrRobo.at(i);
                }
            }
        }
        DrDistance.clear();
        DrRobo.clear();

    }
}


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
