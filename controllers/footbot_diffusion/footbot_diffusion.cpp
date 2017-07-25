/* Include the controller definition */
#include "footbot_diffusion.h"
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "loop_functions/logging_loop_functions/logging_loop_functions.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>


/****************************************/
int Roulette = 0;
int Time = 0;
int RobotNumber = 10;
int EyesOn = 0;
int hang = 0;
int power = 0;
int Stop = 0;
int Stopping = 0;
int ping = 0;
int returnping = 0;
int RABCompare = 0;
int RABReturn = 0;
int TestLM = 0;
int ConfirmLM = 0;
int TestRM = 0;
int ConfirmRM = 0;
int TestStraight = 0;
int ConfirmStraight = 0;
int TestLap = 0;
int ConfirmLap = 0;
int Diagnosed = 0;
int DiagReset = 0;
int SnapshotTaken = 0;
int wall = 0;
int DiagCandidate = 0;

boost::circular_buffer<int>* PingWait;
boost::circular_buffer<int>* StopWait;
boost::circular_buffer<int>* RABWait;
int RABConfirm;
boost::circular_buffer<int>* MotorWait;
int RCMFConfirm;
int LCMFConfirm;
boost::circular_buffer<int>* StraightWait;
int PMFConfirm;
boost::circular_buffer<int>* LapWait;
int LapCount = 0;
int LapDelay = 0;
Real LapStart;
std::vector<Real> Snapshot;
std::vector<Real> TestCase;






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
    GetNodeAttributeOrDefault(t_node, "foldernum", foldernum, 1);
    /****************************************/
    FlockData = new boost::circular_buffer<double>(2*RobotNumber);
    FlockCoordData = new boost::circular_buffer<double>(3*RobotNumber);
    PingWait = new boost::circular_buffer<int>(10);
    StopWait = new boost::circular_buffer<int>(10);
    RABWait = new boost::circular_buffer<int>(10);
    MotorWait = new boost::circular_buffer<int>(10);
    StraightWait = new boost::circular_buffer<int>(10);
    LapWait = new boost::circular_buffer<int>(10);
    IntCoord = new boost::circular_buffer<double>(3*RobotNumber*2);
    TrueIntCoord = new boost::circular_buffer<double>(3*RobotNumber*2);
    YawHolder = new boost::circular_buffer<double>(2*RobotNumber*2);
    FeatureVectors3 = new boost::circular_buffer<int>((1+(6*2))*DetectDelay);
    MemoryLog = new boost::circular_buffer<int>(((DetectDelay*6*2)+2)*MemoryBits);

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
    std::string IDraw = GetId();
    int ID = IDraw.at(2) + 32;
    entity.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(yaw, pitch, roll);
    unsigned seed = Time;
    std::default_random_engine generator(seed);
    std::normal_distribution<double> YawNoise(0,3);
    std::normal_distribution<double> YawCoordNoise(0,0.1);
    std::normal_distribution<double> XNoise(0,0.000025);
    std::normal_distribution<double> YNoise(0,0.000025);
    std::normal_distribution<double> RABNoise(0,3);

    Real Heading = (yaw.GetValue() * 180 / ARGOS_PI) + YawNoise(generator);
    if (Heading > 180) {
        Heading = -180 + (Heading-180);
    }
    if (Heading < -180) {
        Heading = 180 - (abs(Heading)-180);
    }

    Real X = entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX() + XNoise(generator);
    Real Y = entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY() + YNoise(generator);
    Real TrueX = entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
    Real TrueY = entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();




    int Move = 0;
    NoiseLeft = 0.0f;
    NoiseRight = 0.0f;


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
    int Alone;
    int AloneCount = 0;
    int Left = 0;
    int Right = 0;

    // FAULT INJECTION
    if (Time > (BounceCount + 500) && FaultBounce == 0) {
        srand(Time);
        FaultBounce = 1;
        while (Fault == 0) {
            Fault = rand() % 6 + 1;
            if (Fault == 5 && Behaviour > 1) {
                Fault = 0;
            }
        }
        std::cout << "Fault is " << Fault << std::endl;
        if (std::find(std::begin(Checklist), std::end(Checklist),Fault) != std::end(Checklist)) {
            Eligibility = 1;
            //std::cout << "Fault is known" << std::endl;
        }
        else {
            Eligibility = 0;
            Checklist.push_back(Fault);
            //std::cout << "Fault is unknown" << std::endl;
        }
        //Fault = 2;

    }
    // BEHAVIOUR SWITCH
    if (Time > (BehaviourCount*5000)) {
        srand(Time);
        BehaviourCount++;
        Behaviour = rand() % 3+1;
        std::cout << "Behaviour is " << Behaviour << std::endl;
    }
    if (Time <= 36000) {
        //std::string folderName = std::to_string(foldernum);
        //mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        //DataFile.open (folderName + "/Data.csv", std::ios_base::app);

    }


    // BEHAVIOURS //
    int Quarantine = 0;
    if (FlockData->size() == FlockData->capacity() && FlockCoordData->size() == FlockCoordData->capacity()) {
        if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
            cAccumulator.Length() < m_fDelta) {
            Ambulance.clear();
            AloneCount = 0;
            Alone = 1;
            for (CCI_RangeAndBearingSensor::SPacket packet : packets) {
                double bearing = ToDegrees(packet.HorizontalBearing).GetValue();
                double range = packet.Range + RABNoise(generator);
                if (Update.size() > 0 && Detected == 0) {
                    for (int x = 0; x < Update.size(); x++) {
                        if (Update.at(x) == ID) {
                            int comp = Update.at(x+1);
                            if (std::find(std::begin(Update), std::end(Update),ID) != std::end(Update) && std::find(std::begin(Update), std::end(Update),packet.Data[0] + 80) == std::end(Update)) {
                                Update.push_back(packet.Data[0] + 80);
                                Update.push_back(comp);
                                std::cout << packet.Data[0]+80 << " UPDATED TO " << comp << std::endl;
                            }
                            else {
                                for (int i = 0; i < Update.size(); i++) {
                                    if (Update.at(i) == packet.Data[0] + 80 && Update.at(i + 1) < comp) {
                                        Update.at(i + 1) = UpdateNum;
                                        std::cout << packet.Data[0]+80 << " UPDATED TO " << comp << std::endl;
                                    }
                                }
                            }
                        }
                    }





                }
                // Partial Sensor Failure //
                if (ID == 83 && Fault == 6) {
                    if (Detected == 1) {
                        FaultID = ID;
                    }
                    if (abs(bearing) > PMFangle && AloneCount == 0) {
                        //std::cout << "I can't see " << packet.Data[0] + 80 << std::endl;
                    }
                    else {
                        //std::cout << "I can see " << packet.Data[0] + 80 << std::endl;
                        Alone = 0;
                        FlockBearing.push_back(bearing);
                        FlockRange.push_back(range);
                        FlockID.push_back(packet.Data[0] + 80);
                    }
                }
                else {
                    Alone = 0;
                    FlockBearing.push_back(bearing);
                    FlockRange.push_back(range);
                    FlockID.push_back(packet.Data[0] + 80);
                }
                if (packet.Data[0] + 80 == FaultID) {
                    DrRobo.push_back(ID);
                    DrDistance.push_back(range);
                }
                if (packet.Data[0] + 80 == DrID && ID != FaultID) {
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
                    EyesOn = 1;
                    Ambulance.push_back(bearing);
                    Ambulance.push_back(range);
                }
            }
            // OBSTACLE AVOIDANCE
            if (Quarantine == 0 && ID != DrID && Behaviour == 1) {
                for (int i = 0; i < FlockRange.size(); i++) {
                    if (FlockRange[i] < 30) {
                        if (FlockBearing[i] > 0.0f) {
                            Left = 1;
                            Right = 0;
                        }
                        else {
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
                    Left = 1;
                    Right = 1;
                }
            }
            // AGGREGATION
            if (Quarantine == 0 && ID != DrID && Behaviour == 2) {
                for (int i = 0; i < FlockRange.size(); i++) {
                    if (FlockRange[i] < 30) {
                        Danger = 1;
                        if (FlockBearing[i] > 0.0f) {
                            Left = 1;
                            Right = 0;
                        }
                        else {
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
                        Left = 1;
                        Right = 1;
                    }
                    else {
                        if (Heading < GoalBearing) {
                            Left = 0;
                            Right = 1;
                        }
                        else {
                            Left = 1;
                            Right = 0;
                        }
                    }
                }
                if (Alone == 1) {
                    Left = 1;
                    Right = 1;
                }
            }
            // FLOCKING
            if (Quarantine == 0 && ID != DrID && Behaviour == 3) {
                for (int i = 0; i < FlockRange.size(); i++) {
                    if (FlockRange[i] < 30) {
                        Danger = 1;
                        if (FlockBearing[i] > 0.0f) {
                            Left = 1;
                            Right = 0;
                        }
                        else {
                            Left = 0;
                            Right = 1;
                        }
                    }
                }
                if (Danger == 0) {

                    for (int j = 0; j < FlockData->size(); j++) {
                        for (int k = 0; k < FlockID.size(); k++) {
                            if (FlockData->at(j) == FlockID.at(k)) {
                                FlockHeadings.push_back(FlockData->at(j + 1));
                            }
                        }
                    }
                    Real Goal;
                    Real GoalHeading =
                            std::accumulate(FlockHeadings.begin(), FlockHeadings.end(), 0.0) / FlockHeadings.size();
                    Real GoalBearing =
                            std::accumulate(FlockBearing.begin(), FlockBearing.end(), 0.0) / FlockBearing.size();
                    if (std::accumulate(FlockRange.begin(), FlockRange.end(), 0.0) / FlockRange.size() >
                        (30 + (15 * FlockRange.size()))) {
                        Goal = GoalBearing;
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
                        if (abs(Heading) < abs(Goal) + 15 && abs(Heading) > abs(Goal) - 15) {
                            Left = 1;
                            Right = 1;
                        }
                        else {
                            if (Heading < Goal) {
                                Left = 0;
                                Right = 1;
                            }
                            else {
                                Left = 1;
                                Right = 0;
                            }
                        }
                    }
                }
                if (Alone == 1) {
                    Left = 1;
                    Right = 1;
                }
            }

            // DECIDE ON A DR//
            if (ID == DrID && Ambulance.size() > 1) {
                BeginDiagnostic = 1;
                if (Ambulance.at(1) < 50) {
                    Left = 0;
                    Right = 0;

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
            if (ID == FaultID) {
                wall = 0;

            }
        }
        else {
            if (ID == FaultID && power == 0 && hang ==0) {
                wall = 1;
            }


            if (cAngle.GetValue() > 0.0f) {
                Left = 1;
                Right = 0;
            }
            else {
                Left = 0;
                Right = 1;
            }

        }
    }

    // SET NORMAL WHEEL VALUES
    RightWheel = (m_fWheelVelocity*Right);
    LeftWheel = (m_fWheelVelocity*Left);
    NoiseLeft = 0;
    NoiseRight = 0;


    // COMPLETE SENSOR FAULT //
    if (ID == 83 && Fault == 3) {
        if (Detected == 1) {
            FaultID = ID;
        }
        RightWheel = m_fWheelVelocity;
        LeftWheel = m_fWheelVelocity;
    }
    // POWER FAILURE //
    if (ID == 83 && Fault == 2) {
        if (Detected == 1) {
            FaultID = ID;
        }
        power = 1;
        RightWheel = 0;
        LeftWheel = 0;
    }


    // SOFTWARE HANG //
    if (ID == 83 && Fault == 1) {
        if (Detected == 1) {
            FaultID = ID;
        }
        if (hang == 0) {
            hangRight = RightWheel;
            hangLeft = LeftWheel;
            F1hang = Agent.at(2);
            F2hang = Agent.at(3);
            F3hang = Agent.at(4);
            F4hang = Agent.at(5);
            F5hang = Agent.at(6);
            hang = 1;
        }
        else {
            RightWheel = hangRight;
            LeftWheel = hangLeft;
        }
    }


    // COMPUTE FEATURE VECTOR
    // Initialise
    int MidProxCoord = 0;
    int CloseProxCoord = 0;
    int MidProx = 0;
    int CloseProx = 0;
    Agent.clear();
    AgentCoord.clear();
    Indices.clear();
    Indices1.clear();
    Indices4.clear();
    // Timestamp & ID
    Agent.push_back(Time);
    AgentCoord.push_back(Time);
    Agent.push_back(-ID);
    AgentCoord.push_back(-ID);
    // Internally Calculate Wheel Velocity
    Real Velocity_Wheels = 0.5*(GetSpoofLeftWheelVelocity() + GetSpoofRightWheelVelocity());
    Real Difference_Wheels = GetSpoofLeftWheelVelocity() - GetSpoofRightWheelVelocity();
    // Calculate Neighbours in Mid-Close Proximity Internally & Externally
    for(CCI_RangeAndBearingSensor::SPacket packet : packets)
    {

        double bearing = ToDegrees(packet.HorizontalBearing).GetValue();
        double range = packet.Range + RABNoise(generator);
        MidProxCoord = MidProxCoord + 1;
        Real TrueBearing;
        if (bearing + Heading < -180) {
            TrueBearing = (360 - (sqrt(pow(bearing + Heading, 2))));
        }
        else if (bearing + Heading > 180) {
            TrueBearing = -(360 - (sqrt(pow(bearing + Heading, 2))));
        }
        else {
            TrueBearing = bearing + Heading;
        }
        Real TrueRange = (range / 100);
        NeighbourDistance.Set(TrueRange * cos(ARGOS_PI * TrueBearing / 180),
                              TrueRange * sin(ARGOS_PI * TrueBearing / 180));
        if (sqrt(pow(NeighbourDistance.GetX(), 2) + pow(NeighbourDistance.GetY(), 2)) < 0.3) {
            CloseProxCoord = CloseProxCoord + 1;
        }
        if (ID == 83 && abs(bearing) > PMFangle && Fault == 6 || ID == 83 && Fault == 3 || power == 1 || hang == 1) {

        }
        else {


            MidProx = MidProx + 1;




            if (packet.Range < 30) {
                CloseProx = CloseProx + 1;
            }


        }
    }
    // Set Internal & External Neighbour Features
    if (MidProxCoord > 0) {
        AgentCoord.push_back(1);
    }
    else {
        AgentCoord.push_back(0);
    }
    if (hang == 0) {
        if (MidProx > 0) {
            Agent.push_back(1);
        }
        else {
            Agent.push_back(0);
        }
    }
    else {
        Agent.push_back(F1hang);
    }

    if (CloseProxCoord > 0) {
        AgentCoord.push_back(1);
    }
    else {
        AgentCoord.push_back(0);
    }
    if (hang == 0) {
        if (CloseProx > 0) {
            Agent.push_back(1);
        }
        else {
            Agent.push_back(0);
        }
    }
    else {
        Agent.push_back(F2hang);
    }
    // Calculate Linear Velocity Externally
    if (IntCoord->size() == IntCoord->capacity()) {

        for (int i = 0; i < IntCoord->size(); i++) {
            if (IntCoord->at(i) == ID) {
                Indices.push_back(i);

            }
        }
        Real CoordDistance = sqrt(pow(((IntCoord->at(Indices.front() + 1)) - (IntCoord->at(Indices.back() + 1))), 2)
                                  + pow(((IntCoord->at(Indices.front() + 2)) - (IntCoord->at(Indices.back() + 2))), 2));
        Real CoordSpeed = CoordDistance * 10;
        // Set Linear Motion Features Internally & Externally
        if (CoordDistance > 0.0045) {
            AgentCoord.push_back(1);
        }
        else {
            AgentCoord.push_back(0);
        }
        if (hang == 0) {
            if ((Velocity_Wheels / 10) / 100 > 0.0045) {
                Agent.push_back(1);
            }
            else {
                Agent.push_back(0);
            }
        }
        else {
            Agent.push_back(F3hang);
        }

        if (CoordSpeed > 0.01) {
            AgentCoord.push_back(1);
        }
        else {
            AgentCoord.push_back(0);
        }
        if (hang == 0) {
            if (Velocity_Wheels / 100 > 0.01) {
                Agent.push_back(1);
            }
            else {
                Agent.push_back(0);
            }
        }
        else {
            Agent.push_back(F4hang);
        }
        // Calculate Angular Velocity Externally
        YawHolder->push_back(ID);
        YawHolder->push_back(atan2(((TrueIntCoord->at(Indices.front() + 1)) - (TrueIntCoord->at(Indices.front() + 1 + (3*RobotNumber)))),
                                   ((TrueIntCoord->at(Indices.front() + 2)) -
                                    (TrueIntCoord->at(Indices.front() + 2 + (3*RobotNumber))))));
        if (YawHolder->size() == YawHolder->capacity()) {
            for (int i = 0; i < YawHolder->size(); i++) {
                if (YawHolder->at(i) == ID) {
                    Indices1.push_back(i);
                    //std::cout << "CORRECT " << RealID << " YAW AT " << i << std::endl;
                }
            }
            Real IntYawCoord = (YawHolder->at(Indices1.front() + 1) - YawHolder->at(Indices1.back() + 1)) + (YawCoordNoise(generator)*ARGOS_PI/180);
            // Set External Angular Velocity Feature
            if (fabs(IntYawCoord*180/ARGOS_PI) > 0.8 && fabs(IntYawCoord*180/ARGOS_PI) < 5) {
                AgentCoord.push_back(1);
            }
            else {
                AgentCoord.push_back(0);
            }
        }
        // Set Internal Angular Velocity Feature
        if (hang == 0) {
            if (Difference_Wheels == 0) {
                Agent.push_back(0);

            }
            else {
                Agent.push_back(1);
            }
        }
        else {
            Agent.push_back(F5hang);
        }

    }
    // WATCHDOG
    if (hang == 0) {
        /*if (ID == 83) {
            std::cout << "Fine" << std::endl;
        }*/
        Agent.push_back(0);
        AgentCoord.push_back(0);
    }
    else {
        /*if (ID == 83) {
            std::cout << "hung" << std::endl;
        }*/
        Agent.push_back(1);
        AgentCoord.push_back(1);
    }
    // Record Data for Future Comparison
    IntCoord->push_back(ID);
    IntCoord->push_back(X);
    IntCoord->push_back(Y);
    TrueIntCoord->push_back(ID);
    TrueIntCoord->push_back(TrueX);
    TrueIntCoord->push_back(TrueY);
    // Print Features To Log
    for (int i = 0; i < Agent.size(); i++ ) {
        if (Fault != 0 && Detected == 0 && ID == 83) {
            if (Agent.at(i) != AgentCoord.at(i)) {
                Detectmin++;
                //std::cout << "Discrepency" << std::endl;
            }

        }
        if (Agent.at(i) == -83 && Time > 10) {
            /*std::cout << "Agent 83 Control" << std::endl;
            std::cout << Agent[i] << std::endl;
            std::cout << AgentCoord[i] << std::endl;
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
            std::cout << Agent[i+6] << std::endl;
            std::cout << AgentCoord[i+6] << std::endl;*/
            FeatureVectors3->push_back(-Time);
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
            FeatureVectors3->push_back(Agent[i+6]);
            FeatureVectors3->push_back(AgentCoord[i+6]);
            if (AgentCoord.at(i+1) == 1) {
                InRange = 1;
            }
            else {
                InRange = 0;
            }
        }
    }

    // DETECTION BODGE
    if (Fault != 0 && ID == 83 && Detected == 0) {
        if (Detectmin > 0 && InRange == 1) {
            Detect++;
            Detectmin = 0;
            //std::cout << "Increment: " << Detect << std::endl;
        }
        else {
            Detect = 0;
            //std::cout << "No Discrepency, Reset" << std::endl;
        }
    }
    if (Detect == DetectDelay) {
        std::cout << "DETECTED" << std::endl;
        Detected = 1;
        Detect = 0;
        TimeStart = Time;
        DataFile << "Time: " << Time << ", " << "Behaviour: " << Behaviour << ", " << "Fault: " << Fault << ", ";
    }



    if (ID == FaultID && SnapshotTaken == 0) {
        SnapshotTaken = 1;
        //SnapshotFile.open ("SnapShot.csv", std::ios_base::app);
        for (int i = 0; i < FeatureVectors3->size(); i++ ) {
            if (FeatureVectors3->at(i) < 0) {
                Snapshot.push_back(FeatureVectors3->at(i+1));
                Snapshot.push_back(FeatureVectors3->at(i+2));
                Snapshot.push_back(FeatureVectors3->at(i+3));
                Snapshot.push_back(FeatureVectors3->at(i+4));
                Snapshot.push_back(FeatureVectors3->at(i+5));
                Snapshot.push_back(FeatureVectors3->at(i+6));
                Snapshot.push_back(FeatureVectors3->at(i+7));
                Snapshot.push_back(FeatureVectors3->at(i+8));
                Snapshot.push_back(FeatureVectors3->at(i+9));
                Snapshot.push_back(FeatureVectors3->at(i+10));
                Snapshot.push_back(FeatureVectors3->at(i+12));
                Snapshot.push_back(FeatureVectors3->at(i+11));
                //SnapshotFile << FeatureVectors3->at(i) << ", ";
                /*SnapshotFile << FeatureVectors3->at(i+1) << ", ";
                SnapshotFile << FeatureVectors3->at(i+2) << ", ";
                SnapshotFile << FeatureVectors3->at(i+3) << ", ";
                SnapshotFile << FeatureVectors3->at(i+4) << ", ";
                SnapshotFile << FeatureVectors3->at(i+5) << ", ";
                SnapshotFile << FeatureVectors3->at(i+6) << ", ";
                SnapshotFile << FeatureVectors3->at(i+7) << ", ";
                SnapshotFile << FeatureVectors3->at(i+8) << ", ";
                SnapshotFile << FeatureVectors3->at(i+9) << ", ";
                SnapshotFile << FeatureVectors3->at(i+10) << ", ";
                 SnapshotFile << FeatureVectors3->at(i+11) << ", ";
                 SnapshotFile << FeatureVectors3->at(i+12) << ", ";
                SnapshotFile << std::endl;*/
            }
            //std::cout << FeatureVectors3->at(i) << std::endl;
        }
        //SnapshotFile.close();
        //std::cout << "SNAPSHOT TAKEN, BEHAVIOR: " << Behaviour << ", FAULT: " << Fault << std::endl;
    }

    // USE CLASSIFIER
    if (BeginDiagnostic > 0 && ClassBounce == 0) {
        ClassBounce = 1;
        Candidates.clear();
        if (Detected == 1 && MemoryLog->size() >= MemoryLog->capacity() / MemoryBits && SnapshotTaken == 1 && ID == DrID) {
            std::cout << "Run Classifier" << std::endl;
            Real sumtop;
            Real topadd;
            Real bottomadd1;
            Real bottomadd2;
            Real sumbottom;
            Real sumbottom1;
            Real sumbottom2;

            Real MeanMem;
            Real MeanSnap;
            int j = 0;
            for (int i = 0; i < MemoryLog->size(); i++) {
                Real MemSum = 0;

                if (MemoryLog->at(i) < 0) {
                    for (int y = 0; y < Update.size(); y++) {
                        if (Update.at(y) == ID) {
                            scratch = Update.at(y+1);
                        }
                    }
                    if (scratch < MemoryLog->at(i+1)) {
                        noaccess = 1;
                        std::cout << "FAULT " << MemoryLog->at(i) << " AT " << MemoryLog->at(i+1) << " CAN'T BE ACCESSED BY DR FROM " << scratch << std::endl;
                    }
                    else {
                        noaccess = 0;
                        std::cout << "FAULT " << MemoryLog->at(i) << " AT " << MemoryLog->at(i+1) << " IS FINE WITH DR FROM " << scratch << std::endl;
                    }
                    //TestCase.push_back(MemoryLog->at(i));
                    DiagCandidate = MemoryLog->at(i);
                    //std::cout << "DiagCandidate: " << DiagCandidate << std::endl;
                    for (int k = i+2; k < i + Snapshot.size()+2; k++) {
                        MemSum = MemSum + MemoryLog->at(k);

                        //std::cout << "K " << MemoryLog->at(k) << std::endl;
                    }
                    std::cout << "MemSum = " << MemSum << " Snapshot Size = " << Snapshot.size() << std::endl;
                    MeanMem = MemSum / Snapshot.size();
                    MeanSnap = std::accumulate(Snapshot.begin(), Snapshot.end(), 0.0) / Snapshot.size();
                    sumtop = 0;
                    topadd = 0;
                    bottomadd1 = 0;
                    bottomadd2 = 0;
                    sumbottom = 0;
                    sumbottom1 = 0;
                    sumbottom2 = 0;
                }
                else if (MemoryLog->at(i) < 2 && noaccess == 0) {
                    //TestCase.push_back(MemoryLog->at(i));
                    topadd = (Snapshot.at(j) - MeanSnap) * (MemoryLog->at(i) - MeanMem);
                    sumtop = sumtop + topadd;
                    bottomadd1 = pow((Snapshot.at(j) - MeanSnap), 2);
                    bottomadd2 = pow((MemoryLog->at(i) - MeanMem), 2);
                    sumbottom1 = sumbottom1 + bottomadd1;
                    sumbottom2 = sumbottom2 + bottomadd2;
                    //std::cout << "SumTop: " << sumtop << std::endl;
                    //std::cout << "SumBottom: " << sumbottom1 << ", " << sumbottom2 << std::endl;
                    j++;
                    if (j == Snapshot.size()) {
                        j = 0;
                        sumbottom = sumbottom1 * sumbottom2;
                        Real R = sumtop / sqrt(sumbottom);
                        std::cout << "matches prev. fault " << -DiagCandidate << " by " << R << std::endl;
                        //DataFile << "Previous: " << DiagCandidate << ", " << "Similarity: " << R << ", ";
                        if (R > 0.66) {
                            //std::cout << "MATCH" << std::endl;
                            Candidates.push_back(R);
                            Candidates.push_back(DiagCandidate);

                        }
                        else {
                            //std::cout << "Not This One" << std::endl;

                        }
                    }
                }

            }
            if (Candidates.size() == 0) {
                std::cout << "Run Diagnostics" << std::endl;
                BeginMOT = 1;

            }
            else {
                for (int i = 0; i < Candidates.size(); i++) {
                    if (Candidates.at(i)== *max_element(Candidates.begin(), Candidates.end())) {
                        ClassifierSuccess = 1;
                        Diagnosis = -Candidates.at(i+1);
                        std::cout << "Classified at" << Diagnosis << " with " << Candidates.at(i)*100 << "% similarity" << std::endl;
                        DataFile << Candidates.at(i)*100 << ", ";
                    }
                }
            }
        }
        else {
            //std::cout << "Run Diagnostics" << std::endl;
            BeginMOT = 1;
        }
    }

    // DIAGNOSTIC TESTS
    if (BeginMOT > 0 && ID == DrID) {
        //std::cout << "Diagnostic Start" << std::endl;
        if (returnping == 0) {
            ping = 1;
            PingWait->push_back(1);
        }
        else if (returnping == 1) {
            PingWait->push_back(0);
            //std::cout << "Ping Successful" << std::endl;
            // STOP FAULTY ID
            Stop = 1;
            if (Stopping == 1) {
                StopWait->push_back(0);
                //COMPARE RAB
                //std::cout << "Stopped" << std::endl;
                RABCompare = 1;
                if (RABReturn == 1) {
                    RABCompare = 0;
                    //std::cout << "No CSF" << std::endl;
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
                            //std::cout << "NO CMF" << std::endl;
                        }
                    }
                    if (std::accumulate(MotorWait->begin(), MotorWait->end(), 0.0) == MotorWait->capacity()) {
                        //std::cout << "Complete Motor Failure" << std::endl;
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
                                //std::cout << "PSF" << std::endl;
                                Diagnosis = 6;
                                ConfirmLap = 0;
                            }
                        }
                        if (std::accumulate(StraightWait->begin(), StraightWait->end(), 0.0) == StraightWait->capacity()) {
                            //std::cout << "PMF" << std::endl;
                            Diagnosis = 5;
                        }
                        // LAP & COMPARE
                    }
                }
                else if (std::accumulate(RABWait->begin(), RABWait->end(), 0.0) == RABWait->capacity()) {
                    //std::cout << "CSF" << std::endl;
                    Diagnosis = 3;
                }

            }
            else if (Stop == 1 && Stopping == 0 && wall == 0 || Stop == 1 && Stopping == 0 && Fault == 1) {
                StopWait->push_back(1);
                if (std::accumulate(StopWait->begin(), StopWait->end(), 0.0) == StopWait->capacity()) {
                    //std::cout << "Software Hang" << std::endl;
                    Diagnosis = 1;
                }
            }
        }
        if (std::accumulate(PingWait->begin(), PingWait->end(), 0.0) == PingWait->capacity()) {
            //std::cout << "Power Failure" << std::endl;
            Diagnosis = 2;
        }

    }
    if (BeginMOT > 0 && ID == FaultID && power == 0) {
        returnping = 1;
        ping = 0;
        if (Stop == 1 && Fault != 1) {
            if (wall == 1 && Fault > 4) {

            }
            else {
                LeftWheel = 0;
                RightWheel = 0;
                Stopping = 1;
                if (RABCompare == 1) {
                    for (int i = 0; i < Agent.size(); i++) {
                        if (Agent.at(i) == -FaultID) {
                            int CSFFeature = Agent.at(i + 1);
                            if (CSFFeature == 0) {
                                RABWait->push_back(1);
                            }
                            else {
                                RABWait->push_back(0);
                                RABConfirm++;
                                if (RABConfirm == 50) {
                                    RABReturn = 1;
                                    RABConfirm = 0;
                                }

                            }
                        }
                    }
                }
                else if (TestRM == 1) {
                    Right = 1;
                    RightWheel = (m_fWheelVelocity * Right);;
                    for (int i = 0; i < Agent.size(); i++) {
                        if (Agent.at(i) == -FaultID) {
                            int RCMFFeature = AgentCoord.at(i + 4);
                            if (RCMFFeature == 0) {
                                MotorWait->push_back(1);
                            }
                            else {
                                MotorWait->push_back(0);
                                RCMFConfirm++;
                                if (RCMFConfirm == 50) {
                                    ConfirmRM = 1;
                                    RCMFConfirm = 0;
                                }

                            }
                        }
                    }

                }
                else if (TestLM == 1) {
                    Left = 1;
                    LeftWheel = (m_fWheelVelocity * Left);
                    for (int i = 0; i < Agent.size(); i++) {
                        if (Agent.at(i) == -FaultID) {
                            int LCMFFeature = AgentCoord.at(i + 4);
                            if (LCMFFeature == 0) {
                                MotorWait->push_back(1);
                            }
                            else {
                                MotorWait->push_back(0);
                                LCMFConfirm++;
                                if (LCMFConfirm == 50) {
                                    ConfirmLM = 1;
                                    LCMFConfirm = 0;
                                }
                            }
                        }
                    }
                }
                else if (TestStraight == 1) {
                    Right = 1;
                    RightWheel = (m_fWheelVelocity * Right);
                    Left = 1;
                    LeftWheel = (m_fWheelVelocity * Left);
                    for (int i = 0; i < Agent.size(); i++) {
                        if (Agent.at(i) == -FaultID) {
                            int PMFFeature = AgentCoord.at(i + 5);
                            if (PMFFeature == 1) {
                                StraightWait->push_back(1);
                            }
                            else {
                                PMFConfirm++;
                                if (PMFConfirm == 50) {
                                    StraightWait->push_back(0);
                                    ConfirmStraight = 1;
                                    PMFConfirm = 0;
                                }
                            }
                        }
                    }
                }
                else if (TestLap == 1) {
                    if (LapCount == 0) {
                        LapStart = Heading;
                        LapCount = 1;
                    }
                    LapDelay++;
                    if (LapDelay > 50) {
                        if (Heading < LapStart + 10 && Heading > LapStart - 10) {
                            ConfirmStraight = 1;
                        }
                        else {
                            Left = 1;
                            LeftWheel = (m_fWheelVelocity * Left);
                        }
                    }
                    else {
                        Left = 1;
                        LeftWheel = (m_fWheelVelocity * Left);
                    }
                    for (int i = 0; i < Agent.size(); i++) {
                        if (Agent.at(i) == -FaultID) {
                            int PSFFeature = Agent.at(i + 1);
                            if (PSFFeature == 0) {
                                LapWait->push_back(1);
                            }
                            else {
                                LapWait->push_back(0);
                            }
                        }
                    }
                }
            }
        }
    }

    // MOTOR FAULTS //
    if (ID == 83 && Fault == 4) {
        NoiseLeft = -LeftWheel;
        //NoiseRight = -RightWheel;
        if (Detected == 1) {
            FaultID = ID;
        }
    }
    if (ID == 83 && Fault == 5) {
        NoiseLeft = -0.5*LeftWheel;
        if (Detected == 1) {
            FaultID = ID;
        }
    }
    // if (Time > BounceCount + 500 + DetectDelay)
    // SET CONTROLLER VALUES

    m_pcWheels->SetLinearVelocity(LeftWheel + NoiseLeft, RightWheel + NoiseRight);

    timeweight++;
    if (Diagnosis !=0) {

        if (std::find(std::begin(PowerCycle), std::end(PowerCycle),Diagnosis) != std::end(PowerCycle)
            && std::find(std::begin(PowerCycle), std::end(PowerCycle),Fault) != std::end(PowerCycle)) {
            std::cout << "Fault = " << Fault << ", Diagnosis = " << Diagnosis << ", Recovery: Cycle Power" << std::endl;
            DiagReset = 1;
            Diagnosed = 1;
        }
        else if (std::find(std::begin(MotorReplacement), std::end(MotorReplacement),Diagnosis) != std::end(MotorReplacement)
                 && std::find(std::begin(MotorReplacement), std::end(MotorReplacement),Fault) != std::end(MotorReplacement)) {
            std::cout << "Fault = " << Fault << ", Diagnosis = " << Diagnosis << ", Recovery: Replace Motor" << std::endl;
            DiagReset = 1;
            Diagnosed = 1;
        }
        else if (std::find(std::begin(SensorReplacement), std::end(SensorReplacement),Diagnosis) != std::end(SensorReplacement)
                 && std::find(std::begin(SensorReplacement), std::end(SensorReplacement),Fault) != std::end(SensorReplacement)) {
            std::cout << "Fault = " << Fault << ", Diagnosis = " << Diagnosis << ", Recovery: Replace Sensor" << std::endl;
            DiagReset = 1;
            Diagnosed = 1;
        }
        else {
            std::cout << "Fault = " << Fault << ", Diagnosis = " << Diagnosis << ", FAILURE" << std::endl;
            DataFile << "FAILURE " << Diagnosis << ", ";
            ClassifierSuccess = 0;
            Fail++;
            std::cout << "Total Fails: " << Fail << std::endl;
            if (BeginMOT == 0) {
                Diagnosis = 0;
                BeginMOT = 1;
            }
            //Diagnosed = 1;
        }

    }
    if (DiagReset == 1) {
        if (Diagnosed == 1) {
            TrueTotal++;
            MemoryLog->push_back(-Diagnosis);
            MemoryLog->push_back(Time);
            UpdateNum = Time;
            if (std::find(std::begin(Update), std::end(Update),ID) != std::end(Update) && std::find(std::begin(Update), std::end(Update),DrID) == std::end(Update)) {
                Update.push_back(DrID);
                Update.push_back(UpdateNum);
                std::cout << DrID << " UPDATED TO " << UpdateNum << std::endl;
            }
            else {
                for (int i = 0; i < Update.size(); i++) {
                    if (Update.at(i) == DrID && Update.at(i + 1) < UpdateNum) {
                        Update.at(i + 1) = UpdateNum;
                        std::cout << DrID << " UPDATED TO " << UpdateNum << std::endl;
                    }
                }
            }
            Update.push_back(DrID);
            Update.push_back(UpdateNum);
            for (int i = 0; i < Snapshot.size(); i++) {
                MemoryLog->push_back(Snapshot.at(i));
            }
            if (Eligibility == 1 || ClassifierSuccess == 1) {
                Total++;
                std::cout << "Total: " << Total << std::endl;
            }
            if (ClassifierSuccess == 1) {
                std::cout << "DIAGNOSED (CLASSIFIER)" << std::endl;
                DataFile << Diagnosis << " , " << "CLASSIFIED ,";

                Class++;
                std::cout << "Total Class: " << Class << std::endl;

            }
            else {
                std::cout << "DIAGNOSED (MOT)" << std::endl;
                DataFile << "n/a" << ", " << Diagnosis << " , " << "DIAGNOSED ,";
                int TimeTaken = Time - TimeStart;
                DataFile << "Time Taken" << ", " << TimeTaken << ", ";
                if (Eligibility == 1) {
                    MOT++;
                    std::cout << "Total MOT: " << MOT << std::endl;
                    DataFile << "ELLIGIBLE";
                }

                //std::cout << "TimeTaken: " << TimeTaken << std::endl;
            }
            TimeStart = 0;
            DataFile << std::endl;
            BounceCount = Time;
            FaultID = 0;
            DrID = 0;
            Fault = 0;
            Diagnosis = 0;
            Diagnosed = 0;
            DrRobo.clear();
            Ambulance.clear();
            DrDistance.clear();
            Snapshot.clear();
            power = 0;
            hang = 0;
            FaultBounce = 0;
            SnapshotTaken = 0;
            Detected = 0;
            DiagCandidate = 0;
            F1hang = 0;
            F2hang = 0;
            F3hang = 0;
            F4hang = 0;
            F5hang = 0;
        }
        else {
            std::cout << "RESET" << std::endl;
        }
        ClassifierSuccess = 0;
        ClassBounce = 0;
        BeginMOT = 0;
        BeginDiagnostic = 0;
        ping = 0;
        returnping = 0;
        Stop = 0;
        Stopping = 0;
        RABCompare = 0;
        RABConfirm = 0;
        RABReturn = 0;
        ConfirmRM = 0;
        TestRM = 0;
        RCMFConfirm = 0;
        LCMFConfirm = 0;
        ConfirmLM = 0;
        TestLM = 0;
        TestStraight = 0;
        ConfirmStraight = 0;
        PMFConfirm = 0;
        TestLap = 0;
        ConfirmLap = 0;
        LapDelay = 0;
        LapStart = 0;
        LapCount = 0;
        Detect = 0;


        PingWait->clear();
        StopWait->clear();
        RABWait->clear();
        MotorWait->clear();
        StraightWait->clear();
        LapWait->clear();
        DiagReset = 0;

    }
    if (Time == 36000 && StuckBounce == 0) {
        StuckBounce =1;
        Real ClassPer = Class/Total;
        Real ClassFailPer = Fail/(Class+Fail);
        /*std::string folderName = std::to_string(foldernum);
        mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        SpartanPercent.open (folderName + "/ClassFailPercent.csv", std::ios_base::app);
        SpartanPercent << CSimulator::GetInstance().GetRandomSeed() << "," << ClassFailPer;
        SpartanPercent.close();*/
        DataFile << "Total Faults, " << TrueTotal << ", Elligible Total, " << Total << ", DiagTot, " << TrueTotal -(Class+Fail) <<
        ", Elligible DiagTot, " << Total - (Class+Fail) << ", FailTot, " << Fail << ", MemoryTot, " << Class << ", %Memory," << ClassPer << ", %Failure," << ClassFailPer << std::endl;
        std::cout << "Total " << Total << ", Class " << Class << ", Fail " << Fail << std::endl;
        std::cout << "MEMORY % " << ClassPer << ", FAIL % " << ClassFailPer << std::endl;
    }


    if (timeweight == RobotNumber) {
        timeweight = 0;
        if (DrDistance.size()>0 && DrID == 0) {
            //std::cout << *min_element(DrDistance.begin(), DrDistance.end()) << std::endl;
            for (int i = 0; i < DrDistance.size(); i++ ) {
                if (DrDistance.at(i) == *min_element(DrDistance.begin(), DrDistance.end())) {
                    DrID = DrRobo.at(i);
                    std::cout << "DR: " << DrID << std::endl;

                }
            }
        }
        DrDistance.clear();
        DrRobo.clear();

    }

    //DataFile.close();





    if (ID == DrID) {
        if (EyesOn == 0) {
            DrID = 0;
            std::cout << "DR LOST" << std::endl;
            DiagReset = 1;
        }
        EyesOn = 0;
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