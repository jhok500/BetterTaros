/* Include the controller definition */
#include "footbot_diffusion.h"
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <sstream>
//#include "loop_functions/logging_loop_functions/logging_loop_functions.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
//UploadBoyOmegaFault

/****************************************/

int Time = 0;


CFootBotDiffusion::CFootBotDiffusion() :
        m_pcWheels(NULL),
        m_pcProximity(NULL),
        m_cAlpha(10.0f),
        m_fDelta(0.5f),
        m_fWheelVelocity(2.5f),
        MemoryLogNew(NULL),
        DetectBodge(NULL),
        ClassCheck(NULL),
        ClassConfirm(NULL),
        YawHolder(NULL),
        IntCoord(NULL),
        MemoryBounce(NULL),
        TrueIntCoord(NULL),
        m_pcLight(NULL),
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
    m_pcLight  = GetSensor  <CCI_FootBotLightSensor                    >("footbot_light");
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


    //HYPERCUBE STUFF
    int ParamRow;

    GetNodeAttributeOrDefault(t_node, "foldernum", ParamRow, ParamRow);

    std::string row, cell;
    std::ifstream myfile("LHC_Parameters_for_Runs_taros.csv");
    int lineNumber = 0;
    int lineNumberSought = ParamRow;
    if (myfile.is_open()) {
        while (getline(myfile, row)) {
            lineNumber++;
            if (lineNumber == lineNumberSought) {
                std::istringstream myRow(row);
                int i = 0;
                while (getline(myRow, cell, ',')) {
                    switch (i) {
                        case 0:
                            //NumFault = stof(cell);
                            //BearingNoise = stof(cell);
                            //SimilarityThreshold = stof(cell);
                            FaultType=stof(cell);
                            break;
                        case 1:
                            //YawCoordinateNoise = stof(cell);
                            //DetectDelay = stof(cell);
                            break;
                        case 2:
                            //CoordinateNoise = stof(cell);
                            //DetectRatio = stof(cell);
                            break;
                        case 3:
                            //RangeNoiseMultiplier = stof(cell);
                            //doublecheck = stof(cell);
                            break;
                        case 4:
                            //CoordinateNoise = stof(cell);
                            //DetectRatio = stof(cell);
                            break;
                        case 5:
                            //RangeNoiseMultiplier = stof(cell);
                            //doublecheck = stof(cell);
                            break;

                    }
                    i++;
                }
            }
        }

        myfile.close();
    }
    std::cout << "Fault: " << FaultType << std::endl;
    //std::cout << "Parameters: " << SimilarityThreshold << ", " << DetectDelay << ", " << DetectRatio << ", " << doublecheck << std::endl;
    IntCoord = new boost::circular_buffer<double>(2*2);
    TrueIntCoord = new boost::circular_buffer<double>(2*2);
    YawHolder = new boost::circular_buffer<double>(2);
    FaultyFeatureVectors = new boost::circular_buffer<int>(((6*2))*DetectDelay);
    MemoryLogNew = new boost::circular_buffer<int>(((DetectDelay*6*2)+2)*MemoryBits);
    DetectBodge = new boost::circular_buffer<int>(DetectDelay);
    ClassCheck = new boost::circular_buffer<int>(doublecheck);
    ClassConfirm = new boost::circular_buffer<int>(doublecheck);
    MemoryBounce = new boost::circular_buffer<int>(1000);
    Dis2Beacons = new boost::circular_buffer<double> (10);

    if (ImportMemory) {
        std::string folderAccess = std::to_string(foldernum-1);
        std::string seedAccess = std::to_string(CSimulator::GetInstance().GetRandomSeed());
        std::string rowm, cellm;
        std::ifstream memoryfile(folderAccess+"/"+seedAccess+"/MemoryLog.csv");

        if (memoryfile.is_open()) {

            while (getline(memoryfile, rowm)) {
                std::istringstream myRowm(rowm);
                int i = 0;
                while (getline(myRowm, cellm, ',')) {

                    if (stof(cellm) == 8) {
                        break;
                    }
                    else {
                        MemoryLogNew->push_back(stof(cellm));
                        std::cout << stof(cellm) << std::endl;
                    }
                    i++;
                }
            }
        }
    }

}
/****************************************/
Real CFootBotDiffusion::HeadingCorrect() {
    if (Heading > 180) {
        Heading = -180 + (Heading-180);
    }
    if (Heading < -180) {
        Heading = 180 - (abs(Heading)-180);
    }
    return Heading;
}
/****************************************/
void CFootBotDiffusion::BehaviourUpdate() {
    srand(Time);
    BehaviourCount++;
    //Behaviour = rand() % 3+1;
    //Behaviour = 4;
    //std::cout << "Behaviour is " << Behaviour << std::endl;
}
/****************************************/
/****************************************/
void CFootBotDiffusion::Decision() {
    if(AgentNew.at(8) < 0.5) {
        DecH2++;
        std::cout << "power fail detected" << std::endl;
    }
    else {
        if(AgentNew.at(2)>0.5){
            if(AgentNew.at(10)>0.5){
                if(AgentNew.at(7)>0.5){
                    DecH5++;
                }
                else{
                    if(AgentNew.at(3)>0.5){
                        if(AgentNew.at(9)>0.5) {
                            DecH1++;
                        }
                        else {
                            DecH5++;
                        }
                    }
                    else{
                        DecH4++;
                    }
                }
            }
            else{
                if(AgentNew.at(7)>0.5){
                    if(AgentNew.at(3)>0.5) {
                        if(AgentNew.at(4)>0.5){
                            if(AgentNew.at(5)>0.5){
                                DecH6++;
                            }
                            else{
                                DecH5++;
                            }
                        }
                        else{
                            DecH1++;
                        }
                    }
                    else{
                        if(AgentNew.at(5)>0.5){
                            DecH6++;
                        }
                        else{
                            DecH5++;
                        }
                    }
                }
                else{
                    if(AgentNew.at(1)>0.5){
                        if(AgentNew.at(9)>0.5){
                            if(AgentNew.at(4)>0.5){
                                DecH5++;
                            }
                            else{
                                DecH1++;
                            }
                        }
                        else{
                            DecH4++;
                        }
                    }
                    else{
                        DecH1++;
                    }
                }
            }
        }
        else{
            if(AgentNew.at(7)>0.5) {
                if(AgentNew.at(1)>0.5) {
                    DecH3++;
                }
                else {
                    if(AgentNew.at(5)>0.5){
                        DecH6++;
                    }
                    else {
                        if(AgentNew.at(6)>0.5) {
                            DecH4++;
                        }
                        else{
                            DecH6++;
                        }
                    }
                }
            }
            else {
                if(AgentNew.at(6)>0.5) {
                    DecH3++;
                }
                else {
                    DecH6++;
                }
            }
        }
    }


}

void CFootBotDiffusion::FaultInject() {
    TrueTotal++;
    srand(Time);
    //Faulty = rand() % 6 + 1;
    Faulty = 2;
    if (Faulty == 4 || Faulty == 5) {
        if (Time % 2 == 0) {
            MotorRand = 1;
        }
        else {
            MotorRand = 2;
        }
    }
    FaultyIDs.push_back(ID);
    std::cout << "Faulty Robot is " << ID << ": " << Faulty << std::endl;

}

/****************************************/

void CFootBotDiffusion::FaultInjectOmega() {
    Faulty = FaultType;
    if (Faulty == 4 || Faulty == 5) {
        if (Time % 2 == 0) {
            MotorRand = 1;
        }
        else {
            MotorRand = 2;
        }
    }
    FaultyIDs.push_back(ID);
}
/****************************************/
void CFootBotDiffusion::ObstacleAv() {
    const CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
    CVector2 cAccumulator;
    for (size_t i = 0; i < tProxReads.size(); ++i) {
        cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }
    cAccumulator /= tProxReads.size();
    const CCI_RangeAndBearingSensor::TReadings &packets = range_and_bearing_sensor->GetReadings();
    cAngle = cAccumulator.Angle();
    if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
        cAccumulator.Length() < m_fDelta) {
        Left = 1;
        Right = 1;
    }
    else {
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
/****************************************/
void CFootBotDiffusion::Aggregation() {
    const CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
    CVector2 cAccumulator;
    for (size_t i = 0; i < tProxReads.size(); ++i) {
        cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }
    cAccumulator /= tProxReads.size();
    const CCI_RangeAndBearingSensor::TReadings &packets = range_and_bearing_sensor->GetReadings();
    cAngle = cAccumulator.Angle();
    CVector2 GoalCoord;
    Real GoalBearing;
    GoalCoord.Set((std::accumulate(OmegaCoordX.begin(), OmegaCoordX.end(), 0.0) / OmegaCoordX.size()),
                  (std::accumulate(OmegaCoordY.begin(), OmegaCoordY.end(), 0.0) / OmegaCoordY.size()));
    GoalBearing = atan2(GoalCoord.GetY(), GoalCoord.GetX()) * 180 / ARGOS_PI;
    if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
        cAccumulator.Length() < m_fDelta) {
        if (Alone == 0) {
            Left = 1;
            Right = 1;
        }
        else if (Heading/fabs(Heading) == GoalBearing/fabs(GoalBearing)) {
            if (Heading < GoalBearing) {
                Left = 0;
                Right = 1;
            }
            else {
                Left = 1;
                Right = 0;
            }
        }
        else {
            if (Heading < 0) {
                if (fabs(Heading) + fabs(GoalBearing) < 180) {
                    Left = 0;
                    Right = 1;
                } else {
                    Left = 1;
                    Right = 0;
                }
            }
            else {
                if (fabs(Heading) + fabs(GoalBearing) > 180) {
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
    else {
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
/****************************************/
void CFootBotDiffusion::Flocking() {
    const CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
    CVector2 cAccumulator;
    for (size_t i = 0; i < tProxReads.size(); ++i) {
        cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }
    cAccumulator /= tProxReads.size();
    const CCI_RangeAndBearingSensor::TReadings &packets = range_and_bearing_sensor->GetReadings();
    cAngle = cAccumulator.Angle();
    Real Goal;
    Real GoalHeading = std::accumulate(FlockHeadings.begin(), FlockHeadings.end(), 0.0) / FlockHeadings.size();
    Real BearingX = std::accumulate(OmegaCoordX.begin(), OmegaCoordX.end(), 0.0) / OmegaCoordX.size();
    Real BearingY = std::accumulate(OmegaCoordY.begin(), OmegaCoordY.end(), 0.0) / OmegaCoordY.size();
    Real GoalBearing = atan2(BearingY,BearingX)*180/ARGOS_PI;
    if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
        cAccumulator.Length() < m_fDelta) {
        if (std::accumulate(FlockRange.begin(), FlockRange.end(), 0.0) / FlockRange.size() >
            (30 + (15 * FlockRange.size()))) {
            Goal = GoalBearing;
        }
        else {
            Goal = GoalHeading;
        }
        if (Heading/fabs(Heading) == Goal/fabs(Goal)) {
            if (Heading < Goal) {
                Left = 0;
                Right = 1;
            }
            else {
                Left = 1;
                Right = 0;
            }
        }
        else {
            if (Heading < 0) {
                if (fabs(Heading) + fabs(Goal) < 180) {
                    Left = 0;
                    Right = 1;
                } else {
                    Left = 1;
                    Right = 0;
                }
            }
            else {
                if (fabs(Heading) + fabs(Goal) > 180) {
                    Left = 0;
                    Right = 1;
                }
                else {
                    Left = 1;
                    Right = 0;
                }
            }
        }
        if (Alone == 0) {
            Left = 1;
            Right = 1;
        }
    }
    else {
        if (cAngle.GetValue() > 0.0f) {
            Left = 1;
            Right = 0;
        } else {
            Left = 0;
            Right = 1;
        }
    }
}
/****************************************/


/****************************************/

CVector2 CFootBotDiffusion::VectorToLight() {
    /* Get light readings */
    const CCI_FootBotLightSensor::TReadings& tReadings = m_pcLight->GetReadings();
    /* Calculate a normalized vector that points to the closest light */
    CVector2 cAccum;
    for(size_t i = 0; i < tReadings.size(); ++i) {
        cAccum += CVector2(tReadings[i].Value, tReadings[i].Angle);
    }
    if(cAccum.Length() > 0.0f) {
        /* Make the vector long as 1/4 of the max speed */
        cAccum.Normalize();
        cAccum *= 0.25f * m_fWheelVelocity;
    }
    return cAccum;

}


/****************************************/




void CFootBotDiffusion::ControlStep() {
    if (timeweight == 0) {
        Time = Time + 1;
        countyboy = 0;
    }
    int RobotNumber = CSimulator::GetInstance().GetSpace().GetEntitiesByType("foot-bot").size();
    /*if (CSimulator::GetInstance().GetSpace().GetEntitiesByType("box").size() == 4 && !SaveMemory) {
        //std::cout << "SAVEMEMORY" << std::endl;
        SaveMemory = true;}*/
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

    // CODE STARTS HERE



    CFootBotEntity &entity = dynamic_cast<CFootBotEntity &>(CSimulator::GetInstance().GetSpace().GetEntity(GetId()));
    CRadians yaw, pitch, roll;
    /****************************************/
    std::string robot_name = GetId();
    robot_name = robot_name.substr(2, robot_name.size());
    int id = atoi(robot_name.c_str());
    ID = id+100;
    entity.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(yaw, pitch, roll);
    // Add noise
    unsigned seed = Time;
    std::default_random_engine generator(seed);
    std::normal_distribution<double> YawNoise(0,BearingNoise);
    std::normal_distribution<double> YawCoordNoise(0,YawCoordinateNoise);
    std::normal_distribution<double> XNoise(0,CoordinateNoise);
    std::normal_distribution<double> YNoise(0,CoordinateNoise);

    Heading = (yaw.GetValue() * 180 / ARGOS_PI) + YawNoise(generator);
    Heading = HeadingCorrect();

    X = entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX() + XNoise(generator);
    Y = entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY() + YNoise(generator);
    TrueX = entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
    TrueY = entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();

    NoiseLeft = 0.0f;
    NoiseRight = 0.0f;
    Left = 0;
    Right = 0;

    FlockRange.clear();
    AbsoluteFlockBearing.clear();
    FlockBearing.clear();
    FlockHeadings.clear();
    FlockCoordX.clear();
    FlockCoordY.clear();
    OmegaCoordX.clear();
    OmegaCoordY.clear();
    if (Diagnosed && !Doctor && !Dead) {
        Diagnosed = false;
        Diagnosis = 0;
    }
    // FAULT INJECTION
    int prob = rand() % FaultProb;

    if (Time == 500) {

        if (ID == NumFault) {
            FaultInjectOmega();
        }
    }


    // BEHAVIOUR SWITCH
    if (Time > (BehaviourCount*5000)) {
        BehaviourUpdate();
    }


    std::ostringstream oss;
    CLoopFunctions CS;
    CSpace::TMapPerType& foot_bots = CS.GetSpace().GetEntitiesByType("foot-bot");

    Quarantine = false;
    // Reset detection array





    Ambulance.clear();
    Alone = 0;
    int MidProxCoord = 0;
    int CloseProxCoord = 0;
    int MidProx = 0;
    int CloseProx = 0;
    int InvestigateBounce = 0;
    int DoctorBounce = 0;

    for (CCI_RangeAndBearingSensor::SPacket packet : packets) {
        double bearing = ToDegrees(packet.HorizontalBearing).GetValue();
        std::normal_distribution<double> RABNoise(0,packet.Range*RangeNoiseMultiplier);
        double range = packet.Range + RABNoise(generator);
        for (auto &map_element : foot_bots){
            CFootBotEntity &foot_bot = *any_cast<CFootBotEntity *>(map_element.second);
            CFootBotDiffusion &controller = dynamic_cast<CFootBotDiffusion &>(foot_bot.GetControllableEntity().GetController());
            // Alignment
            if (controller.ID == packet.Data[0]+100 && !Dead && !controller.Dead) {
                // Neighbours are in range
                MidProxCoord = MidProxCoord + 1;
                // Neighbours are in close range
                if (sqrt(pow(X - controller.X, 2) + pow(Y-controller.Y, 2)) < 0.3) {
                    CloseProxCoord = CloseProxCoord + 1;
                }
                // Normal Sensor Function
                if (abs(bearing) > PMFangle && Faulty == 6|| Faulty == 3 || power || hang) {
                    if (TestLap && controller.DoctorsOrder == ID && Faulty == 6) {
                        partialsensorboy = false;
                    }
                }
                else {
                    Alone++;
                    FlockBearing.push_back(bearing);
                    AbsoluteFlockBearing.push_back(atan2((controller.TrueY - TrueY), (controller.TrueX - TrueX)) * 180 / ARGOS_PI);
                    OmegaCoordX.push_back(cos(atan2((controller.TrueY - TrueY), (controller.TrueX - TrueX))));
                    OmegaCoordY.push_back(sin(atan2((controller.TrueY - TrueY), (controller.TrueX - TrueX))));
                    FlockRange.push_back(range);
                    MidProx = MidProx + 1;
                    if (packet.Range < 30) {
                        CloseProx = CloseProx + 1;
                    }
                    if (TestLap && controller.DoctorsOrder == ID) {
                        partialsensorboy = true;
                    }
                }
                // Gather Behaviour Data
                FlockCoordX.push_back(controller.X);
                FlockCoordY.push_back(controller.Y);
                FlockHeadings.push_back(controller.Heading);
                // DETECTION
            }

        }



    }

    // BEHAVIOURS

    // OBSTACLE AVOIDANCE
    if (!Quarantine && !Doctor && Behaviour == 1 && !Dead) {
        ObstacleAv();
    }
    // AGGREGATION
    if (!Quarantine && !Doctor && Behaviour == 2 && !Dead) {
        Aggregation();
    }
    // FLOCKING
    if (!Quarantine && !Doctor && Behaviour == 3 && !Dead) {
        Flocking();
    }




    // SET NORMAL WHEEL VALUES
    RightWheel = (m_fWheelVelocity*Right);
    LeftWheel = (m_fWheelVelocity*Left);
    NoiseLeft = 0;
    NoiseRight = 0;
    // COMPLETE SENSOR FAULT //
    if (Faulty == 3 && !FaultResolved && !Dead) {
        RightWheel = m_fWheelVelocity;
        LeftWheel = m_fWheelVelocity;
    }
    // POWER FAILURE //
    if (Faulty == 2 && !FaultResolved && !Dead) {
        power = true;
        RightWheel = 0;
        LeftWheel = 0;
    }
    // SOFTWARE HANG //
    if (Faulty == 1 && !FaultResolved && !Dead) {
        if (!hang) {
            hangRight = RightWheel;
            hangLeft = LeftWheel;
            HangVector.push_back(AgentNew[2]);
            HangVector.push_back(AgentNew[4]);
            HangVector.push_back(AgentNew[6]);
            HangVector.push_back(AgentNew[8]);
            HangVector.push_back(AgentNew[10]);
            hang = true;
        }
        else {
            RightWheel = hangRight;
            LeftWheel = hangLeft;
        }
    }
    // COMPUTE FEATURE VECTOR
    // Initialise

    AgentNew.clear();
    AgentNew.push_back(-ID);
    // Internally Calculate Wheel Velocity
    Real Velocity_Wheels = 0.5*(GetSpoofLeftWheelVelocity() + GetSpoofRightWheelVelocity());
    Real Difference_Wheels = GetSpoofLeftWheelVelocity() - GetSpoofRightWheelVelocity();


    // Set Internal & External Neighbour Features
    // F1 MidProx Ext
    if (MidProxCoord > 0) {
        AgentNew.push_back(1);
    }
    else {
        AgentNew.push_back(0);
    }
    // F2 MidProx Int
    if (!hang) {
        if (MidProx > 0) {
            AgentNew.push_back(1);

        }
        else {
            AgentNew.push_back(0);
        }
    }
    else if (hang) {
        AgentNew.push_back(HangVector[0]);
    }
    // F3 CloseProx Ext
    if (CloseProxCoord > 0) {
        AgentNew.push_back(1);
    }
    else {
        AgentNew.push_back(0);
    }
    // F4 CloseProx Int
    if (!hang) {
        if (CloseProx > 0) {
            AgentNew.push_back(1);
        }
        else {
            AgentNew.push_back(0);
        }
    }
    else if (hang && !FaultResolved) {
        AgentNew.push_back(HangVector[1]);
    }
    // Calculate Linear Velocity Externally
    if (IntCoord->size() == IntCoord->capacity()) {
        Real CoordDistance = sqrt(pow(((IntCoord->at(0)) - (IntCoord->at(2))), 2) + pow(((IntCoord->at(1)) - (IntCoord->at(3))), 2));
        Real CoordSpeed = CoordDistance * 10;
        // Set Linear Motion Features Internally & Externally
        // F5 Velocity Ext
        if (CoordDistance > 0.0045) {
            AgentNew.push_back(1);
        }
        else {
            AgentNew.push_back(0);
        }
        // F6 Velocity Int
        if (!hang || hang && FaultResolved) {
            if ((Velocity_Wheels / 10) / 100 > 0.0045) {
                AgentNew.push_back(1);
            }
            else {
                AgentNew.push_back(0);
            }
        }
        else if (hang && !FaultResolved) {
            AgentNew.push_back(HangVector[2]);
        }
        // F7 Speed Ext
        if (CoordSpeed > 0.01) {
            AgentNew.push_back(1);
        }
        else {
            AgentNew.push_back(0);
        }
        // F8 Speed Int
        if (!hang || hang && FaultResolved) {
            if (Velocity_Wheels / 100 > 0.01) {
                AgentNew.push_back(1);
            }
            else {
                AgentNew.push_back(0);
            }
        }
        else if (hang && !FaultResolved) {
            AgentNew.push_back(HangVector[3]);
        }
        // Calculate Angular Velocity Externally
        YawHolder->push_back(atan2(((TrueIntCoord->at(0)) - (TrueIntCoord->at(2))),
                                   ((TrueIntCoord->at(1)) - (TrueIntCoord->at(3)))));


        if (YawHolder->size() == YawHolder->capacity()) {
            Real IntYawCoord = (YawHolder->front() - YawHolder->back() + (YawCoordNoise(generator)*ARGOS_PI/180));
            // Set External Angular Velocity Feature
            // F9 AngVel Ext
            //std::cout << ID << ": " << IntYawCoord << std::endl;
            if (fabs(IntYawCoord*180/ARGOS_PI) > 0.8 && fabs(IntYawCoord*180/ARGOS_PI) < 5) {
                AgentNew.push_back(1);
            }
            else {
                AgentNew.push_back(0);
            }

        }
        // Set Internal Angular Velocity Feature
        // F10 AngVel Int
        if (!hang || hang && FaultResolved) {
            if (Difference_Wheels == 0) {
                AgentNew.push_back(0);
            }
            else {
                AgentNew.push_back(1);
            }
        }
        else if (hang && !FaultResolved) {
            AgentNew.push_back(HangVector[4]);
        }
    }

    IntCoord->push_back(X);
    IntCoord->push_back(Y);
    TrueIntCoord->push_back(TrueX);
    TrueIntCoord->push_back(TrueY);





    // MOTOR FAULTS //
    if (Faulty == 4 && !FaultResolved && !Dead) {
        if (MotorRand == 1) {
            NoiseLeft = -LeftWheel;
        }
        else if (MotorRand == 2){
            NoiseRight = -RightWheel;
        }
    }
    if (Faulty == 5 && !FaultResolved && !Dead) {
        if (MotorRand == 1) {
            NoiseLeft = -0.5*LeftWheel;
        }
        else if (MotorRand == 2){
            NoiseRight = -0.5*RightWheel;
        }
    }

    // SET CONTROLLER VALUES
    if (!Dead) {

        m_pcWheels->SetLinearVelocity(LeftWheel + NoiseLeft, RightWheel + NoiseRight);
    }
    else {
        m_pcWheels->SetLinearVelocity(0, 0);
    }

    // FAULTY RESET
    if (Faulty != 0) {
        //Decision();
        for(int i =1; i <AgentNew.size(); i++) {
            FeatureVector.push_back(AgentNew.at(i));
            //std::cout<<AgentNew.at(i)<<std::endl;
        }
    }







    if (Time == ExperimentLength && !StuckBounce && Faulty != 0) {
        //std::cout << "END" << std::endl;
        StuckBounce = true;

        std::string folderName = std::to_string(foldernum-1);
        std::string seedFolder = std::to_string(CSimulator::GetInstance().GetRandomSeed());
        std::string slashyboi = "/";
        std::string path = folderName+slashyboi+seedFolder;
        mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        DataFile.open (folderName+"/"+seedFolder+"/Data.csv", std::ios_base::app);
        for (int j = 0; j < FeatureVector.size(); j++) {
            DataFile << FeatureVector.at(j) << ",";
        }

        DataFile.close();


        CSimulator::GetInstance().Terminate();
    }







    timeweight++;
    if (timeweight == RobotNumber) {
        FaultsInPlay = faultCount;
        timeweight = 0;
        faultCount = 0;
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