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
    std::ifstream myfile("LHC_Parameters_for_Runs.csv");
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

                    }
                    i++;
                }
            }
        }

        myfile.close();
    }

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
    Behaviour = rand() % 3+1;
    //Behaviour = 4;
    std::cout << "Behaviour is " << Behaviour << std::endl;
}
/****************************************/

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
void CFootBotDiffusion::OmegaAlg() {
    const CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
    CVector2 cAccumulator;
    for (size_t i = 0; i < tProxReads.size(); ++i) {
        cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }
    cAccumulator /= tProxReads.size();
    const CCI_RangeAndBearingSensor::TReadings &packets = range_and_bearing_sensor->GetReadings();
    cAngle = cAccumulator.Angle();
    OmegaTurnX.clear();
    OmegaTurnY.clear();
    bool OmegaTurning = false;
    Real OmegaX = std::accumulate(OmegaCoordX.begin(), OmegaCoordX.end(), 0.0) / OmegaCoordX.size();
    Real OmegaY = std::accumulate(OmegaCoordY.begin(), OmegaCoordY.end(), 0.0) / OmegaCoordY.size();
    Real OmegaBearing = atan2(OmegaY,OmegaX)*180/ARGOS_PI;
    OmegaTimer++;
    for (int i = 0; i < FlockRange.size(); i++) {
        if (VectorToLight().GetX() != 0 && FlockRange.at(i) <= OmegaGap) {
            OmegaTurnX.push_back(OmegaCoordX.at(i));
            OmegaTurnY.push_back(OmegaCoordY.at(i));
        }
    }
    if (!OmegaTurnX.empty() && !OmegaTurnY.empty()) {
        OmegaTurning = true;
    }
    if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
        cAccumulator.Length() < m_fDelta && !OmegaTurning) {
        if (OmegaTimer > Omega) {
            if (Heading < OmegaBearing + 5 && Heading > OmegaBearing - 5) {
                OmegaTimer = 0;
            }
            else {
                if (Heading/fabs(Heading) == OmegaBearing/fabs(OmegaBearing)) {
                    if (Heading < OmegaBearing) {
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
                        if (fabs(Heading) + fabs(OmegaBearing) < 180) {
                            Left = 0;
                            Right = 1;
                        } else {
                            Left = 1;
                            Right = 0;
                        }
                    }
                    else {
                        if (fabs(Heading) + fabs(OmegaBearing) > 180) {
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
            if (Alone == 0) {
                Left = 1;
                Right = 1;
            }
        }
        else {
            Left = 1;
            Right = 1;
        }
    }
    else if (OmegaTurning && m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
             cAccumulator.Length() < m_fDelta) {
        Real TurnX = std::accumulate(OmegaTurnX.begin(), OmegaTurnX.end(), 0.0) / OmegaTurnX.size();
        Real TurnY = std::accumulate(OmegaTurnY.begin(), OmegaTurnY.end(), 0.0) / OmegaTurnY.size();
        Real TurnAngle = atan2(TurnY, TurnX) * 180/ARGOS_PI;
        if (Heading / fabs(Heading) == TurnAngle / fabs(TurnAngle)) {
            if (Heading > TurnAngle) {
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
                if (fabs(Heading) + fabs(TurnAngle) < 180) {
                    Left = 1;
                    Right = 0;
                }
                else {
                    Left = 0;
                    Right = 1;
                }
            }
            else {
                if (fabs(Heading) + fabs(TurnAngle) > 180) {
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
    else {
        OmegaTimer = 0;
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
void CFootBotDiffusion::DrPursue() {
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
        if (Ambulance.at(1) < 50) {
            Left = 0;
            Right = 0;
            if (!ClassifierSuccess) {
                BeginMOT = true;
                //std::cout << " Begin MOT " << std::endl;
            }
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
void CFootBotDiffusion::TakeSnapshot() {
    for (int i = 0; i < FaultyFeatureVectors->size(); i++) {
        Snapshot.push_back(FaultyFeatureVectors->at(i));
    }
}
/****************************************/
void CFootBotDiffusion::Classify() {
    if (MemoryLogNew->size() >= (MemoryLogNew->capacity() / MemoryBits)) {
        MeanMem = 0;
        MeanSnap = 0;
        Jcount = 0;
        for (int i = 0; i < MemoryLogNew->size(); i++) {
            MemSum = 0;
            if (MemoryLogNew->at(i) < 0) {
                DiagCandidate = MemoryLogNew->at(i);
                FaultTime = -MemoryLogNew->at(i+1);
                for (int k = i+2; k < i + Snapshot.size()+2; k++) {
                    MemSum = MemSum + MemoryLogNew->at(k);
                }
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
            else if (MemoryLogNew->at(i) < 2) {
                topadd = (Snapshot.at(Jcount) - MeanSnap) * (MemoryLogNew->at(i) - MeanMem);
                sumtop = sumtop + topadd;
                bottomadd1 = pow((Snapshot.at(Jcount) - MeanSnap), 2);
                bottomadd2 = pow((MemoryLogNew->at(i) - MeanMem), 2);
                sumbottom1 = sumbottom1 + bottomadd1;
                sumbottom2 = sumbottom2 + bottomadd2;
                Jcount++;
                if (Jcount == Snapshot.size()) {
                    Jcount = 0;
                    sumbottom = sumbottom1 * sumbottom2;
                    Rcorr = sumtop / sqrt(sumbottom);
                    if (Rcorr > SimilarityThreshold) {
                        Candidates.push_back(Rcorr);
                        Candidates.push_back(DiagCandidate);
                        Candidates.push_back(FaultTime);
                    }
                }
            }
        }
        if (Candidates.size() == 0) {
            std::cout << "Run Diagnostics" << std::endl;
            ClassifierSuccess = false;
        }
        else {
            for (int i = 0; i < Candidates.size(); i++) {
                if (Candidates.at(i)== *max_element(Candidates.begin(), Candidates.end())) {
                    RValue = Candidates.at(i);
                    Diagnosis = -Candidates.at(i+1);
                    ClassifyCheck = true;
                    TimeID = -Candidates.at(i+2);
                    std::cout << "Classified as " << Diagnosis << " with " << Candidates.at(i)*100 << "% similarity" << std::endl;
                    break;
                }
            }
        }
    }
    else {
        ClassifierSuccess = false;
    }
}
/****************************************/
void CFootBotDiffusion::ActiveMemory() {
    for (int i = 0; i < MemoryLogNew->size(); i++) {
        if (MemoryLogNew->at(i) == TimeID && !ActiBounce) {
            ActiBounce = true;
            MemoryLogNew->erase(MemoryLogNew->begin() + i-1);
            MemoryLogNew->erase(MemoryLogNew->begin() + i-1);
            MemoryLogNew->push_back(-Diagnosis);
            MemoryLogNew->push_back(TimeID);

            for (int j = i; j < i + (DetectDelay * 6 * 2); j++) {
                int carrier = MemoryLogNew->at(i-1);
                MemoryLogNew->erase(MemoryLogNew->begin() + i-1);
                MemoryLogNew->push_back(carrier);
            }
        }
    }
    ActiBounce = false;
}
/****************************************/
void CFootBotDiffusion::DoctorReset() {
    DetectBodge->clear();
    //std::cout << "DETECT CLEARED" << std::endl;
    if (Eligibility) {
        Total++;
    }
    //ACTIVE MEMORY
    if (RValue >= ActiveThreshold) {
        ActiveMemory();
    }
        // UNIQUE FAULT TYPE AND TIME IDENTIFY
    else {
        MemoryLogNew->push_back(-Diagnosis);
        MemoryLogNew->push_back(Time);
        for (int i = 0; i < Snapshot.size(); i++) {
            MemoryLogNew->push_back(Snapshot.at(i));
        }
    }


    if (ClassifierSuccess) {
        MemoryTimes.push_back(Time);
        CorrCoeff.push_back(RValue);
    }
    else {
        MOTTimes.push_back(Time);

    }
    Candidates.clear();
    TimeID = 0;
    TimeStart = 0;
    RValue = 0;
    Diagnosis = 0;
    Diagnosed = false;
    DiagnosisConfirm = 0;
    DiagnosisConfirmed = false;
    Ambulance.clear();
    Snapshot.clear();
    DiagCandidate = 0;
    Doctor = false;
    DoctorsOrder = 0;
    ClassifierSuccess = true;
    BeginMOT = false;
    ping = false;
    Stop = false;
    RABCompare = false;
    RABConfirm = false;
    ConfirmRM = false;
    TestRM = false;
    ConfirmLM = false;
    TestLM = false;
    TestStraight = false;
    ConfirmStraight = false;
    TestLap = false;
    ClassifyCheck = false;
    ClassConfirm->clear();
    ClassCheck->clear();
    ConfirmLap = 0;
    LapDelay = 0;
    LapStart = 0;
    LapCount = false;
    PingWait.clear();
    StopWait.clear();
    RABWait.clear();
    MotorWait.clear();
    StraightWait.clear();
    LapWait.clear();
}
/****************************************/
void CFootBotDiffusion::FaultyReset() {
    HangVector.clear();
    PingWait.clear();
    StopWait.clear();
    RABWait.clear();
    MotorWait.clear();
    StraightWait.clear();
    LapWait.clear();
    BeginMOT = false;
    ping = false;
    Stop = false;
    RABCompare = false;
    RABConfirm = false;
    ConfirmRM = false;
    TestRM = false;
    ConfirmLM = false;
    TestLM = false;
    TestStraight = false;
    ConfirmStraight = false;
    TestLap = false;
    ConfirmLap = 0;
    LapDelay = 0;
    LapStart = 0;
    LapCount = false;
    hangLeft = 0;
    hangRight = 0;
    hang = false;
    power = false;
    MotorRand = 0;
    DiagnosisConfirm = 0;
    DiagnosisConfirmed = false;
    ClassifyCheck = false;
    ClassCheck->clear();
    ClassConfirm->clear();
    for (int i = 0; i < FaultyIDs.size(); i++) {
        if (FaultyIDs.at(i) == ID) {
            FaultyIDs.erase(FaultyIDs.begin() + i);
        }
    }
    for (int i = 0; i < Doctors.size(); i++) {
        if (Doctors.at(i) == ID) {
            Doctors.erase(Doctors.begin() + i);
            Doctors.erase(Doctors.begin() + i - 1);
        }
    }
    if (Diagnosis == 1 || Diagnosis == 2) {
        std::cout << "Fault = " << Faulty << ", Diagnosis = " << Diagnosis << ", Recovery: Cycle Power" << std::endl;
    } else if (Diagnosis == 4 || Diagnosis == 5) {
        std::cout << "Fault = " << Faulty << ", Diagnosis = " << Diagnosis << ", Recovery: Replace Motor" << std::endl;

    } else if (Diagnosis == 3 || Diagnosis == 6) {
        std::cout << "Fault = " << Faulty << ", Diagnosis = " << Diagnosis << ", Recovery: Replace Sensor" << std::endl;
    }

    if (FaultResolved) {
        Faulty = 0;
        FaultResolved = false;
    }
    else {
        std::cout << "Fault not resolved" << std::endl;
        FaultMissed++;
        EscapedFaults.push_back(Faulty);
    }
    Diagnosed = true;

    //FaultStart = 0;
    RValue = 0;



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
void CFootBotDiffusion::CSFTest() {
    if (AgentNew.at(2) == 0) {
        RABWait.clear();
    }
    else {
        RABWait.push_back(1);
        if (std::accumulate(RABWait.begin(), RABWait.end(), 0.0) > ConfirmDelay) {
            RABConfirm = true;
            RABCompare = false;
            RABWait.clear();
        }
    }
}
/****************************************/
void CFootBotDiffusion::CMFTestLeft() {
    Left = 1;
    LeftWheel = (m_fWheelVelocity * Left);
    if (AgentNew.at(7) == 0) {
        MotorWait.clear();
    }
    else {
        MotorWait.push_back(1);
        if (std::accumulate(MotorWait.begin(), MotorWait.end(), 0.0) > ConfirmDelay) {
            ConfirmLM = true;
            TestLM = false;
            MotorWait.clear();
        }
    }
}
/****************************************/
void CFootBotDiffusion::CMFTestRight() {
    Right = 1;
    RightWheel = (m_fWheelVelocity * Right);
    if (AgentNew.at(7) == 0) {
        MotorWait.clear();
    } else {
        MotorWait.push_back(1);
        if (std::accumulate(MotorWait.begin(), MotorWait.end(), 0.0) > ConfirmDelay) {
            ConfirmRM = true;
            TestRM = false;
            MotorWait.clear();
        }
    }
}
/****************************************/
void CFootBotDiffusion::PMFTest() {
    Right = 1;
    RightWheel = (m_fWheelVelocity * Right);
    Left = 1;
    LeftWheel = (m_fWheelVelocity * Left);
    if (AgentNew.at(9) == 1) {
        StraightWait.clear();
    }
    else {
        StraightWait.push_back(1);
        if (std::accumulate(StraightWait.begin(), StraightWait.end(), 0.0) > ConfirmDelay) {
            StraightWait.clear();
            ConfirmStraight = true;
            TestStraight = false;
        }
    }
}
/****************************************/
void CFootBotDiffusion::PSFTest() {
    if (!LapCount) {
        LapStart = Heading;
        LapCount = true;
    }
    LapDelay++;
    if (LapDelay > 50) {
        if (Heading < LapStart + 10 && Heading > LapStart - 10) {
            ConfirmLap = 1;
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
    if (!partialsensorboy) {
        LapWait.push_back(1);
        if (std::accumulate(LapWait.begin(), LapWait.end(), 0.0) > ConfirmDelay) {
            ConfirmLap = -1;
        }
    }
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
    if (prob > FaultProb - 2 && Faulty == 0 && !Dead && !Doctor && FaultsInPlay < 0.5*RobotNumber) {
        FaultStart = Time;
        FaultInject();
    }
    if (Faulty && !Dead) {
        faultCount++;
    }
    /*if (Time == 500) {

        if (ID <= NumFault) {
            FaultInjectOmega();
        }
    }*/
    Real DisToBeacon = sqrt(pow(X-3,2)+pow(Y-3,2));

    Dis2Beacons->push_back(DisToBeacon);

    if (DisToBeacon < 0.8 && Faulty == 0) {
        countyboy++;
    }




    if (countyboy > 0 && !FirstBoy) {
        FirstBoy = true;
        FirstTime = Time;
    }

    if (countyboy >= 0.5*((RobotNumber+99)- NumFault) && !HalfBoy) {
        HalfBoy = true;
        HalfTime = Time;
        std::cout << "HalfBoys" << std::endl;
    }
    if (countyboy == (RobotNumber+99)- NumFault && !AllBoys) {
        AllBoys = true;
        AllTime = Time;
        std::cout << "AllBoys" << std::endl;
    }
    if (Time == 24388 && ID == 109) {
        DistAtFirst = std::accumulate(Dis2Beacons->begin(), Dis2Beacons->end(), 0.0)/Dis2Beacons->size();
        std::cout << DistAtFirst << std::endl;
    }
    if (Time == 25548 && ID == 109) {
        DistAtHalf = std::accumulate(Dis2Beacons->begin(), Dis2Beacons->end(), 0.0)/Dis2Beacons->size();
        std::cout << DistAtHalf << std::endl;
    }
    if (Time == 26909 && ID == 109) {
        DistAtAll = std::accumulate(Dis2Beacons->begin(), Dis2Beacons->end(), 0.0)/Dis2Beacons->size();
        std::cout << DistAtAll << std::endl;
    }

    if (Time > 270000 && ID == 109|| AllBoys && ID == 109) {
        std::string folderName = std::to_string(foldernum-1);
        std::string seedFolder = std::to_string(CSimulator::GetInstance().GetRandomSeed());
        std::string slashyboi = "/";
        std::string path = folderName+slashyboi+seedFolder;
        mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (!FirstBoy) {
            FirstTime = Time;
        }
        if (!HalfBoy) {
            HalfTime = Time;
        }
        if (!AllBoys) {
            AllTime = Time;
        }
        DataFile.open (folderName+"/"+seedFolder+"/Data.csv", std::ios_base::app);
        DataFile << "DistAtFirst," <<  "DistAtHalf," << "AllAvDist," << "FirstTime," << "HalfTime," << "AllTime" << std::endl;
        DataFile <<  DistAtFirst << "," << DistAtHalf << "," << DistAtAll << "," << FirstTime << "," << HalfTime << "," << AllTime << std::endl;
        DataFile.close();
        CSimulator::GetInstance().Terminate();
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
    if (std::find(std::begin(Doctors), std::end(Doctors),UnderInvestigation) != Doctors.end() && UnderInvestigation != DoctorsOrder) {
        UnderInvestigation = 0;
        DetectBodge->clear();
    }

    // Reset doctor robot
    if (Diagnosed && Doctor && !Dead) {
        DoctorReset();
    }

    if (Faulty != 0 && FaultResolved && std::find(std::begin(Doctors), std::end(Doctors),ID) == Doctors.end()) {
        std::cout << "Faulty Robot was resolved anyway" << std::endl;
        Diagnosis = Faulty;
        Diagnosed = true;
        FaultyReset();
    }




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
                if (abs(bearing) > PMFangle && Faulty == 6 && !FaultResolved || Faulty == 3 && !FaultResolved || power && !FaultResolved|| hang && !FaultResolved) {
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
                if (controller.ID == DoctorsOrder && Doctor) {
                    DoctorBounce++;
                    //std::cout << ID << " eyes on " << controller.ID << std::endl;
                }
                if (!Doctor && Faulty == 0 && std::find(std::begin(Doctors), std::end(Doctors), controller.ID) == std::end(Doctors)) {
                    // Begin monitoring robot when found to be faulty
                    if (UnderInvestigation == 0 && controller.Faulty != 0) {
                        UnderInvestigation = controller.ID;
                    }
                    // Stop monitoring if no longer faulty
                    if (UnderInvestigation == controller.ID && controller.Faulty == 0) {
                        UnderInvestigation = 0;
                    }
                    if (controller.ID == UnderInvestigation) {
                        InvestigateBounce++;
                    }

                    // Record monitored robots BFV if unusual
                    if (controller.ID == UnderInvestigation && controller.Faulty != 0) {
                        for (int i = 1; i < controller.AgentNew.size(); i++) {
                            if (i % 2 != 0) {
                                FaultyFeatureVectors->push_back(controller.AgentNew[i]);
                                FaultyFeatureVectors->push_back(controller.AgentNew[i + 1]);
                                if (controller.AgentNew[i] != controller.AgentNew[i + 1]) {
                                    Discrep++;
                                }
                            }
                        }
                        if (Discrep > 0) {
                            Discrep = 0;
                            DetectBodge->push_back(1);
                        } else {
                            DetectBodge->push_back(0);
                        }
                    }
                    // Declare detected fault & assign doctor
                    /*if (std::accumulate(DetectBodge->begin(), DetectBodge->end(), 0.0) >= DetectDelay*DetectRatio && DetectBodge->size() == DetectBodge->capacity()
                            && FaultyFeatureVectors->size() == FaultyFeatureVectors->capacity()) {
                        std::cout << controller.ID << " DETECTED BY " << ID << " AFTER " << Time - controller.FaultStart << std::endl;
                        if (std::find(MemoryLogNew->begin(), MemoryLogNew->end(),-controller.Faulty) != MemoryLogNew->end()) {
                            Eligibility = true;
                        }
                        else {
                            Eligibility = false;
                        }
                        TimeStart = Time;
                        Doctor = true;
                        DoctorsOrder = UnderInvestigation;

                        std::cout << "DR FOR " << DoctorsOrder << " IS " << ID << std::endl;
                        Doctors.push_back(ID);
                        Doctors.push_back(DoctorsOrder);
                        DetectTime.push_back(Time - controller.FaultStart);
                        DetectEDTime.push_back(Time);
                        TakeSnapshot();
                        Classify();
                    }*/
                    //SHARE MEMORY
                    if (std::find(MemoryBounce->begin(), MemoryBounce->end(), controller.ID) == MemoryBounce->end()) {
                        MemoryBounce->push_back(controller.ID);
                        for (int i = 0; i < controller.MemoryLogNew->size(); i++) {
                            if (controller.MemoryLogNew->at(i) < 0 &&
                                (std::find(MemoryLogNew->begin(), MemoryLogNew->end(),
                                           controller.MemoryLogNew->at(i + 1)) == MemoryLogNew->end())) {
                                for (int j = i; j < i + ((12 * DetectDelay) + 2); j++) {
                                    MemoryLogNew->push_back(controller.MemoryLogNew->at(j));
                                }
                            }
                        }
                    }
                    else {
                        MemoryBounce->push_back(0);
                    }
                }

                // DIAGNOSTIC ROUTINE
                // Faulty
                if (Faulty != 0 && ID == controller.DoctorsOrder) {
                    if (std::find(std::begin(PowerCycle), std::end(PowerCycle), controller.Diagnosis) != std::end(PowerCycle)
                                           && std::find(std::begin(PowerCycle), std::end(PowerCycle), Faulty) != std::end(PowerCycle) && !FaultResolved ||
                            std::find(std::begin(MotorReplacement), std::end(MotorReplacement), controller.Diagnosis) != std::end(MotorReplacement)
                         && std::find(std::begin(MotorReplacement), std::end(MotorReplacement), Faulty) != std::end(MotorReplacement) && !FaultResolved ||
                        std::find(std::begin(SensorReplacement), std::end(SensorReplacement), controller.Diagnosis) != std::end(SensorReplacement)
                         && std::find(std::begin(SensorReplacement), std::end(SensorReplacement), Faulty) != std::end(SensorReplacement) && !FaultResolved) {
                        std::cout << "Fault Resolved" << std::endl;
                        FaultResolved = true;
                        ConfirmLap = 0;
                        LapWait.clear();
                    }
                    if (controller.DiagnosisConfirmed && Diagnosis == controller.Diagnosis) {
                        DiagnosisConfirmed = true;
                        //std::cout << "aligning diagnoses" << controller.Diagnosis << ":" << Diagnosis << std::endl;
                    }
                    if (controller.DiagnosisFailed) {
                        std::cout << "Fault unresolvable. Killing robot" << std::endl;
                        Dead = true;
                        DeadTotal++;
                    }
                    if (controller.DiagnosisConfirm == 0 && DiagnosisConfirm != 0) {
                        DiagnosisConfirm = 0;
                    }
                    if (controller.Diagnosis != 0 && Diagnosis != controller.Diagnosis) {
                        Diagnosis = controller.Diagnosis;
                        if (controller.RValue != 0) {
                            RValue = controller.RValue;
                        }
                        if (controller.ClassifyCheck) {
                            ClassifyCheck = true;
                            DiagnosisConfirm = Diagnosis;
                        }
                    }

                    if (controller.BeginMOT && !Diagnosed) {
                        //std::cout << "Faulty Beginning Diagnosis" << std::endl;
                        BeginMOT = true;
                        if (controller.ping && Faulty != 2 || controller.ping && Faulty == 2 && FaultResolved) {
                            ping = true;
                            //std::cout << "Faulty Pinging" << std::endl;
                        }
                        if (controller.Stop && Faulty != 1 || controller.Stop && Faulty == 1 && FaultResolved) {
                            Stop = true;
                            //std::cout << "Faulty Stopping" << std::endl;
                        }
                        if (controller.RABCompare) {
                            RABCompare = true;
                            //std::cout << "Testing Sensor " << std::endl;
                        }
                        if (controller.TestLM && !ConfirmLM) {
                            TestLM = true;
                            //std::cout << "Testing LM" << std::endl;
                        }
                        if (controller.TestRM && !ConfirmRM) {
                            TestRM = true;
                            //std::cout << "Testing RM" << std::endl;
                        }
                        if (controller.TestStraight && !ConfirmStraight) {
                            TestStraight = true;
                            //std::cout << "Testing Straight" << std::endl;
                        }
                        if (controller.TestLap  && ConfirmLap != 1){
                            TestLap = true;
                            //std::cout << "Testing Lap" << std::endl;
                        }

                        if (controller.DiagnosisConfirm != 0 && DiagnosisConfirm == 0) {
                            DiagnosisConfirm = controller.DiagnosisConfirm;
                            //std::cout << "confirming diagnosis: " << DiagnosisConfirm << std::endl;
                        }
                    }
                }
                // Doctor

                if (Doctor && controller.ID == DoctorsOrder && controller.BeginMOT && !controller.Diagnosed && DiagnosisConfirm == 0) {
                    //std::cout << "Doctor Beginning Diagnosis" << std::endl;
                    if (!controller.ping) {
                        ping = true;
                        //std::cout << " pinging faulty" << std::endl;
                    }
                    if (!ping && controller.ping) {
                        ping = true;
                        //std::cout << "faulty was already pung" << std::endl;
                    }
                    if (ping && !controller.ping) {
                        PingWait.push_back(1);
                        if (std::accumulate(PingWait.begin(), PingWait.end(), 0.0) > FaultDelay) {
                            Diagnosis = 2;
                        }
                    }
                    else if (ping && controller.ping) {
                        Stop = true;
                        ping = false;

                    }
                    if (Stop && !controller.Stop) {
                        StopWait.push_back(1);
                        if (std::accumulate(StopWait.begin(), StopWait.end(), 0.0) > FaultDelay) {
                            Diagnosis = 1;
                        }
                    }
                    else if (Stop && controller.Stop && controller.ping) {
                        RABCompare = true;
                        //std::cout << " CSFing faulty" << std::endl;
                        Stop = false;
                    }
                    if (RABCompare && !controller.RABConfirm) {
                        RABWait.push_back(1);
                        if (std::accumulate(RABWait.begin(), RABWait.end(), 0.0) > FaultDelay) {
                            Diagnosis = 3;
                            RABWait.clear();
                        }
                    }
                    else if (RABCompare && controller.RABConfirm) {
                        TestLM = true;
                        RABCompare = false;
                    }
                    if (TestLM && !controller.ConfirmLM) {
                        MotorWait.push_back(1);
                        if (std::accumulate(MotorWait.begin(), MotorWait.end(), 0.0) > FaultDelay) {
                            if (DiagnosisConfirm > 3) {
                                DiagnosisFailed = true;
                            }
                            else {
                                Diagnosis = 4;
                                MotorWait.clear();
                            }
                        }
                    }
                    else if (TestLM && controller.ConfirmLM) {
                        TestRM = true;
                        TestLM = false;
                        MotorWait.clear();
                    }
                    if (TestRM && !controller.ConfirmRM) {
                        MotorWait.push_back(1);
                        if (std::accumulate(MotorWait.begin(), MotorWait.end(), 0.0) > FaultDelay) {
                            if (DiagnosisConfirm > 3) {
                                DiagnosisFailed = true;
                            }
                            else {
                                Diagnosis = 4;
                                MotorWait.clear();
                            }
                        }
                    }
                    else if (TestRM && controller.ConfirmRM) {
                        TestStraight = true;
                        TestRM = false;
                    }
                    if (TestStraight && !controller.ConfirmStraight) {
                        StraightWait.push_back(1);
                        if (std::accumulate(StraightWait.begin(), StraightWait.end(), 0.0) > FaultDelay) {
                            if (DiagnosisConfirm > 3) {
                                DiagnosisFailed = true;
                            }
                            else {
                                Diagnosis = 5;
                                StraightWait.clear();
                            }
                        }
                    }
                    else if (TestStraight && controller.ConfirmStraight) {
                        TestLap = true;
                        TestStraight = false;
                    }

                    if (TestLap && controller.ConfirmLap != 1) {
                        if (controller.ConfirmLap == -1) {
                            LapWait.push_back(1);
                        }
                        if (std::accumulate(LapWait.begin(), LapWait.end(), 0.0) > FaultDelay) {
                            if (DiagnosisConfirm > 3) {
                                DiagnosisFailed = true;
                            }
                            else {
                                Diagnosis = 6;
                                LapWait.clear();
                            }
                        }
                    }
                    else if (TestLap && controller.ConfirmLap == 1) {
                        ConfirmLap = 1;
                    }
                }
                if (Doctor && !DiagnosisConfirmed && controller.ID == DoctorsOrder && ClassifyCheck && ClassifierSuccess) {

                    for (int i = 1; i < controller.AgentNew.size(); i++) {
                        if (i % 2 != 0) {
                            if (controller.AgentNew[i] != controller.AgentNew[i + 1]) {
                                Discrep++;
                            }
                        }
                    }
                    if (Discrep > 0) {
                        Discrep = 0;
                        ClassCheck->push_back(1);
                        ClassConfirm->push_back(0);
                        //std::cout << "nah " << std::accumulate(ClassCheck->begin(), ClassCheck->end(), 0.0) << std::endl;
                    } else {
                        ClassCheck->push_back(0);
                        ClassConfirm->push_back(1);
                        //std::cout << "yeh " << std::accumulate(ClassConfirm->begin(), ClassConfirm->end(), 0.0) << std::endl;
                    }
                    if (std::accumulate(ClassCheck->begin(), ClassCheck->end(), 0.0) == ClassCheck->capacity()) {
                        if (!controller.FaultResolved) {
                            MemoryIncorrect++;
                            //std::cout << "MEMINCORRECT" << std::endl;
                        }
                        else {
                            MemoryCorrectBodge++;
                            //std::cout << "MEMCORRECTBODGE" << std::endl;
                            ClassBodgeFaults.push_back(controller.Faulty);
                        }
                        ClassifierSuccess = false;
                        Diagnosis = 0;
                        DiagnosisConfirm = 0;
                        BeginMOT = true;
                        ClassifyCheck = false;
                        ClassCheck->clear();
                        ClassConfirm->clear();
                    }
                    else if (std::accumulate(ClassConfirm->begin(), ClassConfirm->end(), 0.0) == ClassConfirm->capacity()) {
                        if (controller.FaultResolved) {
                            MemoryCorrect++;
                            //std::cout << "MEMCORRECT" << std::endl;
                        }
                        else {
                            MemoryIncorrectBodge++;
                            //std::cout << "MEMINCORRECTBODGE" << std::endl;
                        }
                        DiagnosisConfirmed = true;
                        Diagnosed = true;
                        ClassifyCheck = false;
                        ClassCheck->clear();
                        ClassConfirm->clear();
                    }

                }

                if (Doctor && !DiagnosisConfirmed && controller.ID == DoctorsOrder && !ClassifyCheck && Diagnosis != 0) {

                    DiagnosisConfirm = Diagnosis;
                    if (Diagnosis > 3) {

                        //std::cout << "checking diagnosis " << DiagnosisConfirm << std::endl;
                        if (DiagnosisConfirm == 4 && !controller.ConfirmLM) {
                            TestLM = true;
                            MotorWait.push_back(1);
                            if (std::accumulate(MotorWait.begin(), MotorWait.end(), 0.0) > FaultDelay) {
                                DiagnosisFailed = true;
                            }
                        }
                        else if (DiagnosisConfirm == 4 && controller.ConfirmLM && !controller.ConfirmRM) {
                            TestRM = true;
                            MotorWait.push_back(1);
                            if (std::accumulate(MotorWait.begin(), MotorWait.end(), 0.0) > FaultDelay) {
                                DiagnosisFailed = true;
                            }
                        }
                        else if (DiagnosisConfirm == 4 && controller.ConfirmLM && controller.ConfirmRM) {
                            DiagnosisConfirmed = true;
                        }
                        if (DiagnosisConfirm == 5 && !controller.ConfirmStraight) {
                            TestStraight = true;
                            StraightWait.push_back(1);
                            if (std::accumulate(StraightWait.begin(), StraightWait.end(), 0.0) > FaultDelay) {
                                DiagnosisFailed = true;
                            }
                        }
                        else if (DiagnosisConfirm == 5 && controller.ConfirmStraight) {
                            DiagnosisConfirmed = true;
                        }
                        if (DiagnosisConfirm == 6 && controller.ConfirmLap != 1) {
                            TestLap = true;
                            if (controller.ConfirmLap == -1) {
                                LapWait.push_back(1);
                            }
                            if (std::accumulate(LapWait.begin(), LapWait.end(), 0.0) > FaultDelay) {
                                DiagnosisFailed = true;
                            }
                        }
                        else if (DiagnosisConfirm == 6 && controller.ConfirmLap == 1) {
                            DiagnosisConfirmed = true;
                            //std::cout << "CONFIRMED PSF" << std::endl;
                        }
                    }
                    else {
                        DiagnosisConfirmed = true;
                    }
                }

                if (Doctor && !Diagnosed && controller.ID == DoctorsOrder && controller.DiagnosisConfirmed && BeginMOT) {
                    //std::cout << "checkity check" << std::endl;
                    if (std::find(std::begin(PowerCycle), std::end(PowerCycle), Diagnosis) != std::end(PowerCycle)
                        && std::find(std::begin(PowerCycle), std::end(PowerCycle), controller.Faulty) != std::end(PowerCycle) && controller.FaultResolved ||
                        std::find(std::begin(MotorReplacement), std::end(MotorReplacement), Diagnosis) != std::end(MotorReplacement)
                        && std::find(std::begin(MotorReplacement), std::end(MotorReplacement), controller.Faulty) != std::end(MotorReplacement) && controller.FaultResolved ||
                        std::find(std::begin(SensorReplacement), std::end(SensorReplacement), Diagnosis) != std::end(SensorReplacement)
                        && std::find(std::begin(SensorReplacement), std::end(SensorReplacement), controller.Faulty) != std::end(SensorReplacement) && controller.FaultResolved) {
                        MOTCorrect++;
                        //std::cout << "MOTCORRECT" << std::endl;
                    }
                    else {
                        MOTIncorrectBodge++;
                        //std::cout << "MOTINCORRECTBODGE" << std::endl;
                    }
                    Diagnosed = true;
                }

                if (Doctor && !Diagnosed && DiagnosisFailed) {
                    if (resetCounter == 0) {
                        resetCounter = Time;
                    }
                }
            }
            if (DiagnosisFailed && Time - resetCounter > 0 || ConfirmLap == 1 && controller.ID == DoctorsOrder && !DiagnosisConfirmed && !controller.DiagnosisConfirmed
                    || ConfirmLap == 1 && ID == controller.DoctorsOrder && !DiagnosisConfirmed && !controller.DiagnosisConfirmed) {
                if (ConfirmLap == 1) {
                    std::cout << ID << ": " << DoctorsOrder << " there was no fault" << std::endl;
                    for (int i = 0; i < Doctors.size(); i++) {
                        //std::cout << Doctors.at(i) << " /" << Doctors.size() << std::endl;
                        if (Doctors.at(i) == ID && Doctor) {
                            Doctors.erase(Doctors.begin() + i);
                            Doctors.erase(Doctors.begin() + i);
                        }
                        else if (Doctors.at(i) == ID && Faulty != 0) {
                            Doctors.erase(Doctors.begin() + i);
                            Doctors.erase(Doctors.begin() + i - 1);
                        }
                    }
                }
                if (DiagnosisFailed) {
                    if (controller.FaultResolved) {
                        MOTCorrectBodge++;
                        //std::cout << "MOTCORRECTBODGE" << std::endl;
                    } else {
                        MOTIncorrect++;
                        //std::cout << "MOTINCORRECT" << std::endl;
                    }
                }
                resetCounter = 0;
                DetectBodge->clear();
                Candidates.clear();
                TimeID = 0;
                TimeStart = 0;
                RValue = 0;
                Diagnosis = 0;
                Diagnosed = false;
                DiagnosisConfirm = 0;
                DiagnosisConfirmed = false;
                if (FaultResolved) {
                    Faulty = 0;
                    FaultResolved = false;
                    MOTNull++;
                    hang = false;
                    power = false;
                }
                DiagnosisFailed = false;
                Ambulance.clear();
                Snapshot.clear();
                DiagCandidate = 0;
                Doctor = false;
                DoctorsOrder = 0;
                UnderInvestigation = 0;
                ClassifierSuccess = true;
                BeginMOT = false;
                ping = false;
                Stop = false;
                RABCompare = false;
                RABConfirm = false;
                ConfirmRM = false;
                TestRM = false;
                ConfirmLM = false;
                TestLM = false;
                TestStraight = false;
                ConfirmStraight = false;
                TestLap = false;
                ConfirmLap = 0;
                LapDelay = 0;
                LapStart = 0;
                LapCount = false;
                PingWait.clear();
                StopWait.clear();
                RABWait.clear();
                MotorWait.clear();
                StraightWait.clear();
                LapWait.clear();
            }
        }

        if (std::find(std::begin(Doctors), std::end(Doctors),packet.Data[0]+100) != std::end(Doctors) && Faulty == 0 && !Doctor) {
            if (bearing > 0 && bearing < 90) {
                Left = 1;
                Right = 0;
                Quarantine = true;
            }
            else if (bearing < 0 && bearing > -90) {
                Left = 0;
                Right = 1;
                Quarantine = true;
            }
        }
        if (Doctor && packet.Data[0]+100 == DoctorsOrder) {
            Ambulance.push_back(bearing);
            Ambulance.push_back(range);
        }


    }
    if (DoctorBounce == 0 && Doctor && Time > TimeStart || ConfirmLap == 1 && !DiagnosisConfirmed && Doctor) {


        if (DoctorBounce == 0 && Doctor) {
            for (int i = 0; i < Doctors.size(); i++) {
                if (Doctors.at(i) == ID) {
                    //std::cout << Doctors.at(i) << "sacks off " << std::endl;
                    Doctors.erase(Doctors.begin() + i);
                    //std::cout << Doctors.at(i) << std::endl;
                    Doctors.erase(Doctors.begin() + i);
                }
            }
        }
        else if (ConfirmLap == 1) {
            for (int i = 0; i < Doctors.size(); i++) {
                if (Doctor && Doctors.at(i) == ID) {
                    //std::cout << "erasing from doctors: " << Doctors.at(i) << std::endl;
                    Doctors.erase(Doctors.begin() + i);
                    Doctors.erase(Doctors.begin() + i);
                }
                else if (Faulty != 0 && Doctors.at(i) == ID) {
                    //std::cout << "erasing from doctors: " << Doctors.at(i) << std::endl;
                    Doctors.erase(Doctors.begin() + i);
                    Doctors.erase(Doctors.begin() + i - 1);
                }
            }
        }
        Doctor = false;
        UnderInvestigation = 0;
        DetectBodge->clear();
        DoctorsOrder = 0;
        Candidates.clear();
        TimeID = 0;
        TimeStart = 0;
        RValue = 0;
        Diagnosis = 0;
        Diagnosed = false;
        Ambulance.clear();
        Snapshot.clear();
        DiagCandidate = 0;
        ClassifierSuccess = true;
        BeginMOT = false;
        ping = false;
        Stop = false;
        RABCompare = false;
        RABConfirm = false;
        ConfirmRM = false;
        TestRM = false;
        ConfirmLM = false;
        TestLM = false;
        TestStraight = false;
        ConfirmStraight = false;
        TestLap = false;
        ConfirmLap = 0;
        LapDelay = 0;
        LapStart = 0;
        FaultResolved = false;
        LapCount = false;
        PingWait.clear();
        StopWait.clear();
        RABWait.clear();
        MotorWait.clear();
        StraightWait.clear();
        LapWait.clear();
        if (FaultResolved) {
            Faulty = 0;
            FaultResolved = false;
            hang = false;
            power = false;
        }
    }
    // Reset investigation status
    if (InvestigateBounce == 0 && UnderInvestigation != 0) {
        UnderInvestigation = 0;
        DetectBodge->clear();
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
    if (!Quarantine && !Doctor && Behaviour == 4 && !Dead) {
        OmegaAlg();
    }
    // SEND DR TO FAULTY//
    if (Doctor && Ambulance.size() > 1 && !Dead) {
        DrPursue();
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
    if (!hang || hang && FaultResolved) {
        if (MidProx > 0) {
            AgentNew.push_back(1);

        }
        else {
            AgentNew.push_back(0);
        }
    }
    else if (hang && !FaultResolved) {
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
    if (!hang || hang && FaultResolved) {
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
    // WATCHDOG
    // F11 & F12 Hang Ext & Int
    if (!hang || hang && FaultResolved) {
        AgentNew.push_back(0);
        AgentNew.push_back(0);
    }
    else {
        AgentNew.push_back(1);
        AgentNew.push_back(1);
    }

    // Record Data for Future Comparison
    IntCoord->push_back(X);
    IntCoord->push_back(Y);
    TrueIntCoord->push_back(TrueX);
    TrueIntCoord->push_back(TrueY);

    // USE CLASSIFIER
    // Print Features To Log & Take Snapshot


    // FAULTY DIAGNOSTIC ROUTINES
    if (Faulty != 0 && BeginMOT && !Dead) {
        if (Stop) {
            LeftWheel = 0;
            RightWheel = 0;
            //std::cout << ID << " STOPS " << LeftWheel << ", " << RightWheel << std::endl;
        }
        if (RABCompare) {
            //std::cout << ID << " CSF TEST " << std::endl;
            CSFTest();
        }
        if (TestLM) {
            //std::cout << ID << " CMF TEST " << std::endl;
            CMFTestLeft();
        }
        if (TestRM) {
            //std::cout << ID << " CMF TEST " << std::endl;
            CMFTestRight();
        }
        if (TestStraight) {
            //std::cout << ID << " PMF TEST " << std::endl;
            PMFTest();
        }
        if (TestLap) {
            //std::cout << ID << " PSF TEST " << std::endl;
            PSFTest();
        }

        if (DiagnosisConfirm == 4) {
            //std::cout << ID << " CMF TEST CONFIRM " << std::endl;
            if (TestLM) {
                CMFTestLeft();
            }
            if (TestRM) {
                CMFTestRight();
            }
        }
        if (DiagnosisConfirm == 5) {
            //std::cout << ID << " PMF TEST CONFIRM " << std::endl;
            PMFTest();
        }
        if (DiagnosisConfirm == 6) {
            //std::cout << ID << " PSF TEST CONFIRM " << std::endl;
            PSFTest();
        }
    }


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

    if (DiagnosisConfirmed && Faulty != 0 && !Dead) {
        if (resetCounter == 0) {
            resetCounter = Time;
        }
        if (Time - resetCounter > 0) {
            FaultyReset();
            resetCounter = 0;
        }
    }

    //std::cout << ID << ": " << MemoryLogNew->size() << std::endl;



    if (SaveMemory && Time == ExperimentLength-2 && !Dead) {
        MemorySize.push_back(-ID);
        MemorySize.push_back(MemoryLogNew->size());
    }
    if (SaveMemory && Time == ExperimentLength-1 && !Dead) {
        for (int k = 0; k < MemorySize.size(); k++) {
            if (MemorySize.at(k) == MemoryLogNew->capacity() || MemorySize.at(k) == *max_element(MemorySize.begin(), MemorySize.end())) {
                MemoryIndex = k;
                break;
            }
        }
        if (ID == -MemorySize.at(MemoryIndex-1)) {
            ImmortalID = ID;
        }
    }

    /*if (Time % 10000 == 0 && ID == 100) {
        std::cout << MemoryCorrect << ", " << MemoryIncorrectBodge << ", " << MemoryIncorrect << ", " << MemoryCorrectBodge << std::endl;
        std::cout << MOTCorrect << ", " << MOTIncorrectBodge << ", " << MOTIncorrect << ", " << MOTCorrectBodge << ", " << MOTNull << std::endl;
        std::cout << Total << ", " << TrueTotal << std::endl;
        std::cout << "Faults in play: " << FaultsInPlay << std::endl;
    }*/
   /* if (Time == ExperimentLength && !StuckBounce) {
        //std::cout << "END" << std::endl;
        StuckBounce = true;
        Real ClassPer = MemoryCorrect/TrueTotal;
        Real ClassFailPer = MemoryIncorrect/(MemoryCorrect+MemoryIncorrect+MemoryCorrectBodge + MemoryIncorrectBodge);


        Real AvCor = 0;
        if (CorrCoeff.size()>0) {
            AvCor = std::accumulate(CorrCoeff.begin(), CorrCoeff.end(), 0.0) / CorrCoeff.size();
        }
        Real FailCor = 0;
        std::string folderName = std::to_string(foldernum-1);
        std::string seedFolder = std::to_string(CSimulator::GetInstance().GetRandomSeed());
        std::string slashyboi = "/";
        std::string path = folderName+slashyboi+seedFolder;
        mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        DataFile.open (folderName+"/"+seedFolder+"/Data.csv", std::ios_base::app);
        if (FailCoeff.size() > 0) {
            FailCor = std::accumulate(FailCoeff.begin(), FailCoeff.end(), 0.0)/FailCoeff.size();
        }
        Real AvDetTime = 0;
        if (DetectTime.size()>0) {
            AvDetTime = std::accumulate(DetectTime.begin(), DetectTime.end(), 0.0)/DetectTime.size();
        }
        DataFile << "Abs Total, "   << "MemoryPer, "  << "ClassFail, " << "Total Failure, " << "Avg Correlation, "  << "Avg Detection Time " <<  std::endl;
        DataFile  << TrueTotal << ", " << ClassPer << ", "  << ClassFailPer << ", " << DeadTotal << ", " << AvCor << ", "  << AvDetTime <<  std::endl;
        DataFile.close();
        std::cout << "AVERAGE TIMES " << AvDetTime << std::endl;
        if (SaveMemory) {
            MemorySave.open (folderName+"/"+seedFolder+"/MemoryLog.csv", std::ios_base::app);
            for (int j = 0; j < MemoryLogNew->size(); j++) {
                MemorySave << MemoryLogNew->at(j) << ",";
            }
            MemorySave << 8 << std::endl;
            MemorySave.close();
        }

        CSimulator::GetInstance().Terminate();
    }*/







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