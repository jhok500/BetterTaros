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

//do you have any idea how much code a PhD candidate copy pastes in a year?

    if (myfile.is_open()) {
        while (getline(myfile, row)) {
            lineNumber++;
            if (lineNumber == lineNumberSought) {
                std::istringstream myRow(row);
                int i = 0;
                while (getline(myRow, cell, ',')) {
                    switch (i) {
                        case 0:
                            SimilarityThreshold = stof(cell);
                            break;
                        case 1:
                            DetectDelay = abs(stof(cell));
                            //DetectDelay = 5;
                            break;
                        case 2:
                            DetectRatio = stof(cell);
                            //DetectRatio = 1;
                            break;

                    }
                    i++;
                }
            }
        }

        myfile.close();
    }

//shit loads
    //std::cout << ParamRow << ", " << SimilarityThreshold << ", " << DetectDelay << ", " << DetectRatio << std::endl;
    IntCoord = new boost::circular_buffer<double>(2*2);
    TrueIntCoord = new boost::circular_buffer<double>(2*2);
    YawHolder = new boost::circular_buffer<double>(2);
    FaultyFeatureVectors = new boost::circular_buffer<int>(((6*2))*DetectDelay);
    MemoryLogNew = new boost::circular_buffer<int>(((DetectDelay*6*2)+2)*MemoryBits);
    DetectBodge = new boost::circular_buffer<int>(DetectDelay);
    MemoryBounce = new boost::circular_buffer<int>(1000);


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
    //std::cout << "Behaviour is " << Behaviour << std::endl;
}
/****************************************/

void CFootBotDiffusion::FaultInject() {
    srand(Time);
    Faulty = rand() % 6 + 1;
    if (Faulty == 4 || Faulty == 5) {
        if (Time % 2 == 0) {
            MotorRand = 1;
        }
        else {
            MotorRand = 2;
        }
    }
    FaultyIDs.push_back(ID);
    //std::cout << "Faulty Robot is " << ID << ": " << Faulty << std::endl;

    if (std::find(MemoryLogNew->begin(), MemoryLogNew->end(),-Faulty) != MemoryLogNew->end()) {
        Eligibility = true;
    }
    else {
        Eligibility = false;
    }
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
            //std::cout << ID << " has arrived" << std::endl;
        } else {
            if (abs(Ambulance.at(0)) < 20) {
                Left = 1;
                Right = 1;
            } else {
                if (Ambulance.at(0) < 0) {
                    Left = 1;
                    Right = 0;
                } else {
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
    //SnapshotFile.open ("SnapShot.csv", std::ios_base::app);
    for (int i = 0; i < FaultyFeatureVectors->size(); i++) {
        Snapshot.push_back(FaultyFeatureVectors->at(i));
        //SnapshotFile << FaultyFeatureVectors->at(i) << ", ";
        //std::cout << FaultyFeatureVectors->at(i) << std::endl;
        //SnapshotFile.close();
    }
}
/****************************************/
void CFootBotDiffusion::Classify() {

    if (MemoryLogNew->size() >= (MemoryLogNew->capacity() / MemoryBits)) {
        //std::cout << ID << " Run Classifier On " << DoctorsOrder << std::endl;
        MeanMem = 0;
        MeanSnap = 0;
        Jcount = 0;
        for (int i = 0; i < MemoryLogNew->size(); i++) {
            MemSum = 0;
            if (MemoryLogNew->at(i) < 0) {
                //TestCase.push_back(MemoryLog->at(i));
                DiagCandidate = MemoryLogNew->at(i);
                //std::cout << "DiagCandidate: " << DiagCandidate << std::endl;
                FaultTime = -MemoryLogNew->at(i+1);
                for (int k = i+2; k < i + Snapshot.size()+2; k++) {
                    MemSum = MemSum + MemoryLogNew->at(k);
                    //std::cout << "K " << MemoryLog->at(k) << std::endl;
                }
                //std::cout << "MemSum = " << MemSum << " Snapshot Size = " << Snapshot.size() << std::endl;
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
                //TestCase.push_back(MemoryLog->at(i));
                topadd = (Snapshot.at(Jcount) - MeanSnap) * (MemoryLogNew->at(i) - MeanMem);
                sumtop = sumtop + topadd;
                bottomadd1 = pow((Snapshot.at(Jcount) - MeanSnap), 2);
                bottomadd2 = pow((MemoryLogNew->at(i) - MeanMem), 2);
                sumbottom1 = sumbottom1 + bottomadd1;
                sumbottom2 = sumbottom2 + bottomadd2;
                //std::cout << "SumTop: " << sumtop << std::endl;
                //std::cout << "SumBottom: " << sumbottom1 << ", " << sumbottom2 << std::endl;
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
                    else {
                        //std::cout << "Not This One" << std::endl;
                    }
                }
            }
        }
        if (Candidates.size() == 0) {
            //std::cout << "Run Diagnostics" << std::endl;
            ClassifierSuccess = false;
        }
        else {
            for (int i = 0; i < Candidates.size(); i++) {
                if (Candidates.at(i)== *max_element(Candidates.begin(), Candidates.end())) {
                    //std::cout << "CLASS INFO " <<  Candidates.at(i) << ", " << Candidates.at(i+1) << ", " << Candidates.at(i+2) << std::endl;
                    RValue = Candidates.at(i);
                    Diagnosis = -Candidates.at(i+1);
                    TimeID = -Candidates.at(i+2);
                    //std::cout << "Classified as " << Diagnosis << " with " << Candidates.at(i)*100 << "% similarity" << std::endl;
                    break;
                }
            }
        }
    }
    else {
        //std::cout << "Run Diagnostics, snapshot size " << Snapshot.size() << std::endl;
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
            //MemoryLogNew->erase(MemoryLogNew->begin() + (i - 1));
        }
    }
    ActiBounce = false;

}
/****************************************/
void CFootBotDiffusion::DoctorReset() {
    DetectBodge->clear();
    TrueTotal++;

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
    if (Eligibility || ClassifierSuccess) {
        Total++;
        //std::cout << "Total: " << Total << std::endl;
    }
    //std::cout << MemoryLogNew->size() << ": " << MemoryLogNew->capacity() << std::endl;
    if (ClassifierSuccess) {
        //std::cout << "DIAGNOSED (CLASSIFIER)" << std::endl;
        Class++;
        //std::cout << "Total Class: " << Class << std::endl;
        MemoryTimes.push_back(Time);
        CorrCoeff.push_back(RValue);
    } else {
        //std::cout << "DIAGNOSED (MOT)" << std::endl;
        int TimeTaken = Time - TimeStart;
        MOTTimes.push_back(Time);
        if (Eligibility) {
            MOT++;
            //std::cout << "Total MOT: " << MOT << std::endl;
        }
    }
    Candidates.clear();
    TimeID = 0;
    TimeStart = 0;
    RValue = 0;
    Diagnosis = 0;
    Diagnosed = false;
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
    ConfirmLap = 0;
    LapDelay = 0;
    LapStart = 0;
    LapCount = false;
    FailReset = false;
    PingWait.clear();
    StopWait.clear();
    RABWait.clear();
    MotorWait.clear();
    StraightWait.clear();
    LapWait.clear();
}
/****************************************/
void CFootBotDiffusion::FaultyReset() {
    if ((std::find(std::begin(PowerCycle), std::end(PowerCycle), Diagnosis) != std::end(PowerCycle)
         && std::find(std::begin(PowerCycle), std::end(PowerCycle), Faulty) != std::end(PowerCycle)) ||
        (std::find(std::begin(MotorReplacement), std::end(MotorReplacement), Diagnosis) != std::end(MotorReplacement)
         && std::find(std::begin(MotorReplacement), std::end(MotorReplacement), Faulty) != std::end(MotorReplacement)) ||
        (std::find(std::begin(SensorReplacement), std::end(SensorReplacement), Diagnosis) != std::end(SensorReplacement)
         && std::find(std::begin(SensorReplacement), std::end(SensorReplacement), Faulty) != std::end(SensorReplacement))) {
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
        for (int i = 0; i < FaultyIDs.size(); i++) {
            if (FaultyIDs.at(i) == ID) {
                //std::cout << "ERASING: " << FaultyIDs.at(i) << " FROM FAULTY IDS" << std::endl;
                FaultyIDs.erase(FaultyIDs.begin() + i);

            }
        }
        for (int i = 0; i < Doctors.size(); i++) {
            if (Doctors.at(i) == ID) {
                //std::cout << "ERASING: " << Doctors.at(i) << " AND " << Doctors.at(i-1) << " FROM DETECTED IDS" << std::endl;
                Doctors.erase(Doctors.begin() + i);
                Doctors.erase(Doctors.begin() + i - 1);
            }
        }
        if (std::find(std::begin(PowerCycle), std::end(PowerCycle), Diagnosis) != std::end(PowerCycle)
            && std::find(std::begin(PowerCycle), std::end(PowerCycle), Faulty) != std::end(PowerCycle)) {
            //std::cout << "Fault = " << Faulty << ", Diagnosis = " << Diagnosis << ", Recovery: Cycle Power" << std::endl;
        } else if (std::find(std::begin(MotorReplacement), std::end(MotorReplacement), Diagnosis) !=
                   std::end(MotorReplacement)
                   && std::find(std::begin(MotorReplacement), std::end(MotorReplacement), Faulty) !=
                      std::end(MotorReplacement)) {
            //std::cout << "Fault = " << Faulty << ", Diagnosis = " << Diagnosis << ", Recovery: Replace Motor" << std::endl;

        } else if (std::find(std::begin(SensorReplacement), std::end(SensorReplacement), Diagnosis) != std::end(SensorReplacement)
                   && std::find(std::begin(SensorReplacement), std::end(SensorReplacement), Faulty) != std::end(SensorReplacement)) {
            //std::cout << "Fault = " << Faulty << ", Diagnosis = " << Diagnosis << ", Recovery: Replace Sensor" << std::endl;
        }
        Faulty = 0;
        Diagnosed = true;
        FaultsInPlay--;
        FailReset = false;
        FaultStart = 0;
        RValue = 0;
    }
    else {
        //std::cout << "Fault = " << Faulty << ", Diagnosis = " << Diagnosis << ", FAILURE" << std::endl;
        Fail++;
        //std::cout << "Total Fails: " << Fail << std::endl;
        if (RValue != 0) {
            FailCoeff.push_back(RValue);
        }
        if (!BeginMOT) {
            BeginMOT = true;
            FailReset = true;
            Diagnosis = 0;
        }
        else {
            Diagnosis = Faulty;
        }
        //std::cout << "Try Diagnosis From Scratch" << std::endl;

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
    if (Faulty == 0 && Diagnosed && !Doctor) {
        Diagnosed = false;
        Diagnosis = 0;
    }
    // FAULT INJECTION
    int prob = rand() % FaultProb;
    if (prob > FaultProb - 2 && Faulty == 0 && !Doctor && FaultsInPlay < 0.5*RobotNumber) {
        FaultsInPlay++;
        FaultStart = Time;
        FaultInject();
    }
    // BEHAVIOUR SWITCH
    if (Time > (BehaviourCount*5000)) {
        BehaviourUpdate();
        //Behaviour = 2;
    }

    //if (Time > 10000) {std::cout << "AVERAGE FAIL COEFF = " << std::accumulate(FailCoeff.begin(), FailCoeff.end(), 0.0)/FailCoeff.size() << std::endl;}

    std::ostringstream oss;
    CLoopFunctions CS;
    CSpace::TMapPerType& foot_bots = CS.GetSpace().GetEntitiesByType("foot-bot");

    Quarantine = false;
    // DETECT RESET
    if (std::find(std::begin(Doctors), std::end(Doctors),UnderInvestigation) != Doctors.end() && UnderInvestigation != DoctorsOrder) {
        UnderInvestigation = 0;
        DetectBodge->clear();
        //std::cout << ID << " forgets about " << UnderInvestigation << " ALT" << std::endl;
    }

    Ambulance.clear();
    Alone = 0;
    int MidProxCoord = 0;
    int CloseProxCoord = 0;
    int MidProx = 0;
    int CloseProx = 0;
    int InvestigateBounce = 0;
    for (CCI_RangeAndBearingSensor::SPacket packet : packets) {
        double bearing = ToDegrees(packet.HorizontalBearing).GetValue();
        std::normal_distribution<double> RABNoise(0,packet.Range*0.05);
        double range = packet.Range + RABNoise(generator);
        for (auto &map_element : foot_bots){
            CFootBotEntity &foot_bot = *any_cast<CFootBotEntity *>(map_element.second);
            CFootBotDiffusion &controller = dynamic_cast<CFootBotDiffusion &>(foot_bot.GetControllableEntity().GetController());
            // Alignment
            if (controller.ID == packet.Data[0]+100) {
                // Neighbours are in range
                MidProxCoord = MidProxCoord + 1;
                // Neighbours are in close range
                if (sqrt(pow(X - controller.X, 2) + pow(Y-controller.Y, 2)) < 0.3) {
                    CloseProxCoord = CloseProxCoord + 1;
                }
                // Normal Sensor Function
                if (abs(bearing) > PMFangle && Faulty == 6 || Faulty == 3 || power || hang) {
                }
                else {
                    Alone++;
                    FlockBearing.push_back(bearing);
                    //std::cout << atan2((controller.TrueY - TrueY), (controller.TrueX - TrueX)) * 180 / ARGOS_PI << std::endl;
                    AbsoluteFlockBearing.push_back(atan2((controller.TrueY - TrueY), (controller.TrueX - TrueX)) * 180 / ARGOS_PI);
                    OmegaCoordX.push_back(cos(atan2((controller.TrueY - TrueY), (controller.TrueX - TrueX))));
                    OmegaCoordY.push_back(sin(atan2((controller.TrueY - TrueY), (controller.TrueX - TrueX))));
                    FlockRange.push_back(range);
                    MidProx = MidProx + 1;
                    if (packet.Range < 30) {
                        CloseProx = CloseProx + 1;
                    }
                }
                // Gather Behaviour Data
                FlockCoordX.push_back(controller.X);
                FlockCoordY.push_back(controller.Y);
                FlockHeadings.push_back(controller.Heading);
                // DETECTION
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
                    if (std::accumulate(DetectBodge->begin(), DetectBodge->end(), 0.0) >= DetectDelay*DetectRatio && DetectBodge->size() == DetectBodge->capacity()
                            && FaultyFeatureVectors->size() == FaultyFeatureVectors->capacity()) {
                        //std::cout << "DETECTED" << std::endl;
                        TimeStart = Time;
                        Doctor = true;
                        DoctorsOrder = UnderInvestigation;
                        //std::cout << "DR FOR " << DoctorsOrder << " IS " << ID << std::endl;
                        Doctors.push_back(ID);
                        Doctors.push_back(DoctorsOrder);
                        DetectTime.push_back(Time - controller.FaultStart);
                        TakeSnapshot();
                        Classify();
                    }
                    //SHARE MEMORY
                    if (std::find(MemoryBounce->begin(), MemoryBounce->end(), controller.ID) == MemoryBounce->end()) {
                        //std::cout << ID << " Tries " << controller.ID << std::endl;
                        MemoryBounce->push_back(controller.ID);
                        for (int i = 0; i < controller.MemoryLogNew->size(); i++) {
                            if (controller.MemoryLogNew->at(i) < 0 &&
                                (std::find(MemoryLogNew->begin(), MemoryLogNew->end(),
                                           controller.MemoryLogNew->at(i + 1)) == MemoryLogNew->end())) {
                                //std::cout << ID << " Shares With " << controller.ID << std::endl;

                                for (int j = i; j < i + ((12 * DetectDelay) + 2); j++) {
                                    MemoryLogNew->push_back(controller.MemoryLogNew->at(j));
                                }
                            }
                        }
                    }
                    else {
                        //std::cout << ID << " doesn't share with " << controller.ID << std::endl;
                        MemoryBounce->push_back(0);}
                }

                // DIAGNOSTIC ROUTINE
                // Faulty
                if (Faulty != 0 && ID == controller.DoctorsOrder) {
                    if (controller.Diagnosis != 0 && !FailReset) {
                        Diagnosis = controller.Diagnosis;
                        //std::cout << ID << " GOT MY DIAGNOSIS " << Diagnosis << " from " << controller.ID << std::endl;
                        if (controller.RValue != 0) {
                            RValue = controller.RValue;
                        }
                    }
                    if (controller.BeginMOT && FailReset) {
                        FailReset = false;
                        //std::cout << "RESET BY " << controller.ID << " W/ DIAGNOSIS " << controller.Diagnosis << std::endl;
                    }
                    if (controller.BeginMOT && !Diagnosed) {
                        //std::cout << "Faulty Beginning Diagnosis" << std::endl;
                        BeginMOT = true;
                        if (controller.ping && Faulty != 2) {
                            ping = true;
                            //std::cout << "Faulty Pinging" << std::endl;
                        }
                        if (controller.Stop && Faulty != 1) {
                            Stop = true;
                            //std::cout << "Faulty Stopping" << std::endl;
                        }
                        if (controller.RABCompare) {
                            RABCompare = true;
                            //std::cout << "Testing Sensor" << std::endl;
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
                        if (controller.TestLap  && ConfirmLap == 0){
                            TestLap = true;
                            //std::cout << "Testing Lap" << std::endl;
                        }
                    }
                }
                // Doctor
                /*if (controller.FailReset) {
                    std::cout << controller.ID << " HAS TRUE FAIL RESET AS DETERMINED BY " << ID << std::endl;
                }*/
                if (Doctor && controller.ID == DoctorsOrder && controller.FailReset && !BeginMOT) {
                    BeginMOT = true;
                    Diagnosis = 0;
                    ClassifierSuccess = false;
                    //std::cout << "Class failed now try MOT" << std::endl;
                }
                if (Doctor && controller.ID == DoctorsOrder && controller.BeginMOT && !controller.Diagnosed) {
                    //std::cout << "Doctor Beginning Diagnosis" << std::endl;
                    if (!controller.ping) {
                        ping = true;
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
                            Diagnosis = 4;
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
                            Diagnosis = 4;
                        }
                    }
                    else if (TestRM && controller.ConfirmRM) {
                        TestStraight = true;
                        TestRM = false;
                    }
                    if (TestStraight && !controller.ConfirmStraight) {
                        StraightWait.push_back(1);
                        if (std::accumulate(StraightWait.begin(), StraightWait.end(), 0.0) > FaultDelay) {
                            Diagnosis = 5;
                        }
                    }
                    else if (TestStraight && controller.ConfirmStraight) {
                        TestLap = true;
                        TestStraight = false;
                    }
                    if (TestLap && controller.ConfirmLap == -1) {
                        LapWait.push_back(1);
                        if (std::accumulate(LapWait.begin(), LapWait.end(), 0.0) > FaultDelay) {
                            Diagnosis = 6;
                        }
                    }
                    if (Diagnosis != 0 && Diagnosis != controller.Faulty) {
                        if (Diagnosis == 3 && controller.Faulty == 6) {}
                        else {
                            //std::cout << "MOT FAILED, CHANGING ACCORDINGLY" << std::endl;
                            Diagnosis = controller.Faulty;
                        }
                    }
                }
                if (Doctor && !Diagnosed && controller.ID == DoctorsOrder && controller.Diagnosed) {

                    Diagnosed = true;
                }
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
        if (InvestigateBounce == 0) {
            UnderInvestigation = 0;
            DetectBodge->clear();
        }
    }
    // BEHAVIOURS

    // OBSTACLE AVOIDANCE
    if (!Quarantine && !Doctor && Behaviour == 1) {
        ObstacleAv();
    }
    // AGGREGATION
    if (!Quarantine && !Doctor && Behaviour == 2) {
        Aggregation();
    }
    // FLOCKING
    if (!Quarantine && !Doctor && Behaviour == 3) {
        Flocking();
    }
    if (!Quarantine && !Doctor && Behaviour == 4) {
        OmegaAlg();
    }
    // SEND DR TO FAULTY//
    if (Doctor && Ambulance.size() > 1) {
        DrPursue();
    }


    // SET NORMAL WHEEL VALUES
    RightWheel = (m_fWheelVelocity*Right);
    LeftWheel = (m_fWheelVelocity*Left);
    NoiseLeft = 0;
    NoiseRight = 0;
    // COMPLETE SENSOR FAULT //
    if (Faulty == 3) {
        RightWheel = m_fWheelVelocity;
        LeftWheel = m_fWheelVelocity;
    }
    // POWER FAILURE //
    if (Faulty == 2) {
        power = true;
        RightWheel = 0;
        LeftWheel = 0;
    }
    // SOFTWARE HANG //
    if (Faulty == 1) {
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
    else {
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
    else {
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
        if (!hang) {
            if ((Velocity_Wheels / 10) / 100 > 0.0045) {
                AgentNew.push_back(1);
            }
            else {
                AgentNew.push_back(0);
            }
        }
        else {
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
        if (!hang) {
            if (Velocity_Wheels / 100 > 0.01) {
                AgentNew.push_back(1);
            }
            else {
                AgentNew.push_back(0);
            }
        }
        else {
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
        if (!hang) {
            if (Difference_Wheels == 0) {
                AgentNew.push_back(0);
            }
            else {
                AgentNew.push_back(1);
            }
        }
        else {
            AgentNew.push_back(HangVector[4]);
        }
    }
    // WATCHDOG
    // F11 & F12 Hang Ext & Int
    if (!hang) {
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
    if (Faulty != 0 && BeginMOT) {
        if (Stop) {
            LeftWheel = 0;
            RightWheel = 0;
        }
        if (RABCompare) {
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
        if (TestLM) {
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
        if (TestRM) {
            Right = 1;
            RightWheel = (m_fWheelVelocity * Right);
            if (AgentNew.at(7) == 0) {
                MotorWait.clear();
            }
            else {
                MotorWait.push_back(1);
                if (std::accumulate(MotorWait.begin(), MotorWait.end(), 0.0) > ConfirmDelay) {
                    ConfirmRM = true;
                    TestRM = false;
                    MotorWait.clear();
                }
            }
        }
        if (TestStraight) {
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
        if (TestLap) {
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
            if (AgentNew.at(2) == 0) {
                LapWait.push_back(1);
                if (std::accumulate(LapWait.begin(), LapWait.end(), 0.0) > ConfirmDelay) {
                    ConfirmLap = -1;
                }
            }
        }
    }
    /*if (Faulty != 0) {
        for (int j = 0; j < AgentNew.size() ; j++) {
            std::cout << ID << ": " << AgentNew.at(j) << std::endl;
        }
    }*/


    // MOTOR FAULTS //
    if (Faulty == 4) {
        if (MotorRand == 1) {
            NoiseLeft = -LeftWheel;
        }
        else if (MotorRand == 2){
            NoiseRight = -RightWheel;
        }
    }
    if (Faulty == 5) {
        if (MotorRand == 1) {
            NoiseLeft = -0.5*LeftWheel;
        }
        else if (MotorRand == 2){
            NoiseRight = -0.5*RightWheel;
        }
    }

    // SET CONTROLLER VALUES

    m_pcWheels->SetLinearVelocity(LeftWheel + NoiseLeft, RightWheel + NoiseRight);

    /*if (BeginMOT) {
        std::cout << ID << " True! AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
    }
    else {std::cout << ID << " False!" << std::endl;};*/


    // FAULTY RESET

    if (Diagnosis !=0 && Faulty != 0) {

        FaultyReset();
    }

    // DOCTOR RESET
    if (Diagnosed && Doctor) {
        DoctorReset();
    }

    /*if (SaveMemory && Time == ExperimentLength-2) {
        MemorySize.push_back(-ID);
        MemorySize.push_back(MemoryLogNew->size());
    }
    if (SaveMemory && Time == ExperimentLength-1) {
        for (int k = 0; k < MemorySize.size(); k++) {
            if (MemorySize.at(k) == MemoryLogNew->capacity() || MemorySize.at(k) == *max_element(MemorySize.begin(), MemorySize.end())) {
                MemoryIndex = k;
                break;
            }
        }
        if (ID == -MemorySize.at(MemoryIndex-1)) {
            ImmortalID = ID;
        }
    }*/



    if (Time == ExperimentLength && !StuckBounce && !SaveMemory || Time == ExperimentLength && !StuckBounce && ID == ImmortalID && SaveMemory) {
        //std::cout << "END" << std::endl;
        StuckBounce = true;
        Real ClassPer = Class/Total;
        Real ClassFailPer = Fail/Total;
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

        /*std::string folderName = std::to_string(foldernum);
        mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        SpartanPercent.open (folderName + "/ClassFailPercent.csv", std::ios_base::app);
        SpartanPercent << CSimulator::GetInstance().GetRandomSeed() << "," << ClassFailPer;
        SpartanPercent.close();*/
        Real MaxFail = 0;
        if (FailCoeff.size() > 0) {
            FailCor = std::accumulate(FailCoeff.begin(), FailCoeff.end(), 0.0)/FailCoeff.size();
            MaxFail = *max_element(FailCoeff.begin(), FailCoeff.end());
        }
        Real AvDetTime = 0;
        if (DetectTime.size()>0) {
            AvDetTime = std::accumulate(DetectTime.begin(), DetectTime.end(), 0.0)/DetectTime.size();
        }
        DataFile << "Abs Total, " << "Elligible Total, "  << "MemoryTot, "  <<
        "FailTot, "<< "%Memory,"  << "%Failure,"  << "Avg Correlation, "  << "Avg Corr Fail, " <<
        "Max Fail Coeff, "  << "Avg Detection Time, " <<  std::endl;
        DataFile  << TrueTotal << ", " << Total << ", " << Class << ", "  << Fail << ", " << ClassPer << ", " << ClassFailPer << ", "
        << AvCor << ", " << FailCor << ", " << MaxFail << ", " << AvDetTime << std::endl;


        DataFile.close();
    }
    timeweight++;
    if (timeweight == RobotNumber) {
        //std::cout << Time << std::endl;
        timeweight = 0;

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