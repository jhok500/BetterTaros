/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */

#ifndef FOOTBOT_DIFFUSION_H
#define FOOTBOT_DIFFUSION_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <boost/circular_buffer.hpp>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <numeric>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include "RadioActuator.h"
#include "RadioSensor.h"


/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;
std::vector<int> FeatureVector;

std::vector<int> FaultyIDs;
std::vector<int> Doctors;
std::vector<Real> DetectTime;
std::vector<Real> DetectEDTime;
std::vector<int> MemoryTimes;
std::vector<int> MOTTimes;
std::vector<Real> CorrCoeff;
std::vector<Real> FailCoeff;
std::vector<int> MemorySize;
std::vector<int> EscapedFaults;
std::vector<int> ClassBodgeFaults;

boost::circular_buffer<double>* Dis2Beacons;

bool FirstBoy = false;
int FirstTime = 0;
Real FirstAvDis2Beac = 0;
Real DistAtFirst = 0;
bool HalfBoy = false;
int HalfTime = 0;
Real DistAtHalf = 0;
Real HalfAvDis2Beac = 0;
bool AllBoys = false;
int AllTime = 0;
Real AllAvDis2Beac = 0;
Real DistAtAll = 0;
int countyboy = 0;
int NumFault = 104;



Real DecH1 = 0;
Real DecH2 = 0;
Real DecH3 = 0;
Real DecH4 = 0;
Real DecH5 = 0;
Real DecH6 = 0;
Real DecCor = 0;

int FaultType = 2;
bool FaultAnalysis = false;
bool SaveBounce = false;
bool StuckBounce = false;
int MemoryBits = 18;
int ExperimentLength = 3000;
int faultCount = 0;
int FaultsInPlay = 0;
int PMFangle = 60;
int MemoryIndex;
bool SaveMemory = false;
bool ImportMemory = true;
int FaultDelay = 10;
int ConfirmDelay = 5;
int ImmortalID;
int Behaviour = 1;
int BehaviourCount = 0;
int Omega = 3;
Real OmegaGap = 35;
int PowerCycle [2] =  {1,2};
int SensorReplacement [2] = {3,6};
int MotorReplacement [2] = {4,5};
//std::ofstream SnapshotFile;
std::ofstream DataFile;
std::ofstream MemorySave;
//std::ofstream SpartanPercent;
int foldernum;
Real Fail = 0;
int MOT = 0;
Real Total = 0;
Real ElligibleMOT = 0;
int TrueTotal;
Real Class = 0;
int timeweight = 0;
int DeadTotal = 0;



Real MemoryCorrect = 0;
Real MemoryIncorrectBodge = 0;
Real MemoryIncorrect = 0;
Real MemoryCorrectBodge = 0;
Real MemoryFly = 0;
Real MOTCorrect = 0;
Real MOTIncorrect = 0;
Real MOTIncorrectBodge = 0;
Real MOTCorrectBodge = 0;
Real MOTNull = 0;
Real FaultMissed = 0;

// SENSITIVITY ANALYSIS OBJECTS
// CONST
int FaultProb = 10000;


// VAR
//ArenaSide = RobotNumber/5
Real SimilarityThreshold = 0.56;
Real ActiveThreshold = (0.5*(1-SimilarityThreshold)) + SimilarityThreshold;
int DetectDelay = 29;
Real DetectRatio = 0.86;
int doublecheck = 29;
Real BearingNoise = 0.123;
Real YawCoordinateNoise = 0.00035;
Real CoordinateNoise = 0.000025;
Real RangeNoiseMultiplier = 0.05;

//int Objects = 0;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotDiffusion : public CCI_Controller {

public:



   /* Class constructor. */
   CFootBotDiffusion();

   /* Class destructor. */
   virtual ~CFootBotDiffusion() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);


   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();
    CCI_RangeAndBearingSensor::TReadings GetRABSensorReadings();


   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual Real HeadingCorrect();
   virtual void BehaviourUpdate ();
   virtual void FaultInject ();
    virtual void FaultInjectOmega ();
    virtual void ObstacleAv ();
    virtual void Aggregation ();
    virtual void Flocking ();

    virtual void Decision();


   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

    inline CCI_DifferentialSteeringSensor::SReading GetWheelVelocity() const {

        return wheel_encoders->GetReading();

    }

    inline CCI_FootBotProximitySensor::TReadings GetProximity() const {

        return m_pcProximity->GetReadings();
    }
    inline Real GetSpoofLeftWheelVelocity() const {

        return LeftWheel;

    }
    inline Real GetSpoofRightWheelVelocity() const {

        return RightWheel;

    }


    Real NoiseLeft;
    Real NoiseRight;

    Real hangRight;
    Real hangLeft;

    //CVector2 NeighbourDistance;

protected:
    virtual CVector2 VectorToLight();

private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
    CCI_DifferentialSteeringSensor* wheel_encoders;
    //Real* timestep;
    CCI_RangeAndBearingActuator* range_and_bearing_actuator;
    CCI_RangeAndBearingSensor* range_and_bearing_sensor;
    CCI_FootBotLightSensor* m_pcLight;





   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_diffusion_controller> section.
    */

   /* Maximum tolerance for the angle between
    * the robot heading direction and
    * the closest obstacle detected. */
   CDegrees m_cAlpha;
   /* Maximum tolerance for the proximity reading between
    * the robot and the closest obstacle.
    * The proximity reading is 0 when nothing is detected
    * and grows exponentially to 1 when the obstacle is
    * touching the robot.
    */
   Real m_fDelta;
   /* Wheel speed. */
   Real m_fWheelVelocity;
   /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
   CRange<CRadians> m_cGoStraightAngleRange;
    CRadians cAngle;
    int ID;
    int Alone;
    int Left;
    int Right;
    bool wall;
    Real LeftWheel;
    Real RightWheel;
    Real X;
    Real Y;
    Real TrueX;
    Real TrueY;
    Real Heading;
    std::vector<Real> FlockHeadings;
    std::vector<Real> AbsoluteFlockBearing;
    std::vector<Real> OmegaCoordX;
    std::vector<Real> OmegaCoordY;
    std::vector<Real> FlockCoordX;
    std::vector<Real> FlockCoordY;
    std::vector<Real> FlockRange;
    std::vector<Real> FlockBearing;
    boost::circular_buffer<int>* MemoryLogNew;
    boost::circular_buffer<int>* DetectBodge;
    boost::circular_buffer<int>* ClassCheck;
    boost::circular_buffer<int>* ClassConfirm;

    boost::circular_buffer<int>* FaultyFeatureVectors;
    boost::circular_buffer<int>* MemoryBounce;
    boost::circular_buffer<Real>* YawHolder;
    boost::circular_buffer<Real>* IntCoord;
    boost::circular_buffer<Real>* TrueIntCoord;
    std::vector<int> AgentNew;
    std::vector<int> HangVector;
    int OmegaTimer = 0;
    std::vector<Real> OmegaTurnX;
    std::vector<Real> OmegaTurnY;
    int Faulty = 0;
    bool FaultResolved = false;
    int MotorRand = 0;
    Real RValue;
    bool hang = false;
    bool power = false;
    bool Doctor = false;
    int DoctorsOrder = 0;
    std::vector<double> Ambulance;
    int Discrep = 0;
    int UnderInvestigation = 0;
    std::vector<int> Snapshot;
    std::vector<Real> Candidates;
    int TimeID = 0;
    int TimeStart;
    bool Quarantine = false;

    // DIAGNOSTICS
    //bool wall;
    //bool DiagBounce = false;
    //bool SnapTaken = false;


    bool Eligibility = false;
    int Diagnosis = 0;
    int DiagnosisConfirm = 0;
    bool DiagnosisConfirmed = false;
    bool DiagnosisFailed = false;
    bool Dead = false;
    bool Diagnosed = false;
    bool BeginMOT = false;
    bool Stop = false;
    bool ping = false;
    bool RABCompare = false;
    bool RABConfirm = false;
    bool TestLM = false;
    bool ConfirmLM = false;
    bool TestRM = false;
    bool ConfirmRM = false;
    bool TestStraight = false;
    bool ConfirmStraight = false;
    bool TestLap = false;
    bool partialsensorboy = false;
    Real LapStart;
    bool LapCount = false;
    int LapDelay = 0;
    int ConfirmLap = 0;
    Real FaultStart = 0;
    std::vector<int> PingWait;
    std::vector<int> StopWait;
    std::vector<int> RABWait;
    std::vector<int> MotorWait;
    std::vector<int> StraightWait;
    std::vector<int> LapWait;
    Real DiagCandidate = 0;
    bool ActiBounce = false;
    int resetCounter = 0;
    bool BackToNorm = false;

    // CLASSIFICATION

    Real sumtop;
    Real topadd;
    Real bottomadd1;
    Real bottomadd2;
    Real sumbottom;
    Real sumbottom1;
    Real sumbottom2;
    Real MeanMem;
    Real MeanSnap;
    Real MemSum;
    int Jcount;
    Real Rcorr;
    Real FaultTime;
    bool ClassifierSuccess = true;
    bool ClassifyCheck = false;

    bool F1p = false;
    bool F2p = false;
    bool F3p = false;
    bool F4p = false;
    bool F5p = false;
    bool F1e = false;
    bool F2e = false;
    bool F3e = false;
    bool F4e = false;
    bool F5e = false;



};

#endif
