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


std::vector<int> FaultyIDs;
std::vector<int> Doctors;
std::vector<Real> DetectTime;
std::vector<int> MemoryTimes;
std::vector<int> MOTTimes;
std::vector<Real> CorrCoeff;
std::vector<Real> FailCoeff;
std::vector<int> MemorySize;


bool StuckBounce = false;
int MemoryBits = 5;
int ExperimentLength = 36000;
int FaultsInPlay = 0;
int PMFangle = 60;
int MemoryIndex;
bool SaveMemory = false;
int FaultDelay = 10;
int ConfirmDelay = 5;
int ImmortalID;
int Behaviour = 0;
int BehaviourCount = 0;
int Omega = 3;
Real OmegaGap = 35;
int PowerCycle [2] =  {1,2};
int SensorReplacement [2] = {3,6};
int MotorReplacement [2] = {4,5};
std::ofstream SnapshotFile;
std::ofstream DataFile;
std::ofstream SpartanPercent;
int foldernum;
Real Fail = 0;
int MOT = 0;
Real Total = 0;
int TrueTotal;
Real Class = 0;
int timeweight = 0;



// SENSITIVITY ANALYSIS OBJECTS
// CONST
int FaultProb = 10000;


// VAR
//ArenaSide = RobotNumber/5
Real SimilarityThreshold = 0.66;
Real ActiveThreshold = (0.5*(1-SimilarityThreshold)) + SimilarityThreshold;
int DetectDelay = 100;
Real DetectRatio = 1;
Real BearingNoise = 0.123;
Real YawCoordinateNoise = 0.00035;
Real CoordinateNoise = 0.000025;
int Objects = 0;

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
    virtual void ObstacleAv ();
    virtual void Aggregation ();
    virtual void Flocking ();
    virtual void OmegaAlg ();
    virtual void DrPursue();
    virtual void TakeSnapshot();
    virtual void Classify();
    virtual void ActiveMemory();
    virtual void DoctorReset();
    virtual void FaultyReset();
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

    CVector2 NeighbourDistance;

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
    int MotorRand = 0;
    Real RValue;
    bool hang = false;
    bool power = false;
    bool Doctor = false;
    int DoctorsOrder = 0;
    std::vector<double> Ambulance;
    int Discrep = 0;
    int UnderInvestigation;
    std::vector<int> Snapshot;
    std::vector<Real> Candidates;
    int TimeID = 0;
    int TimeStart;
    bool Quarantine = false;

    // DIAGNOSTICS
    bool wall;
    bool DiagBounce = false;
    bool SnapTaken = false;
    bool Eligibility = false;
    int Diagnosis = 0;
    bool Diagnosed = false;
    bool FailReset = false;
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



};

#endif
