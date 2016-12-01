#ifndef EPUCK_OMEGA_ALGORITHM_H
#define EPUCK_OMEGA_ALGORITHM_H

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

using namespace argos;

class CEPuckOmegaAlgorithm : public CCI_Controller
{

public:

	CEPuckOmegaAlgorithm();
	virtual ~CEPuckOmegaAlgorithm() {}

	virtual void Init(TConfigurationNode& t_node);
	virtual void ControlStep();
	virtual void Reset() {}
	virtual void Destroy();

	void InjectFault(std::string type);
	void SetFaultyMotorSpeedScaling(double left, double right);
	bool IsFaulty();

	enum State {forward, avoidance, coherence};
	State state;

	int aggregation_timer;
	double distance_turned;
	CDegrees heading;

	CCI_LEDsActuator* leds;

	bool communication_failure = false;
	bool draw_id;
	std::string draw_sensor_range;

private:

	void OmegaAlgorithm(CCI_RangeAndBearingSensor::TReadings packets);
	void SetMotorSpeeds(double left, double right);

	CCI_RangeAndBearingSensor::TReadings GetRABSensorReadings();

	CCI_DifferentialSteeringActuator* wheels;
	CCI_RangeAndBearingActuator* range_and_bearing_actuator;
	CCI_RangeAndBearingSensor* range_and_bearing_sensor;

	double omega;
	double avoidance_radius;

	double seconds_per_tick;

	double left_motor_speed;
	double right_motor_speed;

	double faulty_left_motor_speed_scaling;
	double faulty_right_motor_speed_scaling;

	bool controller_failure = false;
	bool motor_complete_failure = false;
	bool motor_partial_failure = false;
	bool sensor_complete_failure = false;
	bool sensor_partial_failure = false;
	bool power_failure = false;

	int power_failure_injection_time = 0;

	int random_seed;
	CRandom::CRNG* rng;
};

#endif