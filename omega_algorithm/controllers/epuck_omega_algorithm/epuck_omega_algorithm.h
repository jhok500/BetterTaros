#ifndef EPUCK_OMEGA_ALGORITHM_H
#define EPUCK_OMEGA_ALGORITHM_H

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_light_sensor.h>

using namespace argos;

class CEPuckOmegaAlgorithm : public CCI_Controller
{

public:

	CEPuckOmegaAlgorithm();
	virtual ~CEPuckOmegaAlgorithm() {}

	virtual void Init(TConfigurationNode& t_node);
	virtual void ControlStep();
	virtual void Reset() {}
	virtual void Destroy() {};

private:

	CCI_LEDsActuator* leds;
    CCI_LightSensor* light_sensor;
	CCI_DifferentialSteeringActuator* wheels;
	CCI_RangeAndBearingActuator* range_and_bearing_actuator;
	CCI_RangeAndBearingSensor* range_and_bearing_sensor;

	enum State {forward, avoidance, coherence};
	State state;

	double omega;
    double avoidance_radius;
    double shadowed_avoidance_radius;
	double illuminated_avoidance_radius;
	int aggregation_timer;
	double distance_turned;
	CDegrees heading;

	double left_motor_speed;
	double right_motor_speed;

    double seconds_per_tick;
};

#endif