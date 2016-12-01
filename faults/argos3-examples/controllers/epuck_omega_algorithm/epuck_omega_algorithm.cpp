#include "epuck_omega_algorithm.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>

#include <string.h>

static const Real WHEEL_RADIUS = 0.0205f;
static const Real WHEEL_CIRCUMFERENCE = 2 * CRadians::PI.GetValue() * WHEEL_RADIUS;
static const Real INTERWHEEL_DISTANCE = 0.053f;

static const Real INTERWHEEL_CIRCUMFERENCE = INTERWHEEL_DISTANCE * CRadians::PI.GetValue();
static const Real DISTANCE_PER_DEGREE = INTERWHEEL_CIRCUMFERENCE / 360 * 100; // In centimetres

static const Real MAX_SPEED = WHEEL_CIRCUMFERENCE * 100; // 12.88 cm/s

/////////////////////
// Omega algorithm //
/////////////////////

CEPuckOmegaAlgorithm::CEPuckOmegaAlgorithm()
{
	wheels = NULL;
	leds = NULL;
	state = forward;
	aggregation_timer = 0;
	distance_turned = 0;
}

void CEPuckOmegaAlgorithm::Init(TConfigurationNode& t_node)
{
	// Set up sensor and actuator objects
	wheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
	leds = GetActuator<CCI_LEDsActuator>("leds");
	range_and_bearing_actuator = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
	range_and_bearing_sensor = GetSensor <CCI_RangeAndBearingSensor>("range_and_bearing");

	// Get parameter values from the XML configuration file
	try
	{
		GetNodeAttribute(t_node, "omega", omega);
		GetNodeAttribute(t_node, "avoidance_radius", avoidance_radius);
		GetNodeAttribute(t_node, "draw_id", draw_id);
		GetNodeAttribute(t_node, "draw_sensor_range", draw_sensor_range);
	}
	catch(CARGoSException& ex)
	{
		THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
	}

	// Convert omega from seconds to ticks
	seconds_per_tick = CPhysicsEngine::GetSimulationClockTick();
	omega /= seconds_per_tick;

	// Get the observing robot's name
	std::string robot_name = GetId();

	// Cut off the 'ep' at the start, leaving just the number
	robot_name = robot_name.substr(2, robot_name.size());

	// Convert to number (as string) to integer ID
    int id = atoi(robot_name.c_str()); 

    // Send observing robot's ID via the RAB actuator
	range_and_bearing_actuator->SetData(0, id);

	try
	{
		TConfigurationNode& t_tree = CSimulator::GetInstance().GetConfigurationRoot();

		/////////////////////
		// Get random seed //
		/////////////////////

		TConfigurationNode framework_node = GetNode(t_tree, "framework");
		TConfigurationNode experiment_node = GetNode(framework_node, "experiment");

		GetNodeAttribute(experiment_node, "random_seed", random_seed);
	}
	catch(CARGoSException& ex)
	{
		THROW_ARGOSEXCEPTION_NESTED("Error parsing loop function parameters.", ex);
	}

	rng = CRandom::CreateRNG("argos");
}

void CEPuckOmegaAlgorithm::Destroy()
{

}

void CEPuckOmegaAlgorithm::ControlStep()
{
	// Get RAB packets from other robots within range
	CCI_RangeAndBearingSensor::TReadings packets = GetRABSensorReadings();

	OmegaAlgorithm(packets);
}

void CEPuckOmegaAlgorithm::OmegaAlgorithm(CCI_RangeAndBearingSensor::TReadings packets)
{
	int current_time = CSimulator::GetInstance().GetSpace().GetSimulationClock();

	if(controller_failure)
	{
		// At each tick, there is some chance that the controller state will be randomised
		if(rng->Uniform(CRange<Real>(0, 1)) <= 0.1)
		{
			int random_state = rng->Uniform(CRange<UInt32>(0, 3));

			switch(random_state)
			{
				case 0:
					state = forward;
					break;
				case 1:
					state = avoidance;
					break;
				case 2:
					state = coherence;
					break;
			}
		}
	}

	// Omega algorithm state machine
	if(state == forward)
	{
		// Bjerknes programmed the e-pucks to move at a quarter of their maximum speed
		left_motor_speed = right_motor_speed = MAX_SPEED / 4;

		// Perform coherence if the aggregation timer has expired
		if(current_time - aggregation_timer > omega)
		{
			if(packets.size() == 0)
				heading.SetValue(180);
			else
			{
				CVector2 centroid;

				for(int i = 0; i < packets.size(); ++i)
				{
					const CCI_RangeAndBearingSensor::SPacket packet = packets[i];

					Real range = packet.Range;
					CRadians bearing = packet.HorizontalBearing;

					// No need to ever divide by the number of robots - we only care about the angle, not the actual coordinates
					centroid += CVector2(range, bearing);
				}

				// Calculate the angle towards the swarm centroid
				heading = ToDegrees(centroid.Angle());
			}

			state = coherence;
		}
		else if(current_time - aggregation_timer > 5) // Otherwise, check whether avoidance is required (with some cool-off period since the last turn)
		{
			CVector2 centroid;

			for(int i = 0; i < packets.size(); ++i)
			{
				const CCI_RangeAndBearingSensor::SPacket packet = packets[i];

				Real range = packet.Range / 100; // Convert centimetres to metres

				// Only avoid robots within our avoidance radius
				if(range < avoidance_radius)
				{
					CRadians bearing = packet.HorizontalBearing;

					centroid += CVector2(range, bearing);
				}
			}

			// Only avoid if there were robots close enough
			if(centroid.Length() != 0)
			{
				// Turn 180 degrees away from the robots being avoided
				centroid.Rotate(ToRadians(CDegrees(180)));

				heading = ToDegrees(centroid.Angle());

				state = avoidance;
			}
		}
	}
	else if(state == avoidance || state == coherence)
	{
		// Either turn left or right, depending on the desired heading
		if(heading.GetValue() <= 0)
		{
			left_motor_speed = MAX_SPEED / 4;
			right_motor_speed = -MAX_SPEED / 4;
		}
		else
		{
			left_motor_speed = -MAX_SPEED / 4;
			right_motor_speed = MAX_SPEED / 4;
		}

		// Use the previous desired left motor speed to calculate how far the robot has turned
		distance_turned += std::abs(left_motor_speed * seconds_per_tick);

		// Stop turning once the desired heading has been reached
		if(distance_turned >= std::abs(DISTANCE_PER_DEGREE * heading.GetValue()))
		{
			distance_turned = 0;
			aggregation_timer = current_time;
			state = forward;
		}
	}

	// Set the robot's wheel speeds
	SetMotorSpeeds(left_motor_speed, right_motor_speed);
}

void CEPuckOmegaAlgorithm::InjectFault(std::string type)
{
	if(type == "motor_complete")
		motor_complete_failure = true;
	else if(type == "motor_partial")
	{
		motor_partial_failure = true;
		SetFaultyMotorSpeedScaling(1, 0.5);	
	}
	else if(type == "controller")
		controller_failure = true;
	else if(type == "sensor_complete")
		sensor_complete_failure = true;
	else if(type == "sensor_partial")
		sensor_partial_failure = true;
	else if(type == "communication")
		communication_failure = true;
	else if(type == "power")
		power_failure = true;
}

void CEPuckOmegaAlgorithm::SetFaultyMotorSpeedScaling(double left, double right)
{
	faulty_left_motor_speed_scaling = left;
	faulty_right_motor_speed_scaling = right;
}

bool CEPuckOmegaAlgorithm::IsFaulty()
{
	return controller_failure ||
		   motor_complete_failure ||
		   motor_partial_failure ||
		   sensor_complete_failure ||
		   sensor_partial_failure ||
		   communication_failure ||
		   power_failure;
}

void CEPuckOmegaAlgorithm::SetMotorSpeeds(double left, double right)
{
	int current_time = CSimulator::GetInstance().GetSpace().GetSimulationClock();

	if(motor_complete_failure)
		wheels->SetLinearVelocity(0, 0);
	else if(motor_partial_failure)
		wheels->SetLinearVelocity(left * faulty_left_motor_speed_scaling, right * faulty_right_motor_speed_scaling);
	else
		wheels->SetLinearVelocity(left, right);

	if(power_failure)
	{
		if(current_time - power_failure_injection_time <= 2 / seconds_per_tick)
			wheels->SetLinearVelocity(0, 0);
		else if(rng->Uniform(CRange<Real>(0, 1)) <= 0.05)
			power_failure_injection_time = current_time;
	}
}

CCI_RangeAndBearingSensor::TReadings CEPuckOmegaAlgorithm::GetRABSensorReadings()
{
	// Get RAB packets from other robots within range
	const CCI_RangeAndBearingSensor::TReadings& packets = range_and_bearing_sensor->GetReadings();

	CCI_RangeAndBearingSensor::TReadings faulty_packets;

	if(sensor_complete_failure)
		return faulty_packets; // Return empty vector
	else if(sensor_partial_failure)
	{
		for(CCI_RangeAndBearingSensor::SPacket packet : packets)
		{
			double bearing = ToDegrees(packet.HorizontalBearing).GetValue();

			if(bearing >= -45 && bearing <= 45)
				continue;
			
			faulty_packets.push_back(packet);
		}

		return faulty_packets;
	}
	else
		return packets;
}

REGISTER_CONTROLLER(CEPuckOmegaAlgorithm, "epuck_omega_algorithm_controller")