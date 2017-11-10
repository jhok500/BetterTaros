#include "epuck_omega_algorithm.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>

static const Real WHEEL_RADIUS = 0.0205f;
static const Real WHEEL_CIRCUMFERENCE = 2 * CRadians::PI.GetValue() * WHEEL_RADIUS;
static const Real INTERWHEEL_DISTANCE = 0.053f;

static const Real INTERWHEEL_CIRCUMFERENCE = INTERWHEEL_DISTANCE * CRadians::PI.GetValue();
static const Real DISTANCE_PER_DEGREE = INTERWHEEL_CIRCUMFERENCE / 360 * 100; // In centimetres

static const Real MAX_SPEED = WHEEL_CIRCUMFERENCE * 100; // 12.88 cm/s

CEPuckOmegaAlgorithm::CEPuckOmegaAlgorithm():
        leds(NULL),
        light_sensor(NULL),
        wheels(NULL),
        state(forward),
        aggregation_timer(0),
        distance_turned(0) {};

void CEPuckOmegaAlgorithm::Init(TConfigurationNode& t_node)
{
	// Set up sensor and actuator objects
    leds = GetActuator<CCI_LEDsActuator>("leds");
    light_sensor = GetSensor<CCI_LightSensor>("light");
	wheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
	range_and_bearing_actuator = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
	range_and_bearing_sensor = GetSensor <CCI_RangeAndBearingSensor>("range_and_bearing");

	// Get parameter values from the XML configuration file
	try
	{
		GetNodeAttribute(t_node, "omega", omega);
        GetNodeAttribute(t_node, "shadowed_avoidance_radius", shadowed_avoidance_radius);
        GetNodeAttribute(t_node, "illuminated_avoidance_radius", illuminated_avoidance_radius);
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
}

void CEPuckOmegaAlgorithm::ControlStep()
{
    int current_time = CSimulator::GetInstance().GetSpace().GetSimulationClock();

    // Determine illumination status
    const std::vector<Real>& light_sensor_readings = light_sensor->GetReadings();

    bool illuminated = false;
    int count = 0;

    for(Real reading : light_sensor_readings)
    {
        if(reading > 0)
        {
            count++;

            // Robot is only 'illuminated' if 3 or more of the light sensors can see the beacon
            if(count >= 3)
            {
                illuminated = true;
                break;
            }
        }
    }

    // Set the robot's avoidance radius according to its illumination status
    // The LEDs also change based on illumination status, but only for visual debugging purposes
    if(illuminated)
    {
        leds->SetAllColors(CColor::YELLOW);
        avoidance_radius = illuminated_avoidance_radius;
    }
    else
    {
        leds->SetAllColors(CColor::BLACK);
        avoidance_radius = shadowed_avoidance_radius;
    }

    // Get RAB packets from other robots within range
    const CCI_RangeAndBearingSensor::TReadings& packets = range_and_bearing_sensor->GetReadings();

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
    wheels->SetLinearVelocity(left_motor_speed, right_motor_speed);
}

REGISTER_CONTROLLER(CEPuckOmegaAlgorithm, "epuck_omega_algorithm_controller")