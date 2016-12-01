#include "epuck_omega_algorithm_loop_functions.h"

#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <controllers/epuck_omega_algorithm/epuck_omega_algorithm.h>
#include <argos3/plugins/simulator/media/rab_medium.h>

void CEPuckOmegaAlgorithmLoopFunctions::Init(TConfigurationNode& loop_functions_node)
{
	// Read parameter values from the ARGoS XML configuration file
	try
	{
		// Get random seed
		TConfigurationNode& configuration_tree = CSimulator::GetInstance().GetConfigurationRoot();
		TConfigurationNode& framework_node = GetNode(configuration_tree, "framework");
		TConfigurationNode& experiment_node = GetNode(framework_node, "experiment");

		GetNodeAttribute(experiment_node, "random_seed", random_seed);
	}
	catch(CARGoSException& ex)
	{
		THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
	}

	// Read parameter values from the ARGoS XML configuration file
	try
	{
		TConfigurationNode& simulation_parameters = GetNode(loop_functions_node, "simulation_parameters");
		GetNodeAttribute(simulation_parameters, "output_folder", output_folder);
		TConfigurationNode& configuration_tree = CSimulator::GetInstance().GetConfigurationRoot();

		// Read faults from configuration file
		fault_injector.Init(random_seed, output_folder);
		fault_injector.ReadFaultsFromConfigurationFile(configuration_tree);
	}
	catch(CARGoSException& ex)
	{
		THROW_ARGOSEXCEPTION_NESTED("Error parsing external simulation parameters", ex);
	}

	rng = CRandom::CreateRNG("argos");
}

void CEPuckOmegaAlgorithmLoopFunctions::Reset()
{
}

void CEPuckOmegaAlgorithmLoopFunctions::PreStep()
{
	int current_time = GetSpace().GetSimulationClock();
	CSpace::TMapPerType& epucks = GetSpace().GetEntitiesByType("e-puck");

	// Inject faults, if any should be present at this tick
	fault_injector.InjectFaults(current_time, epucks);
}

void CEPuckOmegaAlgorithmLoopFunctions::PostStep()
{

}

void CEPuckOmegaAlgorithmLoopFunctions::Destroy()
{

}

REGISTER_LOOP_FUNCTIONS(CEPuckOmegaAlgorithmLoopFunctions, "epuck_omega_algorithm_loop_functions")