#ifndef EPUCK_OMEGA_ALGORITHM_LOOP_FUNCTIONS_H
#define EPUCK_OMEGA_ALGORITHM_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

#include <controllers/epuck_omega_algorithm/fault.h>
#include <controllers/epuck_omega_algorithm/fault_injector.h>

using namespace argos;

class CEPuckOmegaAlgorithmLoopFunctions : public CLoopFunctions
{

public:

	CEPuckOmegaAlgorithmLoopFunctions() {}
	virtual ~CEPuckOmegaAlgorithmLoopFunctions() {}

	virtual void Init(TConfigurationNode& loop_functions_node);
	virtual void Reset();
	virtual void Destroy();
	virtual void PreStep();
	virtual void PostStep();

private:

	int random_seed;
	CRandom::CRNG* rng;

	std::string output_folder;
	FaultInjector fault_injector;
};

#endif