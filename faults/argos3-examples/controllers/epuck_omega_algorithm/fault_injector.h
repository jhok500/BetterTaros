#ifndef FAULT_INJECTOR_H
#define FAULT_INJECTOR_H

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <controllers/epuck_omega_algorithm/fault.h>

using namespace argos;

class FaultInjector
{
public:

	FaultInjector() {}
	~FaultInjector();

	void Init(int random_seed, std::string output_folder);
	void ReadFaultsFromConfigurationFile(TConfigurationNode& configuration_tree);
	void InjectFaults(int current_time, CSpace::TMapPerType& epucks);

private:

	CRandom::CRNG* rng;
	std::vector<Fault*> faults; // Vector of faults to be injected
	std::vector<std::string> faulty_robots;

	std::string faults_filename;
	std::ofstream faults_ofstream;
};

#endif