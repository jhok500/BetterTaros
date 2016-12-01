#include "fault_injector.h"

#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <controllers/epuck_omega_algorithm/epuck_omega_algorithm.h>

void FaultInjector::Init(int random_seed, std::string output_folder)
{
	rng = CRandom::CreateRNG("argos");

	std::ostringstream oss;

	oss << "output/" << output_folder << "/faults_" << random_seed << ".csv";
	faults_filename = oss.str();
	faults_ofstream.open(faults_filename, std::ios_base::trunc | std::ios_base::out); // Open the file for writing, erasing its contents
}

FaultInjector::~FaultInjector()
{
	for(Fault* fault : faults)
		delete fault;

	faults_ofstream.close();
}

void FaultInjector::ReadFaultsFromConfigurationFile(TConfigurationNode& configuration_tree)
{
	if(NodeExists(configuration_tree, "faults"))
	{
		TConfigurationNode& faults_node = GetNode(configuration_tree, "faults");

		TConfigurationNodeIterator fault_iter;

		for(fault_iter = fault_iter.begin(&faults_node); fault_iter != fault_iter.end(); ++fault_iter)
		{
			int injection_time;
			std::string robot_id;

			GetNodeAttribute(*fault_iter, "time", injection_time);
			GetNodeAttribute(*fault_iter, "id", robot_id);

			std::string fault_type = fault_iter->Value();

			// Create new Fault object on the heap, and store a pointer to it
			Fault* fault = new Fault(injection_time, robot_id, fault_type);

			faults.push_back(fault);
		}
	}
}

void FaultInjector::InjectFaults(int current_time, CSpace::TMapPerType& epucks)
{
	// Can't delete faults as we iterate over them, so store pointers to the ones to be deleted in a vector
	std::vector<Fault*> to_delete;

	// Iterate over the faults read from the configuration file
	for(Fault* fault : faults)
	{
		if(current_time >= fault->time)
		{
			if(fault->robot_id == "random")
			{
				bool already_faulty = false;
				std::ostringstream oss;

				do
				{
					int random_id = rng->Uniform(CRange<UInt32>(0, epucks.size()));

					oss.str(std::string()); // Clear stringstream
					oss << "ep" << random_id;

					already_faulty = std::find(faulty_robots.begin(), faulty_robots.end(), oss.str()) != faulty_robots.end();

				} while(already_faulty);

				fault->robot_id = oss.str();
			}

			if(fault->type == "random")
			{
				std::vector<std::string> fault_types;

				fault_types.push_back("motor_complete");
				fault_types.push_back("motor_partial");
				fault_types.push_back("controller");
				fault_types.push_back("sensor_complete");
				fault_types.push_back("sensor_partial");
				fault_types.push_back("communication");
				fault_types.push_back("power");

				int random_type = rng->Uniform(CRange<UInt32>(0, fault_types.size()));

				fault->type = fault_types.at(random_type);
			}

			for(auto& map_element : epucks)
			{
				CEPuckEntity& epuck = *any_cast<CEPuckEntity*>(map_element.second);
				
				if(epuck.GetId() == fault->robot_id)
				{
					CEPuckOmegaAlgorithm& controller = dynamic_cast<CEPuckOmegaAlgorithm&>(epuck.GetControllableEntity().GetController());

					controller.InjectFault(fault->type);

					to_delete.push_back(fault);
					faulty_robots.push_back(fault->robot_id);
				}
			}
		}
	}

	// Once a fault has been injected, delete the object and the pointer to it in the faults vector
	for(Fault* fault : to_delete)
	{
		// Log fault details to file before deletion
		faults_ofstream << fault->time << ",";
		faults_ofstream << fault->robot_id << ",";
		faults_ofstream << fault->type << std::endl;
		faults_ofstream.flush();

		delete(fault);
		faults.erase(std::remove(faults.begin(), faults.end(), fault), faults.end());
	}
}