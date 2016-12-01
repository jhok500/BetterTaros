#ifndef FAULT_H
#define FAULT_H

class Fault
{
	
public:

	Fault(int time, std::string robot_id, std::string type) :
		time(time),
		robot_id(robot_id),
		type(type) {}
		
	~Fault() {}

	int time;
	std::string robot_id;
	std::string type;
};

#endif