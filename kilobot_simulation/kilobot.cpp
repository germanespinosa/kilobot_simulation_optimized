#include "robot.h"
class another_robot: public robot 
{
	void robot::init()
	{
	}
	void robot::controller()
	{
	}
	bool robot::comm_out_criteria(double x, double y) //stardard circular transmission area
	{
		double gaus_comm_range = comm_range + robot::gauss_rand(timer) * comm_range;
		if (x < pos[0] - gaus_comm_range || x > pos[0] + gaus_comm_range || y < pos[1] - gaus_comm_range || y > pos[1] + gaus_comm_range) return false;
		double distance = sqrt((x - pos[1])*(x - pos[0]) + (y - pos[1])*(y - pos[1]));
		return distance < gaus_comm_range; //robot within com range, put transmitting robots data in its data_in struct
	}
	bool robot::comm_in_criteria(double x, double y) //omnidirectional
	{
		if (gauss_rand(timer) < .90) return true;
		return false;
	}

};