#include "robot.h"
class another_robot: public robot 
{
	void robot::init()
	{
	}
	void robot::controller()
	{
	}
	bool robot::comm_out_criteria(int c, double x, double y) //stardard circular transmission area
	{
		if (c != 0) return false;
		double gaus_comm_range = comm_range + robot::gauss_rand(timer) * comm_range;
		if (x < pos[0] - gaus_comm_range || x > pos[0] + gaus_comm_range || y < pos[1] - gaus_comm_range || y > pos[1] + gaus_comm_range) return false;
		double distance = sqrt((x - pos[1])*(x - pos[0]) + (y - pos[1])*(y - pos[1]));
		return distance < gaus_comm_range; //robot within com range, put transmitting robots data in its data_in struct
	}
	bool robot::comm_in_criteria(int c, double x, double y) //omnidirectional
	{
		if (c != 0) return false;
		if (gauss_rand(timer) < .90) return true;
		return false;
	}

};