#include "robot.h"
class another_robot: public robot 
{
	int light = 0;

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
	void robot::sensing(int features, int type[], int x[], int y[], int value[])
	{
		for (int i = 0; i < features;i++)
		{
			if (type[i] == sensor_lightsource)
			{
				int thislight = value[i];
				thislight = thislight - distance(pos[0], pos[1], x[i], y[i]) / 2;
				thislight = thislight < 20 ? 20 : thislight;
				double t = find_theta(pos[0], pos[1], x[i], y[i]);
				if (abs(pos[3] - t) < .3)
				{
					thislight = thislight*.1;
				}
				light = light + thislight;
				light = light > 1023 ? 1023 : light;
			}
		}
	}

};