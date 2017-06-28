#include "robot.h"
class kilobot: public robot 
{
	int light = 0;

	void robot::init()
	{
	}
	void robot::controller()
	{
	}
	double robot::comm_out_criteria(int c, double x, double y, int sd) //stardard circular transmission area
	{
		
	}
	bool robot::comm_in_criteria(int c, double x, double y, double d, void *cd) //omnidirectional
	{
	}
	void robot::sensing(int features, int type[], int x[], int y[], int value[])
	{
	}
	void *robot::get_message(int c)
	{
		return NULL;
	}
	char *robot::get_debug_info(char *buffer, char *rt)
	{
		return NULL;
	}
};