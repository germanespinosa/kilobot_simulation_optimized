#include "robot.h"

#define signal_basic 65536
#define signal_smart 32768
#define signal_gradient 16384
#define signal_recruit 8192
#define signal_claim 4096
#define mask_size 31
#define mask_delay 992


class basic_robot : public robot
{
	int changecolor = 0;
	int behavior = 1; // 1 - wander 2 - gradient
	int disk_id = 0;
	int disk_size = 0;
	int steps = 0;
	int final_disk_size = 0;
	bool seed=false;
	int turning_direction = 0;
	int turning_duration = 0;
	int wait = 0;

	void robot::controller()
	{	
		if (incoming_message_flag == 1)
		{
			changecolor = 5;
		}
		switch (behavior)
		{
		case 1://wandering
			{
				if (incoming_message_flag == 1) //i found a brother
				{
					incoming_message_flag = 0;
					// let's see if he wants to recruit me
					if (data_in.message & signal_smart && data_in.message & signal_recruit)
					{
						//i'll be a seed! how exiting!
						motor_command = 4;
						seed = true;
						final_disk_size = data_in.message & mask_size;
						wait = (data_in.message & mask_delay) >> 5;
						disk_size = 0;
						behavior = 2;
						steps = 0;
						break;
					}
					if (data_in.message & signal_basic && data_in.message & signal_recruit && data_in.message & signal_gradient)
					{
						motor_command = 4;
						disk_size = (data_in.message & 31)-1;
						behavior = 3;
						break;
					}
				}
				if (steps < turning_duration)
				{
					motor_command = turning_direction;
				}
				else if (steps < 11 * turning_duration)
				{
					motor_command = 1;
				}
				else 
				{
					randomize_behavior();
				}
				color[0] = 0;
				color[1] = 1;
				color[2] = 0;
				break;
			}	
		case 2:
			{
				if (steps >= (1000 * wait)+10)
				{
					behavior = 3;
				}
				data_out.id = id;
				data_out.message = signal_basic + signal_gradient;
				color[0] = 1;
				color[1] = 0;
				color[2] = 1;
				tx_request = 1;
				break;
			}
		case 3:
			{
				motor_command = 4;
				if (incoming_message_flag)
				{
					incoming_message_flag = 0;
					if (!seed && data_in.message & signal_basic && data_in.message & signal_recruit && data_in.message & signal_gradient)
					{
						if (disk_size < (data_in.message & 31)-1)
						{
							disk_size = (data_in.message & 31)-1;
							behavior = 3;
							color[0] = 1;
							color[1] = 0;
							color[2] = 1;
						}
					}
				}
				if (seed && !(steps % 2000) && final_disk_size>disk_size)
				{
						disk_size++;
				}
				if (disk_size > 0)
				{
					data_out.id = id;
					data_out.message = signal_basic + signal_recruit + signal_gradient + disk_size;
					color[0] = 1;
					color[1] = 1;
					color[2] = 1;
					tx_request = 1;
				}else{
					color[0] = 0;
					color[1] = 1;
					color[2] = 1;
				}
				break;
			}
		}
		//if (changecolor)
		//{
		//	color[0] = 1;
		//	color[1] = 0;
		//	color[2] = 0;
		//	changecolor--;
		//}
		steps++;
	}
	void randomize_behavior()
	{
		steps = 0;
		turning_direction = 2 + rand() * 2 / RAND_MAX;
		turning_duration = 10 + rand() * 50 / RAND_MAX;
	}
	void robot::init()
	{
		randomize_behavior();
		comm_range = 60;
	}
	double robot::comm_out_criteria(int c, double x, double y, int sd) //stardard circular transmission area
	{
		if (c != 1 || sd) return 0;
		static double diameter = 2 * radius + 5;
		if (x < pos[0] - diameter || x > pos[0] + diameter || y < pos[1] - diameter || y > pos[1] + diameter) return 0;
		double dist = robot::distance(pos[0], pos[1], x, y);
		if (dist <= diameter) return dist; //robot within com range, put transmitting robots data in its data_in struct
		return 0;
	}
	bool robot::comm_in_criteria(int c, double x, double y, double d, void *cd) //omnidirectional
	{
		if (c != 1) return false;
		communcation_data *ccd = (communcation_data *)cd;
		data_in = *ccd;
		data_in.distance;
		return true;
	}
	void robot::sensing(int features, int type[], int x[], int y[], int value[])
	{
	}

};