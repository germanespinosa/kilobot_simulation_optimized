#include "robot.h"
#include "touch_channel.h"


class basic_robot : public robot
{
	int changecolor = 0;
	int behavior = 1; // 1 - wander 2 - gradient
	int disk_id = 0;
	int disk_size = 0;
	int steps = 0;
	int final_disk_size = 0;
	bool seed = false;
	int turning_direction = 0;
	int turning_duration = 0;
	int wait = 0;

	//comms
	//received data goes here
	touch_data data_in;
	//data to transmitt goes here
	touch_data data_out;


	void robot::controller()
	{
		if (incoming_message_flag & touch)
		{
			changecolor = 5;
		}
		switch (behavior)
		{
		case 1://wandering
		{
			if (incoming_message_flag & touch) //i found a brother
			{
				incoming_message_flag = incoming_message_flag & !touch;
				// let's see if he wants to recruit me
				if (data_in.action==touch_action::recruit_seed)
				{
					//i'll be a seed! how exiting!
					motor_command = 4;
					seed = true;
					final_disk_size = data_in.data1;
					wait = data_in.int_data;
					disk_id = data_in.data2;
					disk_size = 0;
					behavior = 2;
					steps = 0;
					break;
				}
				if (data_in.action==touch_action::recruit)
				{
					motor_command = 4;
					disk_size = data_in.data1-1;
					disk_id = data_in.data2;
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
			if (steps >= wait + 10)
			{
				behavior = 3;
			}
			data_out.id = id;
			data_out.action=touch_action::accepted;
			color[0] = 1;
			color[1] = 0;
			color[2] = 1;
			if (rand() <RAND_MAX * .2)
				tx_request = tx_request | touch;
			break;
		}
		case 3:
		{
			motor_command = 4;
			if (incoming_message_flag)
			{
				incoming_message_flag = 0;
				if (!seed && data_in.action==touch_action::recruit)
				{
					if (disk_size < data_in.data1 - 1)
					{
						disk_size = data_in.data1 - 1;
						disk_id = data_in.data2;
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
				data_out.action = touch_action::recruit;
				data_out.data1= disk_size;
				data_out.data2 = disk_id;
				color[0] = 1;
				color[1] = 1;
				color[2] = 1;
				if (rand() <RAND_MAX * .2)
					tx_request = tx_request | touch;
			}
			else {
				data_out.id = id;
				data_out.action = touch_action::border;
				data_out.data1 = disk_size;
				data_out.data2 = disk_id;
				color[0] = 0;
				color[1] = 1;
				color[2] = 1;
				if (rand() <RAND_MAX * .2)
					tx_request = tx_request | touch;
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
		dest[0] = -1;
		dest[1] = -1;
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
		touch_data *ccd = (touch_data *)cd;
		data_in = *ccd;
		data_in.distance;
		return true;
	}
	void robot::sensing(int features, int type[], int x[], int y[], int value[])
	{
	}
	void *robot::get_message(int c)
	{
		return (void *)&data_out;
	}
	char *robot::get_debug_info()
	{
		return "test\n";
	}

};