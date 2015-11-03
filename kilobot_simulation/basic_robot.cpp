#include "robot.h"

#define signal_basic 256
#define signal_smart 128
#define signal_gradient 64
#define signal_recruit 32

#define tolerance 60

class basic_robot : public robot
{
	int behavior = 1; // 1 - wander 2 - gradient
	int disk_id = 0;
	int disk_size = 0;
	int steps = 0;
	
	void robot::controller()
	{	
		switch (behavior)
		{
		case 1://wandering
			{
				if (incoming_message_flag == 1) //i found a brother
				{
					incoming_message_flag = 0;
					if (data_in.distance < tolerance) //is he close enough?
					{
						// he is, let's see if he wants to recruit me
						if (data_in.message & signal_smart && data_in.message & signal_recruit)
						{
							//i'll be a seed! how exiting!
							disk_id = data_in.message & 31;
							motor_command = 4;
							behavior = 2;
							steps = 0;
							break;
						}
						if (data_in.message & signal_basic && data_in.message & signal_recruit && data_in.message & signal_gradient)
						{
							disk_size = (data_in.message & 31)-1;
							behavior = 3;
							break;
						}
					}
				}
				
				if (steps < 100)
				{
					motor_command = 2;
				}
				else if (steps < 300)
				{
					motor_command = 1;
				}
				else if (steps < 400)
				{
					motor_command = 3;
				}
				else if (steps < 600)
				{
					motor_command = 1;
				}
				else {
					steps = 0;
				}
				color[0] = 0;
				color[1] = 1;
				color[2] = 0;
				break;
			}	
		case 2:
			{
				if (incoming_message_flag)
				{
					incoming_message_flag = 0;
					if (data_in.message & signal_smart && data_in.message & signal_gradient)
					{
						disk_size = data_in.message & 31;
						behavior = 3;
						color[0] = 1;
						color[1] = 1;
						color[2] = 1;
					}
				}
				if (steps >= 500)
				{
					//looks like we lost touch with the smart robot... let's go back to wander mode
					behavior = 1;
					break;
				}
				color[0] = 1;
				color[1] = 1;
				color[2] = 0;
				data_out.id = id;
				data_out.message = signal_basic + signal_recruit;
				if (timer % 10)
				{
					tx_request = 1;
				}
				break;
			}
		case 3:
			{
				motor_command = 4;
				if (incoming_message_flag)
				{
					incoming_message_flag = 0;
					if (data_in.distance < tolerance)
					{
						if (data_in.message & signal_basic && data_in.message & signal_recruit && data_in.message & signal_gradient)
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
						if (data_in.message & signal_smart && data_in.message & signal_gradient)
						{
							{
								disk_size = data_in.message & 31;
								color[0] = 1;
								color[1] = 1;
								color[2] = 1;
							}
						}
					}
				}

				if (disk_size > 0)
				{
					data_out.id = id;
					data_out.message = signal_basic + signal_recruit + signal_gradient + disk_size;
					color[0] = 1;
					color[1] = 1;
					color[2] = 1;
					if (timer % 10)
					{
						color[0] = 1;
						color[1] = 0;
						color[2] = 0;
						tx_request = 1;
					}
				}else{
					color[0] = 0;
					color[1] = 1;
					color[2] = 1;
				}
			}
		}
		steps++;
		timer++;
	}
	void robot::init_robot()
	{
		comm_range = 60;
	}
};