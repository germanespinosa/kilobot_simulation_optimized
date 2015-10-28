#include "robot.h"

#define signal_basic 256
#define signal_smart 128
#define signal_gradient 64
#define signal_recruit 32

#define tolerance 20

class basic_robot : public robot
{
	int behavior = 1; // 1 - wander 2 - gradient
	
	int steps = 0;
	
	void robot::controller()
	{	
		switch (behavior)
		{
		case 1://wandering
			{
				//if (incoming_message_flag == 1) //i found a brother
				//{
				//	incoming_message_flag = 0;
				//	if (data_in.distance < tolerance) //is he close enough?
				//	{
				//		// he is, let's stop
				//		behavior = 2;
				//		break;
				//	}
				//}
				if (!(steps % 100))
				{
					motor_command = (steps / 100) % 3 + 1;
				}
				color[1] = 1;
				break;
			}	
		case 2:
			{
			}
		}
		steps++;
		return;
		data_out.id = id;//send out my id

						 //if i received a message
		if (incoming_message_flag == 1)
		{
			//clear the message rx flag
			incoming_message_flag = 0;

			//if i have a higher id than my neighbors
			if ((data_in.id)< id)
			{
				//choose motor command based on distance and past distance value
				color[0] = 1;
				if (data_in.distance > 55)
				{

					if (previous_distance>data_in.distance)
					{
						motor_command = 1;
						color[0] = 1;
						color[1] = 0;
						color[2] = 0;
					}
					else
					{
						color[0] = 0;
						color[1] = 1;
						color[2] = 0;
						motor_command = 2;
					}
				}
				else
				{
					if (previous_distance<data_in.distance)
					{
						color[0] = 1;
						color[1] = 0;
						color[2] = 0;
						motor_command = 1;
					}
					else
					{
						color[0] = 0;
						color[1] = 0;
						color[2] = 1;
						motor_command = 3;
					}
				}

			}
			previous_distance = data_in.distance;
		}

		//timer to controll how frequently i tx
		if ((timer % 10) == 0)
		{
			tx_request = 1;
		}
		timer++;
	}
	void robot::init_robot()
	{
	}
};