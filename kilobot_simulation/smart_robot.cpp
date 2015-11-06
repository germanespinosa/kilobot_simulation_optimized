#include "robot.h"
#include <iostream>

#define signal_basic 256
#define signal_smart 128
#define signal_gradient 64
#define signal_recruit 32

#define min_movement 40
#define tolerance 50
#define PI 3.14159265358979324

class smart_robot : public robot
{

	int disks = 3;
	int disks_size[3] = { 2, 2, 0 };
	int disks_center_x[3] = { 400, 500, 10000 };
	int disks_center_y[3] = { 800, 600, 10000 };
	bool disks_completed[3] = { false,false,false };
	int disks_ids[3] = { 0,0,0 };
	int closest_disk = -1;
	//smart robots turn in place, walk straight, have gps and compass 
	//smart robot variables
	int behavior = 1; // 1 - identify the closest circle center 2 - moving toward a circle center 3 - recruiting a seed
	double prev_pos[3];
	int steps;
	void robot::controller()
	{
		double x = pos[0];
		double y = pos[1];
		double t = pos[2];
		switch(behavior)
		{ 
			case 1:
			{
				double dist = 0;
				closest_disk = -1;
				for(int i = 0;i < disks;i++)
				{
					if (!disks_completed[i])
					{
						double newdist = distance(disks_center_x[i], disks_center_y[i], x, y);
						if (dist > newdist || closest_disk==-1)
						{
							dist = newdist;
							closest_disk = i;
						};
					};
				}
				if (closest_disk == -1) // we're done
				{
					behavior = 5;
					break;
				}
				//let's record our position to know if we are moving
				prev_pos[0] = pos[0];
				prev_pos[1] = pos[1];
				prev_pos[2] = pos[2];
				steps = 1;
				behavior = 2; // move toward the closest disk
			}
			case 2:
			{
				// find out if we are there yet
				if (distance(x, y, disks_center_x[closest_disk], disks_center_y[closest_disk]) < tolerance)
				{
					//hurray we are there!
					motor_command = 4;
					behavior = 3; //recruiting
					break;
				}
				//darn, we are not there
				// let's find if we are pointing in the right direction
				double target_tetha = find_theta(x, y, disks_center_x[closest_disk], disks_center_y[closest_disk]);
				if ((target_tetha - .1 < t) && (target_tetha + .1 > t))
				{
					// hurray we are pointing in the right direction!

					//let's see if we are stuck
					if ((steps % 200) == 0)
					{
						if (distance(prev_pos[0], prev_pos[1], pos[0], pos[1]) < min_movement)
						{
							steps = 0;
							behavior = 4; //evade the obstacle
							break;
						}
						//let's record our new position
						prev_pos[0] = pos[0];
						prev_pos[1] = pos[1];
						prev_pos[2] = pos[2];
						steps = 0;
					}
					motor_command = 1;// lets move forward
					break;
				}
				// terrible luck today... let's correct the direction
				steps = 0;
				color[0] = 1;
				color[1] = 0;
				color[2] = 0;
				if (target_tetha > t)
				{
					if (target_tetha < t + PI)
					{
						motor_command = 2;
					}
					else
					{
						motor_command = 3;
					}
				}else {
					if (target_tetha + PI < t )
					{
						motor_command = 2;
					}
					else
					{
						motor_command = 3;
					}
				}
				break;
			}
			case 3:
			{
				color[0] = 0;
				color[1] = 0;
				color[2] = 1;
				//let's check if somebody received the message, we have a possible seed
				if (incoming_message_flag)
				{
					incoming_message_flag = 0;
					if (data_in.message == signal_basic + signal_recruit)
					{
						disks_ids[closest_disk] = data_in.id;
						steps = 0;
						behavior = 7;
						break;
					}
				}

				data_out.id = id;
				data_out.message = signal_smart + signal_recruit + closest_disk;
				if (!(steps % 5))
				{
					tx_request = 1;
				}
				break;
			}
			case 4: //we are stuck... lets evade
			{
				if(steps < 100)
				{
					motor_command = 2;
				} else if (steps < 200) 
				{
					motor_command = 1;
				} else if(steps < 300)
				{
					motor_command = 3;
				} else if (steps < 400)
				{   
					motor_command = 1;
				}
				else { 
					behavior = 2; 
					prev_pos[0] = pos[0];
					prev_pos[1] = pos[1];
					prev_pos[2] = pos[2];
					steps = 0;
				}
				break;
			}
			case 7:
			{
				if (incoming_message_flag)
				{
					incoming_message_flag = 0;
					if (data_in.message & signal_basic && data_in.message & signal_recruit && data_in.message & signal_gradient)
					{
						if (disks_ids[closest_disk] == data_in.id)
						{
							disks_completed[closest_disk] = true;
							behavior = 1;
							break;
						}
					}
				}
				if (steps >= 500)
				{
					behavior = 2;
					break;
				}
				data_out.id = id;
				data_out.message = signal_smart + signal_gradient + disks_size[closest_disk];
				if (!(steps % 10))
				{
					tx_request = 1;
				}
				break;
			}
		}
		steps++;
		timer++;
	}
	void robot::init_robot()
	{
		comm_range = 60;
	}

	double distance(int x1, int y1, int x2, int y2)
	{
		double x = x1 - x2;
		double y = y1 - y2;
		double s = pow(x,2) + pow(y, 2);
		return sqrt(s);
	}
	double find_theta(int x1, int y1, int x2, int y2)
	{
		if (x1 == x2) return 0;
		double x = x2 - x1; 
		double y = y2 - y1;
		
		if (x >= 0 && y >= 0)
		{
			return atan(y / x);
		}
		if (x < 0 && y < 0)
		{
			return atan(y / x) + PI;
		}
		if (x < 0 && y > 0)
		{
			return atan(abs(x) / y) + PI/2;
		}
		return atan(x / abs(y)) + PI / 2 * 3;
	}
	bool smart_robot::comm_out_criteria(double x, double y)
	{
		double theta=0;
		if (robot::comm_out_criteria(x, y))
		{
			theta = find_theta(pos[0], pos[1], x, y);
			if (pos[2] > theta - .2 && pos[2] < theta + .2)
			{
				return true;
			}
		}
		return false;
	}
};