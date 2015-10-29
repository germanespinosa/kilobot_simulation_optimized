#include "robot.h"
#include <iostream>

#define signal_basic 256
#define signal_smart 128
#define signal_gradient 64
#define signal_recruit 32

#define min_movement 40
#define tolerance 20
#define PI 3.14159265358979324

class smart_robot : public robot
{

	int disks = 2;
	int disks_size[2] = { 5, 7 };
	int disks_center_x[2] = { 800, 500 };
	int disks_center_y[2] = { 800, 600 };
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
				double dist = distance(disks_center_x[0],disks_center_y[0],x, y);
				closest_disk = 0;
				for(int i = 1;i < disks;i++)
				{
					double newdist = distance (disks_center_x[i], disks_center_y[i], x,  y);
					if (dist > newdist)
					{
						dist = newdist;
						closest_disk = i;
					};
				}
				//let's record our position to know if we are moving
				prev_pos[0] = pos[0];
				prev_pos[1] = pos[1];
				prev_pos[2] = pos[2];
				steps = 0;
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
					color[0] = 1;
					color[1] = 1;
					color[2] = 1;
					break;
				}
				//let's see if we are stuck
				if ((steps % 300) == 0)
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
				//darn, we are not there
				// let's find if we are pointing in the right direction
				double target_tetha = find_theta(x, y, disks_center_x[closest_disk], disks_center_y[closest_disk]);
				if ((target_tetha - .1 < t) && (target_tetha + .1 > t))
				{
					// hurray we are pointing in the right direction!
					motor_command = 1;// lets move forward
					color[0] = 0;
					color[1] = 1;
					color[2] = 1;
					break;
				}
				// terrible luck today... let's correct the direction
				if (target_tetha > t)
				{
					motor_command = 2;
					color[0] = 1;
					color[1] = 0;
					color[2] = 0;
				}else {
					motor_command = 3;
					color[0] = 1;
					color[1] = 1;
					color[2] = 0;
				}
				break;
			}
			case 3:
			{
				data_out.id = id;
				data_out.message = signal_smart + signal_recruit + disks_size[closest_disk];	
			}
			case 4: //we are stuck... lets evade
			{
				if(steps < 80)
				{
					motor_command = 2;
				} else if (steps < 200) 
				{
					motor_command = 1;
				} else if(steps < 280)
				{
					motor_command = 3;
				} else if (steps < 400)
				{   
					motor_command = 1;
				}
				else { behavior = 2; }
				color[1] = 1;
				color[0] = 0;
				color[0] = 0;
				break;
			}
		}
		steps++;
		timer++;
	}
	void robot::init_robot()
	{
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
		double t = atan(y / x);
		return t;
	}
	bool smart_robot::comm_out_criteria(double x, double y)
	{
		if (robot::comm_out_criteria(x, y))
		{
			double theta = find_theta(pos[0], pos[1], x, y);
			if (theta > pos[2] - .76 || theta < pos[2] + .76) return false;
		}
		return true;
	}
};