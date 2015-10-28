#include "robot.h"
#include <iostream>

#define signal_basic 256
#define signal_smart 128
#define signal_gradient 64
#define signal_recruit 32

#define tolerance 20
#define PI 3.14159265358979324

class smart_robot : public robot
{

	int disks = 1;
	int disks_size[2] = { 5, 7 };
	int disks_center_x[2] = { 800, 2500 };
	int disks_center_y[2] = { 800, 3600 };
	int closest_disk = -1;
	//smart robots turn in place, walk straight, have gps and compass 
	//smart robot variables
	int behavior = 1; // 1 - identify the closest circle center 2 - moving toward a circle center 3 - recruiting a seed
		
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
		}
		if ((timer % 10) == 0)
		{
			tx_request = 1;
		}
		timer++;
	}
	void robot::init_robot()
	{
		motor_error = 0; //yes I'm cheating and I like it!
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
};