#include "robot.h"
#include <iostream>

#define signal_basic 256
#define signal_smart 128
#define signal_gradient 64
#define signal_recruit 32

#define min_movement 40
#define tolerance 50
#define PI 3.14159265358979324

#define radius 20
#define X 0
#define Y 1
#define T 2

class smart_robot : public robot
{
	enum behavior_enum
	{
		finding,
		moving,
		evading,
		recruiting,
		registering,
		finish
	};


	int disks = 3;
	int disks_size[3] = { 2, 2, 0 };
	int disks_center_x[3] = { 400, 500, 10000 };
	int disks_center_y[3] = { 800, 600, 10000 };
	bool disks_completed[3] = { false,false,false };
	int disks_ids[3] = { 0,0,0 };
	int closest_disk = -1;
	behavior_enum behavior = finding; // 1 - identify the closest circle center 2 - moving toward a circle center 3 - recruiting a seed
	double prev_pos[3];
	int steps;
	double destination[2];

	void robot::controller()
	{
		switch(behavior)
		{ 
			case finding:
			{
				findClosestDisk();
				break;
			}
			case moving:
			{
				moveToDestination();
				break;
			}
			case recruiting:
			{
				recruitBasicRobot();
				break;
			}
			case evading: //we are stuck... lets evade
			{
				evadeObstacle();
				break;
			}
			case registering:
			{
				registerSeed();
				break;
			}
		}
		steps++;
	}
	void robot::init()
	{
	}

	bool robot::comm_out_criteria(double x, double y)
	{
		if (x < pos[0] - radius || x > pos[0] + radius || y < pos[1] - radius || y > pos[1] + radius) return false;
		if ( robot::distance(pos[0], pos[1], x, y) > radius) return false; //robot within com range, put transmitting robots data in its data_in struct
		double theta = find_theta(pos[0], pos[1], x, y);
		if (pos[2] > theta - .2 && pos[2] < theta + .2)
		{
			return true;
		}
		return false;
	}
	bool robot::comm_in_criteria(double x, double y) //omnidirectional
	{
		if (gauss_rand(timer) < .90) return true;
		return false;
	}
	
	void findClosestDisk()
	{
		double dist = 0;
		closest_disk = -1;
		for (int i = 0;i < disks;i++)
		{
			if (!disks_completed[i])
			{
				double newdist = distance(disks_center_x[i], disks_center_y[i], pos[X], pos[Y]);
				if (dist > newdist || closest_disk == -1)
				{
					dist = newdist;
					closest_disk = i;
				};
			};
		}
		if (closest_disk == -1) // we're done
		{
			behavior = finish;
			return;
		}
		//let's record our position to know if we are moving
		recordPosition();
		setDestination(disks_center_x[closest_disk], disks_center_y[closest_disk]);
		behavior = moving; // move toward the closest disk
	}

	void setDestination(double x, double y)
	{
		destination[X] = x;
		destination[Y] = y;
	}

	void recruitBasicRobot()
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
				behavior = registering;
				return;
			}
		}

		data_out.id = id;
		data_out.message = signal_smart + signal_recruit + closest_disk;
		if (!(steps % 5))
		{
			tx_request = 1;
		}
	}
	void moveToDestination()
	{
		// find out if we are there yet
		if (distance(pos[X], pos[Y], destination[0], destination[1]) < tolerance)
		{
			//hurray we are there!
			motor_command = 4;
			behavior = recruiting; //recruiting
			return;
		}
		//darn, we are not there
		// let's find if we are pointing in the right direction
		double target_tetha = find_theta(pos[X], pos[Y], destination[0], destination[1]);
		if ((target_tetha - .1 < pos[T]) && (target_tetha + .1 > pos[T]))
		{
			// hurray we are pointing in the right direction!

			//let's see if we are stuck
			if ((steps % 200) == 0)
			{
				if (distance(prev_pos[0], prev_pos[1], pos[0], pos[1]) < min_movement)
				{
					steps = 0;
					behavior = evading; //evade the obstacle
					return;
				}
				//let's record our new position
				prev_pos[0] = pos[0];
				prev_pos[1] = pos[1];
				prev_pos[2] = pos[2];
				steps = 0;
			}
			motor_command = 1;// lets move forward
			return;
		}
		// terrible luck today... let's correct the direction
		steps = 0;
		color[0] = 1;
		color[1] = 0;
		color[2] = 0;
		if (target_tetha > pos[T])
		{
			if (target_tetha < pos[T] + PI)
			{
				motor_command = 2;
			}
			else
			{
				motor_command = 3;
			}
		}
		else {
			if (target_tetha + PI < pos[T])
			{
				motor_command = 2;
			}
			else
			{
				motor_command = 3;
			}
		}
	}

	void registerSeed()
	{
		if (incoming_message_flag)
		{
			incoming_message_flag = 0;
			if (data_in.message & signal_basic && data_in.message & signal_recruit && data_in.message & signal_gradient)
			{
				if (disks_ids[closest_disk] == data_in.id)
				{
					disks_completed[closest_disk] = true;
					behavior = finding;
					return;
				}
			}
		}
		if (steps >= 500)
		{
			behavior = recruiting;
			return;
		}
		data_out.id = id;
		data_out.message = signal_smart + signal_gradient + disks_size[closest_disk];
		if (!(steps % 10))
		{
			tx_request = 1;
		}
	}

	void evadeObstacle()
	{
		if (steps < 100)
		{
			motor_command = 2;
		}
		else if (steps < 200)
		{
			motor_command = 1;
		}
		else if (steps < 300)
		{
			motor_command = 3;
		}
		else if (steps < 400)
		{
			motor_command = 1;
		}
		else {
			behavior = moving;
			recordPosition();
			steps = 0;
		}
	}
	void recordPosition()
	{
		prev_pos[X] = pos[X];
		prev_pos[Y] = pos[Y];
		prev_pos[T] = pos[T];
	}

};