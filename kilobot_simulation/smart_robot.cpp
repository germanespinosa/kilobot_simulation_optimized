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

	int disks = 12;
	int disks_size[12] = { 10,10,10,10,10,10,10,10,10,10,10,10 };
	int disks_center_x[12] = { 1800, 1800, 1800, 1800, 2100, 2600, 3000, 3400, 3700, 3700, 3700, 3700 };
	int disks_center_y[12] = { 1800, 2200, 2600, 3200, 3500, 3500, 3500, 3500, 3200, 2600, 2200, 1800 };
	bool disks_completed[12];
	int disks_ids[12];


	int closest_disk = -1;
	behavior_enum behavior = finding; // 1 - identify the closest circle center 2 - moving toward a circle center 3 - recruiting a seed
	double prev_pos[3];
	int steps;
	double destination[2];

	void robot::controller()
	{
		color[0] = 1;
		color[1] = 1;
		color[2] = 1;
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
				evadeObstacle(false);
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
		tx_request = 1;
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

		//let's see if we are stuck
		if ((steps % 200) == 0)
		{
			if (distance(prev_pos[0], prev_pos[1], pos[0], pos[1]) < min_movement)
			{
				steps = 0;
				behavior = evading; //evade the obstacle
				evadeObstacle(true);
				return;
			}
			//let's record our new position
			recordPosition();
			steps = 0;
		}

		// let's find the direction we need to move to
		double target_tetha = find_theta(pos[X], pos[Y], destination[0], destination[1]);
		// lets move 
		motor_command = defineAction(pos[T],target_tetha);
	}

	int defineAction(double at, double tt)
	{
		double td = robot::tetha_diff(at, tt);
		if ((td > -.1 ) && (td < .1))
		{
			// hurray we are pointing in the right direction!
			return forward;
		}
		// terrible luck today... let's correct the direction
		if (td>0)
		{
			return left;
		}
		else
		{
			return right;
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
		tx_request = 1;
	}

	void evadeObstacle(bool reset)
	{
		static int rnd_steps;
		static int rnd_direction;
		if (reset) 
		{ 
			rnd_steps = timer % 200; 
			rnd_direction = (timer % 2 + 2);
		}
		if (steps < rnd_steps)
		{
			motor_command = rnd_direction;
		}
		else if (steps < rnd_steps * 3)
		{
			motor_command = forward;
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

	bool robot::comm_out_criteria(double x, double y)
	{
		static double diameter = 2 * radius+1;
		if (x < pos[0] - diameter || x > pos[0] + diameter || y < pos[1] - diameter || y > pos[1] + diameter) return false;
		if (robot::distance(pos[0], pos[1], x, y) > diameter) return false; //robot within com range, put transmitting robots data in its data_in struct
		double theta = find_theta(pos[0], pos[1], x, y);
		double td = robot::tetha_diff(pos[2], theta);
		if (abs(td)<.5)
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

};