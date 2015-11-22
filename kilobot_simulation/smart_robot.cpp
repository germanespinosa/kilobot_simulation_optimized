#include "robot.h"
#include <iostream>

#define signal_basic 65536
#define signal_smart 32768
#define signal_gradient 16384
#define signal_recruit 8192
#define signal_claim 4096
#define mask_size 31
#define mask_delay 992


#define min_movement 40
#define tolerance 50
#define PI 3.14159265358979324

#define radius 20
#define X 0
#define Y 1
#define T 2


class FigureHorseShoe
{
public:
	int disks = 23;
	int disks_size[23] = { 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10 };
	int disks_center_x[23] = { 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1950, 2100, 2300, 2500, 2700, 2900, 3100, 3300, 3450, 3600, 3600, 3600, 3600, 3600, 3600, 3600 };
	int disks_center_y[23] = { 1800, 2000, 2200, 2400, 2600, 2800, 3000, 3150, 3300, 3300, 3300, 3300, 3300, 3300, 3300, 3150, 3000, 2800, 2600, 2400, 2200, 2000, 1800 };
	int disks_status[23];
	int disks_ids[23];
};

class FigureX
{
public:
	int disks = 20;
	int disks_size[20] = { 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10 };
	int disks_wait[20] = { 15,15,10,10,1,1,10,10,15,15,15,15,10,10,1,1,10,10,15,15 };
	int disks_center_x[20] = { 1800, 2000, 2200, 2400, 2600, 2800, 3000, 3200, 3400, 3600, 1800, 2000, 2200, 2400, 2600, 2800, 3000, 3200, 3400, 3600 };
	int disks_center_y[20] = { 1800, 2000, 2200, 2400, 2600, 2800, 3000, 3200, 3400, 3600, 3600, 3400, 3200, 3000, 2800, 2600, 2400, 2200, 2000, 1800 };
	int disks_status[20];
	int disks_ids[20];
};

class FigureLine
{
public:
	int disks = 23;
	int disks_size[23] = { 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10 };
	int disks_center_x[23] = { 1800, 2000, 2200, 2400, 2600, 2800, 3000, 3200, 3400, 3600, 1800, 2000, 2200, 2400, 2600, 2800, 3000, 3200, 3400, 3600 };
	int disks_center_y[23] = { 1800, 2000, 2200, 2400, 2600, 2800, 3000, 3200, 3400, 3600, 3600, 3400, 3200, 3000, 2800, 2600, 2400, 2200, 2000, 1800 };
	int disks_status[23];
	int disks_ids[23];
};

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

	FigureX f;

	int closest_disk = -1;
	behavior_enum behavior = finding; // 1 - identify the closest circle center 2 - moving toward a circle center 3 - recruiting a seed
	double prev_pos[3];
	int steps;
	double destination[2];

	void robot::controller()
	{
		color[0] = 1;
		color[1] = 0;
		color[2] = 0;
		checkIncoming();
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

		for each(int i in f.disks_status)
			i = 0;
	}

	void findClosestDisk()
	{
		double dist = 0;
		closest_disk = -1;
		for (int i = 0;i < f.disks;i++)
		{
			if (f.disks_status[i]==0)
			{
				double newdist = distance(f.disks_center_x[i], f.disks_center_y[i], pos[X], pos[Y]);
				if (dist > newdist || closest_disk == -1)
				{
					dist = newdist;
					closest_disk = i;
				};
			};
		}
		if (closest_disk == -1) // Let's go back to base
		{
			destination[0] = 2500;
			destination[1] = 200;
			behavior = moving;
			return;
		}
		//let's record our position to know if we are moving
		claimDisk(closest_disk);
		recordPosition();
		setDestination(f.disks_center_x[closest_disk], f.disks_center_y[closest_disk]);
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
		if (incoming_message_flag==1)
		{
			incoming_message_flag = 0;
			if (data_in.message == signal_basic + signal_gradient)
			{
				f.disks_ids[closest_disk] = data_in.id;
				steps = 0;
				behavior = finding;
				return;
			}
		}

		int msg = f.disks_wait[closest_disk];
		msg = msg << 5;
		msg = msg + f.disks_size[closest_disk];
		data_out.id = id;
		data_out.message = signal_smart + signal_recruit + msg;
		tx_request = 1;
	}
	void moveToDestination()
	{
		// find out if we are there yet
		if (distance(pos[X], pos[Y], destination[0], destination[1]) < tolerance)
		{
			//hurray we are there!
			if (closest_disk >= 0)
			{
				claimDisk(closest_disk);
				behavior = recruiting; //recruiting
			}
			else
			{
				behavior = finish;
			}
			motor_command = 4;
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

	void claimDisk(int diskId)
	{
		f.disks_status[diskId] = 1;
		data_out.id = id;
		data_out.message = signal_smart + signal_claim + diskId;
		tx_request = 2;
	}

	void registerSeed()
	{
		if (incoming_message_flag==1)
		{
			incoming_message_flag = 0;
			if (data_in.message & signal_basic && data_in.message & signal_recruit && data_in.message & signal_gradient)
			{
				if (f.disks_ids[closest_disk] == data_in.id)
				{
					f.disks_status[closest_disk] = true;
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
		data_out.message = signal_smart + signal_gradient + f.disks_size[closest_disk];
		tx_request = 1;
	}

	void checkIncoming()
	{
		if (incoming_message_flag == 2)
		{
			incoming_message_flag = 0;
			if (data_in.message && signal_claim)
			{
				int diskId = data_in.message - signal_smart - signal_claim;
				if (f.disks_status[diskId] == 1)
				{
					if (data_in.id > id)
					{
						behavior = finding;
						f.disks_status[diskId] = 2;
					}
					else
					{
						claimDisk(diskId);
					}
				}
				else
				{
					f.disks_status[diskId] = 2;
				}
			}
		}
	}

	void evadeObstacle(bool reset)
	{
		static int rnd_steps;
		static int rnd_direction;
		if (reset) 
		{ 
			rnd_steps = timer % 199; 
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

	bool robot::comm_out_criteria(int c, double x, double y)
	{
		switch (c)
		{
			case 1:
			{
				static double diameter = 2 * radius + 1;
				if (x < pos[0] - diameter || x > pos[0] + diameter || y < pos[1] - diameter || y > pos[1] + diameter) return false;
				if (robot::distance(pos[0], pos[1], x, y) > diameter) return false; //robot within com range, put transmitting robots data in its data_in struct
				return true;
			}
			case 2:
			{
				return true;
			}
		}
		return false;
	}
	bool robot::comm_in_criteria(int c, double x, double y) //omnidirectional
	{
		switch (c)
		{
			case 1:
			{
				if (gauss_rand(timer) < .90) return true;
			}
			case 2:
			{
				return true;
			}
		}
		return false;
	}
};

