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
#define tolerance 10
#define PI 3.14159265358979324


class FigureX
{
public:
	int disks = 38;
	int disks_size[38] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
	int disks_delay[38] = { 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10 };
	int disks_center_x[38] = { -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,-9,-8,-7,-6,-5,-4,-3,-2,-1,0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
	int disks_center_y[38] = { -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 8, 7, 6, 5, 4, 3, 2, 1,0,-1,-2,-3,-4,-5,-6,-7,-8,-9};
	double figure_x = 1600;
	double figure_y = 1600;
	int figure_scale = 100;
	int figure_thickness = 4;
	int disks_status[20];
	int disks_ids[20];
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

	int diskX(int i)
	{
		return f.figure_x + f.disks_center_x[i] * f.figure_scale;
	}
	int diskY(int i)
	{
		return f.figure_y + f.disks_center_y[i] * f.figure_scale;
	}
	int diskSize(int i)
	{
		return f.disks_size[i] * f.figure_thickness;
	}

	void findClosestDisk()
	{
		double disttocenter = -1;
		double disttorobot = -1;
		closest_disk = -1;
		for (int i = 0;i < f.disks;i++)
		{
			if (f.disks_status[i]==0)
			{

				double newdisttocenter = distance(0,0,f.disks_center_x[i],f.disks_center_y[i]);
				if (disttocenter >= newdisttocenter || closest_disk == -1)
				{
					double newdisttorobot = distance(pos[X], pos[Y], diskX(i), diskY(i));
					if (disttocenter > newdisttocenter || disttorobot > newdisttorobot || closest_disk == -1)
					{
						disttorobot = newdisttorobot;
						disttocenter = newdisttocenter;
						closest_disk = i;
					}
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
		setDestination(diskX(closest_disk), diskY(closest_disk));
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

		int msg = f.disks_delay[closest_disk];
		msg = msg << 5;
		msg = msg + diskSize(closest_disk);
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
		data_out.message = signal_smart + signal_gradient + diskSize(closest_disk) ;
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

	double robot::comm_out_criteria(int c, double x, double y, int sd)
	{
		switch (c)
		{
			case 1:
			{
				if (sd) return 0;
				static double diameter = 2 * radius + 5;
				if (x < pos[0] - diameter || x > pos[0] + diameter || y < pos[1] - diameter || y > pos[1] + diameter) return 0;
				double dist = robot::distance(pos[0], pos[1], x, y);
				if (dist <= diameter) return dist; //robot within com range, put transmitting robots data in its data_in struct
				break;
			}
			case 2:
			{
				return -1;
			}
		}
		return 0;
	}
	bool robot::comm_in_criteria(int c, double x, double y, communcation_data cd) //omnidirectional
	{
		data_in = cd;
		switch (c)
		{
			case 1:
			{
				return true;
			}
			case 2:
			{
				return true;
			}
		}
		return false;
	}
	void robot::sensing(int features, int type[], int x[], int y[], int value[])
	{
	}
};

