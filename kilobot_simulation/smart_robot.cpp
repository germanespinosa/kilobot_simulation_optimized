#include "robot.h"
#include "touch_channel.h"
#include "wifi_channel.h"
#include <iostream>

#define time_out 100


#define forced_reshuffle 1000
#define signal_basic 65536
#define signal_smart 32768
#define signal_gradient 16384
#define signal_recruit 8192
#define signal_claim 4096
#define mask_size 31
#define mask_delay 992


#define min_movement 40
#define tolerance 5
#define PI 3.14159265358979324


class FigureX
{
public:
	int disks = 37;
	int disks_priority[37]  = { 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1 };
	int disks_size[37] =      { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
	int disks_delay[37] =    { 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10 };
	int disks_center_x[37] = { -9,-8,-7,-6,-5,-4,-3,-2,-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,-9,-8,-7,-6,-5,-4,-3,-2,-1, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
	int disks_center_y[37] = { -9,-8,-7,-6,-5,-4,-3,-2,-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 8, 7, 6, 5, 4, 3, 2, 1,-1,-2,-3,-4,-5,-6,-7,-8,-9 };
	int disks_status[37];
	int disks_ids[37];
	double figure_x = 1200;
	double figure_y = 1200;
	int figure_scale = 100;
	int figure_thickness = 4;

};



class smart_robot : public robot
{
	enum behavior_enum
	{
		bidding,
		assigning,
		receiving,
		moving,
		evading,
		recruiting,
		registering,
		finish
	};

	//comms
	//received data goes here
	touch_data data_in;
	//data to transmitt goes here
	touch_data data_out;

	//comms
	//received data goes here
	wifi_data wifi_in;
	//data to transmitt goes here
	wifi_data wifi_out;

	FigureX f;

	int closest_disk = -1;
	behavior_enum behavior = behavior_enum::bidding;
	double prev_pos[3];
	int steps;
	double destination[2];

	int bid_leader = 0;

	int s_robots = 0;
	int s_rob_id[100];
	double s_rob_pos[100][2];
	int s_rob_disk[100];

	int next_forced_reshuffle = 0;

	void robot::controller()
	{
		motor_command = 4;
		if (next_forced_reshuffle && timer > next_forced_reshuffle)
		{
			behavior = behavior_enum::bidding;
			next_forced_reshuffle = 0;
		}
		if (incoming_message_flag & wifi) checkIncoming();
		switch (behavior)
		{
		case bidding:
		{
			if (bid_leader == 0)
			{
				steps = 0;
				s_robots = 0;
				bid_leader = rand();
			}
			if (steps > time_out) //I'm the winner
			{
				bid_leader = 0;
				behavior = behavior_enum::assigning;
				break;
			}
			wifi_out.action = wifi_action::bid; //bid for leadership
			wifi_out.destination = 0; //broadcast
			wifi_out.id = id;
			wifi_out.int_data1 = bid_leader;
			if (rand() <RAND_MAX * .2)
				tx_request = tx_request | wifi;
			break;
		}
		case receiving:
		{
			bid_leader = 0;
			s_robots = 0;
			if (steps > 2 * time_out) //looks like nobody is assigning
			{
				steps = 0;
				behavior = behavior_enum::bidding;
				break;
			}
			wifi_out.action = wifi_action::receive;
			wifi_out.destination = 0; //broadcast
			wifi_out.id = id;
			wifi_out.dbl_data1 = pos[0];
			wifi_out.dbl_data2 = pos[1];
			if (rand() <RAND_MAX * .2)
				tx_request = tx_request | wifi;
			break;
		}
		case assigning:
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
			s_robots = 0;
			f.disks_status[closest_disk] = 4;
			wifi_out.action = wifi_action::finish; //bid for leadership
			wifi_out.destination = 0; //broadcast
			wifi_out.id = id;
			wifi_out.int_data1 = closest_disk;
			if (rand() <RAND_MAX * .2)
				tx_request = tx_request | wifi;
			recruitBasicRobot();
			break;
		}
		case evading: //we are stuck... lets evade
		{
			evadeObstacle(false);
			break;
		}
		}
		send_assigned();
		set_color();
		steps++;
	}

	void set_color()
	{
		switch (behavior)
		{
		case behavior_enum::assigning:
		{
			color[0] = .5;
			color[1] = 0;
			color[2] = 0;
		}
		case behavior_enum::bidding:
		{
			color[0] = 0;
			color[1] = .5;
			color[2] = 0;
		}
		case behavior_enum::evading:
		{
			color[0] = .5;
			color[1] = .5;
			color[2] = .5;
		}
		case behavior_enum::finish:
		{
			color[0] = .5;
			color[1] = .5;
			color[2] = 0;
		}
		case behavior_enum::moving:
		{
			color[0] = .5;
			color[1] = .5;
			color[2] = .5;
		}
		case behavior_enum::receiving:
		{
			color[0] = .5;
			color[1] = 0;
			color[2] = .5;
		}
		case behavior_enum::recruiting:
		{
			color[0] = 0;
			color[1] = .5;
			color[2] = .5;
		}

		}
		if (tx_request)
		{
			color[0] += color[0];
			color[1] += color[1];
			color[2] += color[2];

		}
	}

	void send_assigned()
	{
		if (s_robots > 0 && behavior==behavior_enum::moving)
		{
			int selR = steps  % s_robots;
			wifi_out.action = wifi_action::assign;
			wifi_out.destination = s_rob_id[selR]; //specific
			wifi_out.int_data1 = s_rob_disk[selR];
			wifi_out.id = id;
			if (rand() <RAND_MAX * .2)
				tx_request = tx_request | wifi;

		}
	}

	void robot::init()
	{
		dest[0] = -1;
		dest[1] = -1;
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
		s_rob_id[s_robots] = id;
		s_rob_pos[s_robots][0] = pos[0];
		s_rob_pos[s_robots][1] = pos[1];
		s_robots++;

		for (int i = 0; i < s_robots;i++)
			s_rob_disk[i] = -1;

		int assigned = 0;
		while (assigned<s_robots)
		{
			int selR = -1;
			for (int i = 0; i < s_robots;i++)
				if (s_rob_disk[i] == -1)
				{
					selR = i;
					break;
				}

			int priority = -1;
			int selD = -1;
			double dist = -1;
			for (int i = 0;i < f.disks;i++)
			{
				if (f.disks_status[i] == 0 && (priority >= f.disks_priority[i] || selD == -1))
				{
					double newdist = distance(s_rob_pos[selR][0], s_rob_pos[selR][1], diskX(i), diskY(i));
					if (newdist < dist || selD == -1 || priority > f.disks_priority[i])
					{
						int prev = -1;
						for (int j = 0; j < s_robots;j++)
							if (s_rob_disk[j] == i)
							{
								prev = j;
								break;
							}
						if (prev != -1)
						{
							double prevdist = distance(s_rob_pos[prev][0], s_rob_pos[prev][1], diskX(i), diskY(i));
							if (prevdist > newdist)
							{
								s_rob_disk[prev] = -1;
								prev = -1;
							}
						}
						if (prev == -1)
						{
							selD = i;
							dist = newdist;
							priority = f.disks_priority[i];
						}
					}
				}
			}
			if (selD == -1)
			{
				break;
			}
			s_rob_disk[selR] = selD;
			assigned = 0;
			for (int i = 0;i < s_robots;i++)
				if (s_rob_disk[i] != -1) assigned++;
		}
		recordPosition();
		s_robots--;
		closest_disk = s_rob_disk[s_robots];
		setDestination(diskX(closest_disk), diskY(closest_disk));
		behavior = moving; // move toward the closest disk
	}

	void setDestination(double x, double y)
	{
		next_forced_reshuffle = timer + forced_reshuffle;
		dest[X] = x;
		dest[Y] = y;
		destination[X] = dest[X];
		destination[Y] = dest[Y];
		behavior = behavior_enum::moving;
	}

	void recruitBasicRobot()
	{
		color[0] = 0;
		color[1] = 0;
		color[2] = 1;
		//let's check if somebody received the message, we have a possible seed
		if (incoming_message_flag&touch)
		{
			incoming_message_flag = incoming_message_flag & !touch;
			if (data_in.message == signal_basic + signal_gradient)
			{
				f.disks_ids[closest_disk] = data_in.id;
				f.disks_status[closest_disk] = 4;
				steps = 0;
				behavior = bidding;
				return;
			}
		}

		int msg = f.disks_delay[closest_disk];
		msg = msg << 5;
		msg = msg + diskSize(closest_disk);
		data_out.id = id;
		data_out.message = signal_smart + signal_recruit + msg;
		tx_request = tx_request | touch;
	}
	void moveToDestination()
	{
		// find out if we are there yet
		if (distance(pos[X], pos[Y], destination[0], destination[1]) < tolerance)
		{
			//hurray we are there!
			if (closest_disk >= 0)
			{
				dest[0] = -1;
				dest[1] = -1;
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
		motor_command = defineAction(pos[T], target_tetha);
	}

	int defineAction(double at, double tt)
	{
		double td = robot::tetha_diff(at, tt);
		if ((td > -.1) && (td < .1))
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

	void checkIncoming()
	{
		incoming_message_flag = incoming_message_flag & !wifi;
		switch (wifi_in.action)
		{
		case wifi_action::bid:
		{
			if (behavior != behavior_enum::receiving && behavior != behavior_enum::recruiting)
			{
				if (behavior == behavior_enum::bidding)
				{
					if (wifi_in.int_data1 > bid_leader)
					{
						steps = 0;
						behavior = behavior_enum::receiving;
					}
				}
				else
				{
					behavior = behavior_enum::bidding;
				}
			}
			break;
		}
		case wifi_action::receive:
		{
			if (behavior == behavior_enum::bidding)
			{
				bool found = false;
				for (int i = 0;i < s_robots;i++)
				{
					if (s_rob_id[i] == wifi_in.id)
						found = true;
				}
				if (!found)
				{
					s_rob_id[s_robots] = wifi_in.id;
					s_rob_pos[s_robots][0] = wifi_in.dbl_data1;
					s_rob_pos[s_robots][1] = wifi_in.dbl_data2;
					s_robots++;
				}
			}
			break;
		}
		case wifi_action::assign:
		{
			if (behavior = behavior_enum::receiving)
			{
				closest_disk = wifi_in.int_data1;
				setDestination(diskX(closest_disk), diskY(closest_disk));
			}
			break;
		}
		case wifi_action::finish:
		{
			f.disks_status[wifi_in.int_data1] = 4;
			break;
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
	bool robot::comm_in_criteria(int c, double x, double y, double d, void *cd) //omnidirectional
	{
		switch (c)
		{
		case 1:
		{
			touch_data *ccd = (touch_data *)cd;
			data_in = *ccd;
			data_in.distance = d;
			return true;
		}
		case 2:
		{
			wifi_data *cwd = (wifi_data *)cd;
			if (!cwd->destination || cwd->destination == id)
				wifi_in = *cwd;
			return true;
		}
		}
		return false;
	}
	void robot::sensing(int features, int type[], int x[], int y[], int value[])
	{
	}
	void *robot::get_message(int c)
	{
		if (c == 1)
			return (void *)&data_out;
		else
			return (void *)&wifi_out;
	}
};

