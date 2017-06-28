#include "smart_robot.h"


	double smart_robot::figure_x = 1200;
	double smart_robot::figure_y = 1200;
	int smart_robot::figure_scale = 100;
	int smart_robot::figure_thickness = 4;

	int smart_robot::disks = 37;
	int smart_robot::disks_priority[100] = { 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1 };
	int smart_robot::disks_size[100] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
	int smart_robot::disks_delay[100] = { 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10 };
	int smart_robot::disks_center_x[100] = { -9,-8,-7,-6,-5,-4,-3,-2,-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,-9,-8,-7,-6,-5,-4,-3,-2,-1, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
	int smart_robot::disks_center_y[100] = { -9,-8,-7,-6,-5,-4,-3,-2,-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 8, 7, 6, 5, 4, 3, 2, 1,-1,-2,-3,-4,-5,-6,-7,-8,-9 };

	int smart_robot::bases = 4;
	int smart_robot::bases_x[4] = { -11,11,  0, 0 };
	int smart_robot::bases_y[4] = { 0, 0,-11,11 };

	void smart_robot::load_shape(char *filename)
	{

		FILE *shape;
		fopen_s(&shape, filename, "r");
		if (shape)
		{

			fscanf_s(shape, "%d,%d,%lf,%lf,%d,%d\n", &disks, & bases, &figure_x, &figure_y, &figure_scale, &figure_thickness);
			for (int i = 0;i < disks;i++)
			{
				fscanf_s(shape, "%d,%d,%d,%d,%d\n", &disks_center_x[i], &disks_center_y[i], &disks_priority[i], &disks_size[i], &disks_delay[i]);
			}
			for (int i = 0;i < bases;i++)
			{
				fscanf_s(shape, "%d,%d\n", &bases_x[i], &bases_y[i]);
			}
			fclose(shape);
		}
	}

	void smart_robot::controller()
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
			next_forced_reshuffle = 0;
			disks_status[closest_disk] = 4;
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
		smart_robot::set_color();
		incoming_message_flag = 0;
		steps++;
	}

	void smart_robot::set_color()
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

	void smart_robot::send_assigned()
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

	void smart_robot::init()
	{
		battery = 360 * 60 * radius * (1 + gauss_rand(timer)*.2);
		dest[0] = -1;
		dest[1] = -1;
		collide = false;
		size = 16;
		prerender();
		for (int i = 0;i < disks;i++)
			disks_status[i] = 0;
	}

	double smart_robot::diskX(int i)
	{
		return smart_robot::figure_x + disks_center_x[i] * figure_scale;
	}
	double smart_robot::diskY(int i)
	{
		return figure_y + disks_center_y[i] * figure_scale;
	}
	int smart_robot::diskSize(int i)
	{
		return disks_size[i] * figure_thickness;
	}

	void smart_robot::findClosestDisk()
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
			for (int i = 0;i < disks;i++)
			{
				if (disks_status[i] == 0 && (priority >= disks_priority[i] || selD == -1))
				{
					double newdist = distance(s_rob_pos[selR][0], s_rob_pos[selR][1], diskX(i), diskY(i));
					if (newdist < dist || selD == -1 || priority > disks_priority[i])
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
							priority = disks_priority[i];
						}
					}
				}
			}
			if (selD == -1)
			{
				s_rob_disk[selR] = -2; //no disk for this guy
			}
			else {
				s_rob_disk[selR] = selD;
			}
			assigned = 0;
			for (int i = 0;i < s_robots;i++)
				if (s_rob_disk[i] != -1) assigned++;
		}
		recordPosition();
		s_robots--;
		closest_disk = s_rob_disk[s_robots];
		if (closest_disk >= 0)
		{
			setDestination(diskX(closest_disk), diskY(closest_disk));
		}else
		{
			gotobase();
		}
	}

	void smart_robot::setDestination(double x, double y)
	{
		next_forced_reshuffle = timer + forced_reshuffle;
		dest[X] = x;
		dest[Y] = y;
		behavior = behavior_enum::moving;
	}

	void smart_robot::recruitBasicRobot()
	{
		color[0] = 0;
		color[1] = 0;
		color[2] = 1;
		//let's check if somebody received the message, we have a possible seed
		if (incoming_message_flag & touch)
		{
			incoming_message_flag = incoming_message_flag & !touch;
			if (data_in.action==touch_action::accepted)
			{
				disks_ids[closest_disk] = data_in.id;
				disks_status[closest_disk] = 4;
				steps = 0;
				behavior = bidding;
				return;
			}
		}
		data_out.id = id;
		data_out.action=touch_action::recruit_seed;
		data_out.data1 = diskSize(closest_disk);
		data_out.data2 = closest_disk;
		data_out.int_data = disks_delay[closest_disk] * 3000;
		tx_request = tx_request | touch;
	}
	void smart_robot::moveToDestination()
	{
		// find out if we are there yet
		if (distance(pos[X], pos[Y], dest[X], dest[Y]) < tolerance)
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
			if (distance(prev_pos[X], prev_pos[Y], pos[X], pos[Y]) < min_movement)
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
		double target_tetha = find_theta(pos[X], pos[Y], dest[X], dest[Y]);
		// lets move 
		motor_command = defineAction(pos[T], target_tetha);
	}

	int smart_robot::defineAction(double at, double tt)
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

	void smart_robot::checkIncoming()
	{
		incoming_message_flag = incoming_message_flag - wifi;
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
				if (closest_disk >= 0)
				{
					setDestination(diskX(closest_disk), diskY(closest_disk));
				}
				else //let's go to the cclosest base
				{
					gotobase();
				}
			}
			break;
		}
		case wifi_action::finish:
		{
			disks_status[wifi_in.int_data1] = 4;
			break;
		}
		}
	}

	void smart_robot::gotobase()
	{
		int closest_base = 0;
		for (int i = 1;i < bases;i++)
		{
			if (distance(pos[0], pos[1], smart_robot::figure_x + figure_scale *smart_robot::bases_x[closest_base], figure_y + figure_scale *bases_y[closest_base]) > distance(pos[0], pos[1], smart_robot::figure_x + figure_scale *smart_robot::bases_x[i], figure_y + figure_scale *bases_y[i]))
			{
				closest_base = i;
			}
		}
		setDestination(smart_robot::figure_x + figure_scale *bases_x[closest_base], figure_y + figure_scale *bases_y[closest_base]);
	}

	void smart_robot::evadeObstacle(bool reset)
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
	void smart_robot::recordPosition()
	{
		prev_pos[X] = pos[X];
		prev_pos[Y] = pos[Y];
		prev_pos[T] = pos[T];
	}

	double thetaD(double t1, double t2)
	{
		double d = t1 - t2;
		while (d < -2 * PI) d +=  2 * PI;
		if (d < -PI) d = 2 * PI + d;
		while (d > 2 * PI) d -= 2 * PI;
		if (d > PI) d = 2 * PI - d;
		return fabs(d);
	}

	double smart_robot::comm_out_criteria(int c, double x, double y, int sd)
	{
		switch (c)
		{
		case 1:
		{
			if (sd) return 0;
			static double diameter = 2 * radius + 10;
			if (x < pos[0] - diameter || x > pos[0] + diameter || y < pos[1] - diameter || y > pos[1] + diameter) return 0;
			double dist = robot::distance(pos[0], pos[1], x, y);
			if ((dist <= diameter) && (dist > diameter - 10))
			{
				double th = atan2(pos[1]-y, pos[0]-x);
				th += PI;
				//double th = atan2(y - pos[1], x - pos[0]);
				double d = thetaD(th, pos[2]);
				if (d<.5)
					return dist; //robot within com range, put transmitting robots data in its data_in struct
				else
					return 0;
			}
			break;
		}
		case 2:
		{
			return -1;
		}
		}
		return 0;
	}
	bool smart_robot::comm_in_criteria(int c, double x, double y, double d, void *cd) //omnidirectional
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
	void smart_robot::sensing(int features, int type[], int x[], int y[], int value[])
	{
	}
	void *smart_robot::get_message(int c)
	{
		if (c == 1)
			return (void *)&data_out;
		else
			return (void *)&wifi_out;
	}
	char *smart_robot::get_debug_info(char *buffer, char *rt)
	{
		sprintf_s(buffer,255, "%s, smart, %d, %4.2f, %4.2f, %d, %d, %4.2f, %4.2f\n", rt, id, pos[0], pos[1], smart_robot::behavior, closest_disk, dest[0], dest[1]);
		*buffer = '\0';
		return buffer;
	}
