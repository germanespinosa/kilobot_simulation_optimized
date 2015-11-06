#include <iostream>
#include "robot.h"

void robot::init(int x, int y, int t)
{
	//initalize robot variables
	pos[0] = x;
	pos[1] = y;
	pos[2] = t;
	motor_command = 0;
	timer = rand() / 100;
	incoming_message_flag = 0;
	tx_request = 0;
	id = rand();
	hop = 255;
	rand();
	motor_error = gaussrand()*motion_error_std;
	init_robot();
	for (int i = 0; i < 100;i++)
	{
		gaus_rand[i] = gaussrand();
	};
}

double robot::gaussrand()
{
	static double V1, V2, S;
	static int phase = 0;
	double X;

	if (phase == 0) {
		do {
			double U1 = (double)rand() / RAND_MAX;
			double U2 = (double)rand() / RAND_MAX;

			V1 = 2 * U1 - 1;
			V2 = 2 * U2 - 1;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1 || S == 0);

		X = V1 * sqrt(-2 * log(S) / S);
	}
	else
		X = V2 * sqrt(-2 * log(S) / S);

	phase = 1 - phase;

	return X;
}
bool robot::comm_out_criteria(double x, double y) //stardard circular transmission area
{
	double gaus_comm_range = comm_range + gaus_rand[timer % 100] * comm_range;
	if (x < pos[0] - gaus_comm_range || x > pos[0] + gaus_comm_range || y < pos[1] - gaus_comm_range ||y > pos[1] + gaus_comm_range) return false;
	double distance = sqrt((x-pos[1])*(x-pos[0])+(y-pos[1])*(y-pos[1]));
	return distance < gaus_comm_range; //robot within com range, put transmitting robots data in its data_in struct
}
bool robot::comm_in_criteria(double x, double y) //omnidirectional
{
	return true;
}

