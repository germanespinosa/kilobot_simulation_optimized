#include <iostream>
#include "robot.h"

void robot::robot_controller()
{
	timer++;
	controller();
}

void robot::robot_init(double x, double y, double t)
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
	rand();
	motor_error = robot::gauss_rand(timer)*motion_error_std;
	init();
}