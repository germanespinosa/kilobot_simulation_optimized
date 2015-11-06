#ifndef ROBOT_H
#define ROBOT_H
#define motion_error_std .007

class robot
{
public:

	double pos[3];//x,y,theta position in real world, dont use these in controller, thats cheating!!
	double motor_error;//value of how motors differ from ideal, dont use these, thats cheating!!
	double comm_range = 60; //communication range between robots

	double gaus_rand[100];

	double color[3]; //robot color output, values 0-1

	//robot commanded motion 1=forward, 2=cw rotation, 3=ccw rotation, 4=stop
	int motor_command;

	//function initalized varaibles
	void init(int, int, int);

	//must implement an robot initialization
	virtual void init_robot() = 0;

	//robots internal timer
	int timer;

	double gaussrand();

	//must implement the controller
	virtual void controller() = 0;

	//flag set to 1 when robot wants to transmitt
	int tx_request;
		
	//flag set to 1 when new message received
	int incoming_message_flag;
	int id;
	int hop;
	int previous_distance;

	//communication data struct
	struct communcation_data {
		int message;
		int id;
		double distance;
	};

	//received data goes here
	communcation_data data_in;

	//data to transmitt goes here
	communcation_data data_out;

	virtual bool comm_out_criteria(double x, double y);
	virtual bool comm_in_criteria(double x, double y);

};
#endif