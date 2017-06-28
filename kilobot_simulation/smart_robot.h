#include <Windows.h>
#include "robot.h"
#include "touch_channel.h"
#include "wifi_channel.h"
#include <iostream>
#include <string>

#define time_out 100

#define forced_reshuffle 3000
#define signal_basic 65536
#define signal_smart 32768
#define signal_gradient 16384
#define signal_recruit 8192
#define signal_claim 4096
#define mask_size 31
#define mask_delay 992


#define min_movement 40
#define tolerance 16
#define PI 3.14159265358979324


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

	behavior_enum behavior = behavior_enum::bidding;
	static double figure_x ;
	static double figure_y ;
	static int figure_scale ;
	static int figure_thickness ;

	static int disks ;
	static int disks_priority[100] ;
	static int disks_size[100] ;
	static int disks_delay[100] ;
	static int disks_center_x[100] ;
	static int disks_center_y[100] ;

	static int bases ;
	static int bases_x[4] ;
	static int bases_y[4] ;
	
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

	int disks_status[100];
	int disks_ids[100];

	int closest_disk = -1;
	double prev_pos[3];
	int steps;

	int bid_leader = 0;

	int s_robots = 0;
	int s_rob_id[100];
	double s_rob_pos[100][2];
	int s_rob_disk[100];

	int next_forced_reshuffle = 0;

public:
	static void load_shape(char *filename);
	void robot::controller();
	char *robot::get_debug_info(char *buffer, char *rt);
	void *robot::get_message(int c);
	void robot::sensing(int features, int type[], int x[], int y[], int value[]);
	bool robot::comm_in_criteria(int c, double x, double y, double d, void *cd);
	double robot::comm_out_criteria(int c, double x, double y, int sd);
	void set_color();
	void send_assigned();
	void robot::init();
	double diskX(int i);
	double diskY(int i);
	int diskSize(int i);
	void findClosestDisk();
	void setDestination(double x, double y);
	void recruitBasicRobot();
	void moveToDestination();
	void checkIncoming();
	void gotobase();
	void evadeObstacle(bool reset);
	void recordPosition();
	int defineAction(double at, double tt);
};