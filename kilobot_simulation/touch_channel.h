#pragma once
#define touch 1
//communication data struct
struct touch_data {
	int action;
	int message;
	int id;
	double distance;
};
