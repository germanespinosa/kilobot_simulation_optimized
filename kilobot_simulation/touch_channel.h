#pragma once
#define touch 1

enum touch_action
{
	recruit_seed,
	recruit,
	border,
	accepted
};

//communication data struct
struct touch_data {
	touch_action action;
	unsigned char data1;
	unsigned char data2;
	int int_data;
	int id;
	double distance;
};
