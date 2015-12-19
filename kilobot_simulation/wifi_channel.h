#pragma once
#define wifi 2

enum wifi_action
{
	bid,
	receive,
	assign,
	finish
};
//communication data struct
struct wifi_data {
	int id;
	int destination;
	wifi_action action;
	int int_data1;
	int int_data2;
	double dbl_data1;
	double dbl_data2;
};
