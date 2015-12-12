#pragma once
enum wifi_action
{
	bid,
	lead,
	receive,
	finish
};
//communication data struct
struct wifi_data {
	int id;
	wifi_action action;
	int data1;
	int data2;
	int data3[100];
};
