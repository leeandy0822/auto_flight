#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <serial.hpp>
#include <mutex>
#include <cmath>
#include <vector>
#include <string>

#include "main_thread.h"


using namespace std;

transportation_t tx_data_1;
transportation_t tx_data_2;
transportation_t tx_data_3;

double last_exe_time_1 = 0 ; 
double last_exe_time_2 = 0 ; 
double last_exe_time_3 = 0 ; 

void callback1(const auto_flight::transportation::ConstPtr& msg){
	tx_data_1.tracker_id = msg->tracker_id;
	tx_data_1.tracker_id = 1;

	tx_data_1.px = msg->px;
	tx_data_1.py = msg->py;
	tx_data_1.pz = msg->pz;
	tx_data_1.qx = msg->qx;
	tx_data_1.qy = msg->qy;
	tx_data_1.qz = msg->qz;
	tx_data_1.qw = msg->qw;
	tx_data_1.roll = msg->roll;
	tx_data_1.pitch = msg->pitch;	
	tx_data_1.thrust = msg->thrust;	
}

void callback2(const auto_flight::transportation::ConstPtr& msg){
	tx_data_2.tracker_id = msg->tracker_id;
	tx_data_2.tracker_id = 15;

	tx_data_2.px = msg->px;
	tx_data_2.py = msg->py;
	tx_data_2.pz = msg->pz;
	tx_data_2.qx = msg->qx;
	tx_data_2.qy = msg->qy;
	tx_data_2.qz = msg->qz;
	tx_data_2.qw = msg->qw;
	tx_data_2.roll = msg->roll;
	tx_data_2.pitch = msg->pitch;	
	tx_data_2.thrust = msg->thrust;	
}

void callback3(const auto_flight::transportation::ConstPtr& msg){
	tx_data_3.tracker_id = msg->tracker_id;
	tx_data_3.tracker_id = 5;
	tx_data_3.px = msg->px;
	tx_data_3.py = msg->py;
	tx_data_3.pz = msg->pz;
	tx_data_3.qx = msg->qx;
	tx_data_3.qy = msg->qy;
	tx_data_3.qz = msg->qz;
	tx_data_3.qw = msg->qw;
	tx_data_3.roll = msg->roll;
	tx_data_3.pitch = msg->pitch;	
	tx_data_3.thrust = msg->thrust;	
}

int send_thread_entry_1(int serial_fd){

   	ros::Rate loop_rate(100);
	ros::NodeHandle nh1;
	ros::Subscriber sub1;

	sub1 = nh1.subscribe<auto_flight::transportation>("pc_to_pixhawk_1",1,callback1);
	void callback1(const auto_flight::transportation::ConstPtr& msg);

	while(ros::ok()){

		send_pose_to_serial_1(tx_data_1.tracker_id, tx_data_1.px , tx_data_1.py, tx_data_1.pz,
			 tx_data_1.qx, tx_data_1.qy, tx_data_1.qz, tx_data_1.qw, \
			 tx_data_1.roll, tx_data_1.pitch, tx_data_1.thrust, serial_fd);

		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;

}


int send_thread_entry_2(int serial_fd){

   	ros::Rate loop_rate(100);
	ros::NodeHandle nh2;
	ros::Subscriber sub2;

	sub2 = nh2.subscribe<auto_flight::transportation>("pc_to_pixhawk_2",1,callback2);
	void callback2(const auto_flight::transportation::ConstPtr& msg);

	while(ros::ok()){
		
		send_pose_to_serial_2(tx_data_2.tracker_id, tx_data_2.px , tx_data_2.py, tx_data_2.pz,
			 tx_data_2.qx, tx_data_2.qy, tx_data_2.qz, tx_data_2.qw, \
			 tx_data_2.roll, tx_data_2.pitch, tx_data_2.thrust, serial_fd);

		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;

}


int send_thread_entry_3(int serial_fd){

   	ros::Rate loop_rate(100);
	ros::NodeHandle nh3;
	ros::Subscriber sub3;

	sub3 = nh3.subscribe<auto_flight::transportation>("pc_to_pixhawk_3",1,callback3);
	void callback3(const auto_flight::transportation::ConstPtr& msg);

	while(ros::ok()){

		send_pose_to_serial_3(tx_data_3.tracker_id, tx_data_3.px , tx_data_3.py, tx_data_3.pz,
			 tx_data_3.qx, tx_data_3.qy, tx_data_3.qz, tx_data_3.qw, \
			 tx_data_3.roll, tx_data_3.pitch, tx_data_3.thrust ,serial_fd);

		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;

}
