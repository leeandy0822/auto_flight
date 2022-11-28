#include<iostream>
#include<thread>
#include"ros/ros.h"
#include<serial.hpp>

#include"thread/main_thread.h"

using namespace std;

transportation_t tx_data;

void transport_callback(const auto_flight::transportation::ConstPtr& msg){
	tx_data.tracker_id = msg->tracker_id;
	tx_data.px = msg->px;
	tx_data.py = msg->py;
	tx_data.pz = msg->pz;
	tx_data.qx = msg->qx;
	tx_data.qy = msg->qy;
	tx_data.qz = msg->qz;
	tx_data.qw = msg->qw;
	tx_data.roll = msg->roll;
	tx_data.pitch = msg->pitch;	
	tx_data.thrust = msg->thrust;	
}


int main(int argc ,char **argv){

	ros::init( argc, argv,"ncrl_auto_flght");
	ros::Time::init();
   	ros::Rate loop_rate(100);
	ros::NodeHandle nh;
	ros::Subscriber sub;

	std::string port_name;
	std::string node_name;

	nh.getParam("/uav1/xbee_transform/port_name", port_name);      
	nh.getParam("/uav1/xbee_transfrom/node_name", node_name);

	port_name = "/dev/ttyUSB0";
	cout << port_name << endl;
	
	cout << "main start" << endl;           

	char port_name_arr[port_name.length() + 1]; 
	strcpy(port_name_arr, port_name.c_str()); 

	serial_init((char *)port_name_arr, 115200);
	sub = nh.subscribe<auto_flight::transportation>(node_name,1, transport_callback);
	
	/* init tx_data */

	tx_data.tracker_id = 1;
	tx_data.px = 1;
	tx_data.py = 2;
	tx_data.pz = 3;
	tx_data.qx = 0;
	tx_data.qy = 0;
	tx_data.qz = 0;
	tx_data.qw = -1;
	tx_data.roll = 0.1;
	tx_data.pitch = 0.01;	
	tx_data.thrust = 0.3;	

	while(ros::ok()){
		
		send_pose_to_serial(tx_data.tracker_id, tx_data.px , tx_data.py, tx_data.pz,
			 tx_data.qx, tx_data.qy, tx_data.qz, tx_data.qw, \
			 tx_data.roll, tx_data.pitch, tx_data.thrust);

		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

